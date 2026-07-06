#pragma once

// Pure, platform-free screen-snapshot cache for a capture stream.
//
// Problem: the controller repaints only CHANGED rows, so a capture client
// that connects mid-session starts from a blank reconstruction -- a static
// menu page may not repaint for hours. A device that sits on the bus 24/7
// has seen every repaint, so it keeps the latest complete, checksum-valid
// display frame per (terminal, row) -- raw wire bytes, no screen decoding
// -- and replays them to a freshly connected capture client so the client
// paints both screens within a second of connecting. Text rows (0x0B) and
// numeric fields (0x0C) only; graphics bands (0x64) are ephemeral selection
// highlights, worthless minutes later, and 5x the RAM.
//
// Feed it the received (byte, bit9) pairs from ordinary task context (no
// allocation, but no ISR guarantees), one pair at a time; frames are
// delimited at the bit9 address marks. Host-tested in
// test/test_plan_snapshot.cpp.

#include "plan_frame.h"

#include <cstddef>
#include <cstdint>

namespace plan {

// The two terminals whose screens the controller paints: the physical pGD
// (0x20) and your own enrolled session (0x1F).
static constexpr uint8_t SNAP_TERM_PGD = 0x20;
static constexpr uint8_t SNAP_TERM_ESP = 0x1F;
static constexpr uint8_t SNAP_TEXT_TYPE = 0x0B;
static constexpr uint8_t SNAP_VAR_TYPE = 0x0C;
static constexpr size_t SNAP_ROWS = 8;    // pGD text mode is 8 rows
static constexpr size_t SNAP_VARS = 16;   // 0x0C selectors seen live: 0x00..0x08
// Longest cacheable frame: ADDR 0B LEN 01 ROW + 22 columns + CK = 28 bytes.
static constexpr size_t SNAP_FRAME_MAX = 32;
// The pGD's ack that follows every 0x20 display frame on the wire; a capture
// parser (planscope) anchors 0x20 frames on it, so the replay must carry it.
static const uint8_t SNAP_TRAILER[4] = {0x01, 0x03, 0x20, 0xDB};

class PlanSnapshot {
 public:
  // Feed one received (byte, bit9) pair; now_ms stamps a completed frame.
  void feed(uint8_t b, uint8_t bit9, uint32_t now_ms) {
    if (bit9 != 0) {
      close_(now_ms);
      cur_len_ = 0;
      cur_open_ = true;  // (joining mid-frame at boot: first partial run is skipped)
    }
    if (!cur_open_)
      return;
    if (cur_len_ >= SNAP_FRAME_MAX) {
      cur_open_ = false;  // overlong (graphics/session frame): not cacheable
      return;
    }
    cur_[cur_len_++] = b;
  }

  // Visit every cached frame: cb(bytes, len, at_ms, trailer) -- trailer is
  // true for 0x20 frames, whose replay must append SNAP_TRAILER.
  template <typename F> void visit(F &&cb) const {
    for (int t = 0; t < 2; t++) {
      for (size_t i = 0; i < SNAP_ROWS; i++)
        if (rows_[t][i].len > 0)
          cb(rows_[t][i].bytes, rows_[t][i].len, rows_[t][i].at_ms, t == 0);
      for (size_t i = 0; i < SNAP_VARS; i++)
        if (vars_[t][i].len > 0)
          cb(vars_[t][i].bytes, vars_[t][i].len, vars_[t][i].at_ms, t == 0);
    }
  }

 protected:
  // One cached frame; len == 0 means the slot was never painted.
  struct Slot {
    uint8_t len{0};
    uint32_t at_ms{0};
    uint8_t bytes[SNAP_FRAME_MAX];
  };

  // Cache a completed frame when it is a valid display frame:
  //   ADDR TYPE LEN 01 <payload> CK, LEN = total length, byte-sum 0xFF.
  void close_(uint32_t now_ms) {
    if (!cur_open_ || cur_len_ < 6)
      return;
    int t;
    if (cur_[0] == SNAP_TERM_PGD)
      t = 0;
    else if (cur_[0] == SNAP_TERM_ESP)
      t = 1;
    else
      return;
    if (cur_[3] != 0x01 || cur_[2] != cur_len_ || sum8(cur_, cur_len_) != 0xFF)
      return;
    Slot *s = nullptr;
    if (cur_[1] == SNAP_TEXT_TYPE && cur_[4] < SNAP_ROWS) {
      s = &rows_[t][cur_[4]];
      // A full-row repaint supersedes any cached cell drift on that row.
      // Without this, a 0x0C digit from a page long gone survives every
      // page change and gets replayed onto the new page's row -- planscope
      // showed ghost digits scattered across the menu after connecting.
      vars_[t][cur_[4]].len = 0;
    } else if (cur_[1] == SNAP_VAR_TYPE && cur_len_ == 8 && cur_[4] < SNAP_VARS)
      s = &vars_[t][cur_[4]];
    if (s == nullptr)
      return;
    s->len = static_cast<uint8_t>(cur_len_);
    s->at_ms = now_ms;
    for (size_t i = 0; i < cur_len_; i++)
      s->bytes[i] = cur_[i];
  }

  Slot rows_[2][SNAP_ROWS];  // [0] = pGD 0x20, [1] = own 0x1F; ~2.3 KB total
  // ponytail: vars keyed per ROW, so if two cells of one row drift between
  // row repaints only the last is replayed (a transient chimera value that
  // the next live drift corrects within ~1 s). Upgrade path: key by
  // (row, col) with a compact 8-byte slot.
  Slot vars_[2][SNAP_VARS];
  uint8_t cur_[SNAP_FRAME_MAX];
  size_t cur_len_{0};
  bool cur_open_{false};  // false: mid-frame at start, or frame ran overlong
};

}  // namespace plan
