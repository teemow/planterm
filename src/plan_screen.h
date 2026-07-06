#pragma once

// Pure, platform-free pGD screen reconstructor.
//
// Accumulates the display frames the controller sends into the current
// screen state of BOTH terminals on the bus: the physical pGD (0x20) and
// your own enrolled session (0x1F) -- the latter is what injected keys
// navigate, so it is the screen an automated reader works from.
//
// Feed it the received (byte, bit9) pairs from your transport, one at a
// time, from ordinary task context (no allocation, but no ISR guarantees);
// frames are delimited at the bit9 address marks, so this code sees the
// raw stream and needs none of the lossy-log salvage the planscope
// original carries.
//
// Frame handling (grammar per docs/protocol.md):
//   0x0B  text row      payload = ROW + chars; replaces the whole row
//   0x0C  single cell   payload = ROW COL CHAR (drifting digits, edit mode)
//   0x64  graphic band  inverse-video detection for menu-cursor verification
//   0x65  page sync     clears body bands; rows survive (delta repaint)
// Classic frames byte-sum to 0xFF; 0x64/0x65/0x66 carry a CRC-16/Modbus LE
// trailer instead.
//
// Host-tested in test/test_plan_screen.cpp by replaying an archived
// menu-walk capture.

#include "plan_frame.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace plan {

// The two terminals whose screens the controller paints.
static constexpr uint8_t SCR_TERM_PGD = 0x20;
static constexpr uint8_t SCR_TERM_ESP = 0x1F;

static constexpr uint8_t SCR_TYPE_TEXT = 0x0B;
static constexpr uint8_t SCR_TYPE_CELL = 0x0C;
static constexpr uint8_t SCR_TYPE_GRAPHIC = 0x64;
static constexpr uint8_t SCR_TYPE_INIT = 0x65;
static constexpr uint8_t SCR_TYPE_CTL = 0x66;

static constexpr size_t SCR_ROWS = 8;   // pGD text mode is 8 rows...
static constexpr size_t SCR_COLS = 22;  // ...of 22 columns
// CAREL's degree glyph. Kept verbatim in the row text (single byte, so the
// grid stays plain char[]); a field-extraction layer matches 0xDF where a
// renderer would show the UTF-8 '°'.
static constexpr uint8_t SCR_DEGREE = 0xDF;
// Graphics bands at pixel y < 8 are the title bar; y >= 8 is the body.
static constexpr int SCR_TITLE_BAR_PX = 8;
// LEN is a single byte < 250 (planscope's parser bound), so 256 holds any
// valid display frame, graphics included.
static constexpr size_t SCR_FRAME_MAX = 256;
// feed_graphic_ rejects bands with y > 64, so band state is indexed by y.
static constexpr int SCR_BAND_MAX_Y = 64;

// Charset mapping (planscope rowText): printable ASCII and the degree glyph
// pass through, everything else renders as '.'.
static inline char scr_char(uint8_t c) {
  if (c == SCR_DEGREE)
    return static_cast<char>(SCR_DEGREE);
  if (c >= 0x20 && c < 0x7F)
    return static_cast<char>(c);
  return '.';
}

class PlanScreen {
 public:
  // Terminal index for an address byte: 0 = pGD, 1 = own session, -1 = other.
  static int term_index(uint8_t addr) {
    if (addr == SCR_TERM_PGD)
      return 0;
    if (addr == SCR_TERM_ESP)
      return 1;
    return -1;
  }

  // Feed one received (byte, bit9) pair; now_ms stamps completed frames.
  // Returns true when the visible screen changed: a text-row repaint, a
  // graphics band, or a page sync. Single-cell updates (0x0C) do NOT count
  // -- they are value drift, and live pages emit them at ~1 Hz forever, so
  // counting them would make change-waiters never see a quiet page (they
  // still land in the rows and still advance painted_ms for settle).
  bool feed(uint8_t b, uint8_t bit9, uint32_t now_ms) {
    bool changed = false;
    if (bit9 != 0) {
      changed = close_(now_ms);
      cur_len_ = 0;
      cur_open_ = true;  // (joining mid-frame at boot: first partial run is skipped)
    }
    if (cur_open_) {
      if (cur_len_ >= SCR_FRAME_MAX)
        cur_open_ = false;  // overlong: not a valid display frame
      else
        cur_[cur_len_++] = b;
    }
    return changed;
  }

  // Row text, NUL-terminated, exactly SCR_COLS chars once painted ("" while
  // never painted -- matching planscope's absent-row semantics).
  const char *row(uint8_t addr, int r) const {
    int t = term_index(addr);
    if (t < 0 || r < 0 || r >= static_cast<int>(SCR_ROWS) || !row_set_[t][r])
      return "";
    return rows_[t][r];
  }

  // Whether a text row is covered by an inverse-video graphics band (the pGD
  // paints menu selection and title bars this way).
  bool row_inverse(uint8_t addr, int r) const {
    int t = term_index(addr);
    if (t < 0)
      return false;
    for (int y = 0; y <= SCR_BAND_MAX_Y; y++) {
      const Band &bd = bands_[t][y];
      if (bd.set && bd.inverse && r >= y / 8 && r < (y + bd.h + 7) / 8)
        return true;
    }
    return false;
  }

  // Last display frame for the terminal (ms clock of feed()); 0 = never.
  uint32_t painted_ms(uint8_t addr) const {
    int t = term_index(addr);
    return t < 0 ? 0 : painted_[t];
  }

  // Settle tracking: quiet on the terminal's display session for at least
  // quiet_ms means the repaint is done (planscope's settleQuiet is 600 ms).
  // False while never painted -- there is nothing settled to read.
  bool settled(uint8_t addr, uint32_t now_ms, uint32_t quiet_ms) const {
    int t = term_index(addr);
    return t >= 0 && painted_[t] != 0 && now_ms - painted_[t] >= quiet_ms;
  }

  uint32_t frames() const { return frames_; }
  uint32_t bad() const { return bad_; }

 protected:
  struct Band {
    uint8_t h{0};
    bool inverse{false};
    bool set{false};
  };

  // CRC-16/Modbus (reflected poly 0xA001, init 0xFFFF). Residue property:
  // over a whole frame INCLUDING its little-endian trailer it yields 0.
  static uint16_t crc16_(const uint8_t *d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
      crc = static_cast<uint16_t>(crc ^ d[i]);
      for (int k = 0; k < 8; k++)
        crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
  }

  // Validate and digest one completed bit9-delimited run as a display frame:
  //   ADDR TYPE LEN 01 <payload> CK    LEN = total frame length
  // (For 0x20 frames the pGD's ack trailer 01' 03 20 DB is its own run, so
  // the run is exactly the frame; your own acks never appear in your RX.)
  bool close_(uint32_t now_ms) {
    if (!cur_open_ || cur_len_ < 6)
      return false;
    int t = term_index(cur_[0]);
    if (t < 0 || cur_[3] != 0x01 || cur_[2] != cur_len_)
      return false;
    uint8_t typ = cur_[1];
    frames_++;
    bool crc_type = typ == SCR_TYPE_GRAPHIC || typ == SCR_TYPE_INIT || typ == SCR_TYPE_CTL;
    bool ck_ok = crc_type ? (crc16_(cur_, cur_len_) == 0) : (sum8(cur_, cur_len_) == 0xFF);
    if (!ck_ok) {
      bad_++;
      return false;
    }
    painted_[t] = now_ms;
    switch (typ) {
      case SCR_TYPE_TEXT:
        // payload = ROW + chars (frame minus ADDR TYPE LEN 01 ... CK)
        return set_row_(t, cur_[4], cur_ + 5, cur_len_ - 6);
      case SCR_TYPE_CELL:
        // single-cell update ROW COL CHAR: how the controller repaints one
        // drifting digit, and the ONLY repaint an edit-mode value change or
        // a PIN digit gets (planscope ground truth 2026-07-03)
        if (cur_len_ == 8)
          set_cell_(t, cur_[4], cur_[5], cur_[6]);
        return false;
      case SCR_TYPE_GRAPHIC:
        feed_graphic_(t, cur_ + 4, cur_len_ - 6);
        return true;
      case SCR_TYPE_INIT:
        // 0x65 is a page-sync marker, NOT a blank-slate init: the burst after
        // it is a DELTA against the current screen (live-hardware ground
        // truth 2026-07-03), so rows survive; body bands clear like on a
        // page turn.
        clear_body_bands_(t);
        return true;
      default:
        return false;
    }
  }

  // Replace one row's text (a 0x0B frame repaints the whole row). Reports
  // whether the visible text changed; a row-0 change is a page transition,
  // which clears the body bands so the old page's selection band cannot
  // haunt the new page as a phantom highlight.
  bool set_row_(int t, uint8_t r, const uint8_t *chars, size_t n) {
    if (r >= SCR_ROWS)
      return false;
    char text[SCR_COLS + 1];
    for (size_t c = 0; c < SCR_COLS; c++)
      text[c] = c < n ? scr_char(chars[c]) : ' ';
    // ponytail: payloads beyond 22 columns are truncated; the pGD text mode
    // has no wider rows, and planscope has never rendered one.
    text[SCR_COLS] = '\0';
    if (row_set_[t][r] && std::memcmp(rows_[t][r], text, SCR_COLS) == 0)
      return false;
    if (r == 0)
      clear_body_bands_(t);
    std::memcpy(rows_[t][r], text, SCR_COLS + 1);
    row_set_[t][r] = true;
    return true;
  }

  // Paint one character cell. A cell update into a never-painted row lands
  // on spaces -- the controller only sends these against a screen it already
  // drew, so that covers attaching mid-page.
  bool set_cell_(int t, uint8_t r, uint8_t col, uint8_t ch) {
    if (r >= SCR_ROWS || col >= SCR_COLS)
      return false;
    if (!row_set_[t][r]) {
      std::memset(rows_[t][r], ' ', SCR_COLS);
      rows_[t][r][SCR_COLS] = '\0';
      row_set_[t][r] = true;
    }
    char c = scr_char(ch);
    if (rows_[t][r][col] == c)
      return false;
    rows_[t][r][col] = c;
    return true;
  }

  // Digest one 0x64 graphic-bitmap payload. Ground truth (menu navigation
  // capture 2026-07-02): 7 unknown/fragmentation bytes, then x,y,w,h as
  // 16-bit BE, then vertical-byte pixel data. The pGD renders menu selection
  // (and title bars) as inverse-video bands: a band painted mostly-lit is
  // inverse, mostly-dark is normal.
  // ponytail: fragment 2+ of a split band carries the same x/y/w/h, so
  // last-writer-wins per y is fine -- fragments of an inverse band are all
  // FF-heavy anyway.
  void feed_graphic_(int t, const uint8_t *p, size_t n) {
    if (n < 16)
      return;
    int y = (p[9] << 8) | p[10];
    int w = (p[11] << 8) | p[12];
    int h = (p[13] << 8) | p[14];
    if (w < 40 || h < 8 || h > 32 || y > SCR_BAND_MAX_Y)
      return;  // icons / large-font fragments, not a row band
    size_t px = n - 15;
    size_t set = 0;
    for (size_t i = 15; i < n; i++)
      for (uint8_t v = p[i]; v != 0; v &= v - 1)
        set++;
    Band b;
    b.h = static_cast<uint8_t>(h);
    b.inverse = set * 2 > px * 8;
    b.set = true;
    // Menu invariant: below the title bar at most one band is lit -- the
    // selection. The controller moves it by painting the old band dark and
    // the new one lit; newest lit band wins so a missed dark repaint cannot
    // leave two selections standing.
    if (b.inverse && y >= SCR_TITLE_BAR_PX) {
      for (int y2 = SCR_TITLE_BAR_PX; y2 <= SCR_BAND_MAX_Y; y2++)
        if (y2 != y && bands_[t][y2].inverse)
          bands_[t][y2].set = false;
    }
    bands_[t][y] = b;
  }

  // Drop every graphics band below the title bar (page transition / page
  // sync). Title-bar bands (y < 8) stay -- the status screen repaints row 0
  // every minute without touching its title band.
  void clear_body_bands_(int t) {
    for (int y = SCR_TITLE_BAR_PX; y <= SCR_BAND_MAX_Y; y++)
      bands_[t][y].set = false;
  }

  char rows_[2][SCR_ROWS][SCR_COLS + 1]{};  // [0] = pGD 0x20, [1] = own 0x1F
  bool row_set_[2][SCR_ROWS]{};
  Band bands_[2][SCR_BAND_MAX_Y + 1];
  uint32_t painted_[2]{0, 0};
  uint32_t frames_{0};
  uint32_t bad_{0};

  uint8_t cur_[SCR_FRAME_MAX];
  size_t cur_len_{0};
  bool cur_open_{false};  // false: mid-frame at start, or frame ran overlong
};

}  // namespace plan
