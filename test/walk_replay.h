#pragma once

// Shared replay harness for the host tests: parse a planscope-format device
// log (`[HH:MM:SS.mmm] ... : [  N] 20' 0B ...` -- hex tokens, `'` = bit9
// mark) and feed the RX byte stream into a PlanScreen, calling back on
// every settled snapshot. C++ port of planscope's replayWalk
// (replay_test.go); works on an archived menu-walk capture as well as a
// fresh `planscope observe --raw` / `planscope log` tee.
//
// TX(9bit) lines are the emulated terminal's own transmissions, not part of
// the RX stream: injected mid-frame they would abort the in-flight run, so
// they are skipped (planscope's frame scanner is immune by construction).

#include "../src/plan_screen.h"

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace walk_replay {

using plan::PlanScreen;
using plan::SCR_BAND_MAX_Y;
using plan::SCR_TERM_ESP;
using plan::SCR_TITLE_BAR_PX;

// The settle threshold: planscope's settleQuiet (600 ms) -- above the
// capture's worst measured intra-repaint gap / press latency (<= 480 ms,
// planscope TestReplaySettleGaps), below any key pacing (a press only
// happens after a 600 ms quiet wait, so the next repaint starts later).
static constexpr int REPLAY_SETTLE_MS = 600;

// PlanScreen with the band state opened up for the selection invariant.
struct ReplayScreen : PlanScreen {
  // Lit bands below the title bar -- the menu-selection invariant says <= 1.
  int inverse_body_bands() const {
    int n = 0;
    for (int y = SCR_TITLE_BAR_PX; y <= SCR_BAND_MAX_Y; y++)
      if (bands_[1][y].set && bands_[1][y].inverse)  // [1] = own terminal
        n++;
    return n;
  }
};

struct Tok {
  uint8_t b;
  bool bit9;
};

// Strip ANSI CSI color sequences (ESC [ ... letter).
inline std::string strip_ansi(const char *s) {
  std::string out;
  for (const char *p = s; *p != '\0';) {
    if (p[0] == '\x1b' && p[1] == '[') {
      p += 2;
      while (*p != '\0' && !std::isalpha(static_cast<unsigned char>(*p)))
        p++;
      if (*p != '\0')
        p++;
    } else {
      out.push_back(*p++);
    }
  }
  return out;
}

// Parse one log line: timestamp (ms since midnight, -1 if the line has
// none) and the hex tokens with their bit9 marks. Bracketed segments
// (timestamps, log tags, burst counts) are removed before tokenizing, like
// planscope's parseLine. Returns false when the line carries no hex bytes.
inline bool parse_line(const std::string &line, int &ts_ms, std::vector<Tok> &toks) {
  ts_ms = -1;
  int h, m, s, ms;
  if (std::sscanf(line.c_str(), "[%2d:%2d:%2d.%3d]", &h, &m, &s, &ms) == 4)
    ts_ms = ((h * 60 + m) * 60 + s) * 1000 + ms;
  toks.clear();
  std::string stripped;
  bool in_bracket = false;
  for (char c : line) {
    if (c == '[')
      in_bracket = true;
    else if (c == ']')
      in_bracket = false;
    else if (!in_bracket)
      stripped.push_back(c);
    else
      continue;
    if (c == '[' || c == ']')
      stripped.push_back(' ');
  }
  auto hex = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    return -1;
  };
  size_t i = 0, n = stripped.size();
  while (i < n) {
    while (i < n && std::isspace(static_cast<unsigned char>(stripped[i])))
      i++;
    size_t j = i;
    while (j < n && !std::isspace(static_cast<unsigned char>(stripped[j])))
      j++;
    size_t len = j - i;
    if (len == 2 || (len == 3 && stripped[i + 2] == '\'')) {
      int hi = hex(stripped[i]), lo = hex(stripped[i + 1]);
      if (hi >= 0 && lo >= 0)
        toks.push_back({static_cast<uint8_t>(hi << 4 | lo), len == 3});
    }
    i = j;
  }
  return !toks.empty();
}

// Feed the capture at `path` into `scr`, calling on_settled(ts_ms, scr) on
// every settled snapshot (>= REPLAY_SETTLE_MS quiet after the last own-
// session display activity). Returns the settled count, or -1 when the file
// cannot be opened (caller decides whether that is a skip or a failure).
template<typename F>
inline int replay_walk(const char *path, ReplayScreen &scr, F &&on_settled) {
  std::FILE *f = std::fopen(path, "r");
  if (f == nullptr)
    return -1;
  char buf[4096];
  std::vector<Tok> toks;
  int last_ts = 0, last_change = -1, settled = 0;
  bool dirty = false;
  while (std::fgets(buf, sizeof buf, f) != nullptr) {
    std::string line = strip_ansi(buf);
    if (line.find("TX(9bit)") != std::string::npos)
      continue;
    int ts;
    if (!parse_line(line, ts, toks))
      continue;
    if (ts < 0)
      ts = last_ts;
    last_ts = ts;
    if (dirty && ts - last_change >= REPLAY_SETTLE_MS) {
      settled++;
      on_settled(last_change, scr);
      dirty = false;
    }
    // Activity = a visible change on either terminal (feed's return) OR any
    // completed own-session display frame (painted_ms advance) -- the latter
    // covers 0x0C single-cell updates, which flip the D-page ID digit when
    // paging.
    uint32_t p0 = scr.painted_ms(SCR_TERM_ESP);
    bool changed = false;
    for (const Tok &t : toks)
      changed |= scr.feed(t.b, t.bit9 ? 1 : 0, static_cast<uint32_t>(ts));
    if (changed || scr.painted_ms(SCR_TERM_ESP) != p0) {
      last_change = ts;
      dirty = true;
    }
  }
  std::fclose(f);
  if (dirty) {
    settled++;
    on_settled(last_change, scr);
  }
  return settled;
}

// Default archive location relative to the repo root (where CI compiles and
// runs the tests from). The reference capture is not part of this repo --
// pass a capture path as argv[1] to run against one; without it the replay
// tests skip (and still exit 0).
static constexpr const char *WALK_LOG = "test/walk.log";

// Test argv convention: [path] [--live]. --live relaxes the assertions to
// what any capture of a scrape-style route must satisfy (a fresh hardware
// capture visits a route subset, not the full manual menu walk of the
// reference archive).
struct ReplayArgs {
  const char *path = WALK_LOG;
  bool live = false;
};

inline ReplayArgs parse_args(int argc, char **argv) {
  ReplayArgs a;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--live")
      a.live = true;
    else
      a.path = argv[i];
  }
  return a;
}

}  // namespace walk_replay
