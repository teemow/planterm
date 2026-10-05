#pragma once

// Replay-conformance input: a normalized pLAN frame stream as printed by
// ekobeescope (`ekobeescope frames --text --live <capture>`, the golden
// format of ekobeescope's corpus/fixtures/*.frames.txt), plus a header of
// `# key: value` lines that records the fixture's provenance and the task
// state the bridge firmware had while the capture ran.
//
//   L20 23:10:30.491 rx #1 01>02 bit9 ok walk | 02 02 01 FF FF FF FF 00 00 00 00 FE
//   L49 23:10:30.779 TX:rollcall @112904363835us 1F>01 bit9 ok walk | 01 02 1F 40 ...
//
// `rx` frames are what the bridge's receiver heard (every byte the RX ISR fed
// to PlanTerminal::on_byte). `TX:<kind>` frames are the bridge's own
// transmissions from its tx-sent diagnostics: DE and RE are tied, so they
// never appear in the receive stream, and they carry the device clock.
//
// Timing model (host stamps are batched and are NEXT-frame stamps: ekobeescope
// emits frame k when the address byte of frame k+1 arrives, or after a 250 ms
// read deadline on a quiet bus -- wave row A2, link-layer.md section 0):
//   start(k) = stamp(k-1)                      if stamp(k) - stamp(k-1) < 250 ms
//            = stamp(k) - dur(k) [- 250 ms if frame k itself was idle-flushed]
//   start(k) >= end(k-1) + 200 us              (monotonic, never overlapping)
// and byte j of a frame completes at start + (j+1) * 176 us (11-bit chars at
// 62500 Bd). Good to a few ms on a busy bus and to ~250 ms around silences;
// every decision PlanTerminal takes on time (15 s pGD liveness, 2 s forward
// probe pacing, 1 s forward back-off) is seconds-scale.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace replay {

static constexpr int64_t CHAR_US = 176;          // 11-bit character at 62500 Bd
static constexpr int64_t IDLE_FLUSH_US = 250000; // ekobeescope's read deadline
static constexpr int64_t MIN_GAP_US = 200;       // floor between two frames

struct Frame {
  int line = 0;            // source line in the raw capture (the L<n> field)
  int64_t t_ms = -1;       // host receive time, ms since midnight (monotonic)
  bool tx = false;         // the bridge's own transmission
  std::string kind;        // tx kind: rollcall link session_ack ident key fwd_poll 9bit
  int64_t dev_us = 0;      // device clock of a tx frame, 0 = none
  int from = -1, to = -1;  // sender / destination per the frame grammar, -1 = unknown
  bool bit9 = false;       // the first byte carried the 9th (address) bit
  std::vector<uint8_t> bytes;
  int64_t start_us = 0;    // rx frames: wire start per the timing model
};

struct Fixture {
  std::string name;
  std::map<std::string, std::string> meta;  // header `# key: value`
  std::vector<Frame> frames;                // file order, rx and tx interleaved

  std::string get(const std::string &k, const std::string &def = "") const {
    auto it = meta.find(k);
    return it == meta.end() ? def : it->second;
  }
  // "enroll=1 fwd=1 tx_mode=2" style lists inside one header value.
  int get_kv(const std::string &key, const std::string &field, int def) const {
    std::istringstream is(get(key));
    std::string tok;
    while (is >> tok) {
      auto eq = tok.find('=');
      if (eq != std::string::npos && tok.substr(0, eq) == field)
        return std::atoi(tok.c_str() + eq + 1);
    }
    return def;
  }
};

inline int hex_addr(const std::string &s) {
  if (s == "??")
    return -1;
  return static_cast<int>(std::strtol(s.c_str(), nullptr, 16));
}

inline int64_t parse_hms_ms(const std::string &s) {
  int h = 0, m = 0, sec = 0, ms = 0;
  if (std::sscanf(s.c_str(), "%d:%d:%d.%d", &h, &m, &sec, &ms) != 4)
    return -1;
  return ((h * 60LL + m) * 60 + sec) * 1000 + ms;
}

// One `ekobeescope frames --text` line. Returns false for anything else.
inline bool parse_frame_line(const std::string &line, Frame &f) {
  auto bar = line.find(" | ");
  if (line.empty() || line[0] != 'L' || bar == std::string::npos)
    return false;
  std::istringstream head(line.substr(0, bar));
  std::string tok;
  std::vector<std::string> t;
  while (head >> tok)
    t.push_back(tok);
  if (t.size() < 7)
    return false;
  f = Frame();
  f.line = std::atoi(t[0].c_str() + 1);
  f.t_ms = parse_hms_ms(t[1]);
  size_t i = 2;
  if (t[i].rfind("TX:", 0) == 0) {
    f.tx = true;
    f.kind = t[i].substr(3);
  } else if (t[i] != "rx") {
    return false;  // snapshot replay or unknown: not live traffic
  }
  i++;
  if (i < t.size() && t[i][0] == '#')
    i++;
  if (i < t.size() && t[i][0] == '@')
    f.dev_us = std::atoll(t[i++].c_str() + 1);
  if (i + 1 >= t.size())
    return false;
  auto gt = t[i].find('>');
  if (gt == std::string::npos)
    return false;
  f.from = hex_addr(t[i].substr(0, gt));
  f.to = hex_addr(t[i].substr(gt + 1));
  f.bit9 = t[i + 1] == "bit9";
  std::istringstream body(line.substr(bar + 3));
  while (body >> tok)
    f.bytes.push_back(static_cast<uint8_t>(std::strtol(tok.c_str(), nullptr, 16)));
  return !f.bytes.empty();
}

inline bool load_fixture(const std::string &path, const std::string &name, Fixture &fx) {
  std::ifstream in(path);
  if (!in)
    return false;
  fx = Fixture();
  fx.name = name;
  std::string line;
  int64_t last_ms = -1;
  while (std::getline(in, line)) {
    if (!line.empty() && line[0] == '#') {
      auto colon = line.find(": ");
      if (colon != std::string::npos && colon > 2)
        fx.meta[line.substr(2, colon - 2)] = line.substr(colon + 2);
      continue;
    }
    Frame f;
    if (!parse_frame_line(line, f))
      continue;
    // The text form prints time modulo one day; keep it monotonic.
    if (f.t_ms < 0)
      f.t_ms = last_ms;
    else if (last_ms >= 0 && f.t_ms + 12 * 3600 * 1000LL < last_ms)
      f.t_ms += 24 * 3600 * 1000LL;
    last_ms = f.t_ms;
    fx.frames.push_back(f);
  }
  return true;
}

// The timing model above, applied in place to the receive stream.
inline void assign_times(std::vector<Frame> &rx) {
  const size_t n = rx.size();
  int64_t prev_end = 0;
  for (size_t k = 0; k < n; k++) {
    const int64_t stamp = rx[k].t_ms * 1000;
    const int64_t dur = static_cast<int64_t>(rx[k].bytes.size()) * CHAR_US;
    int64_t s;
    if (k > 0 && stamp - rx[k - 1].t_ms * 1000 < IDLE_FLUSH_US) {
      s = rx[k - 1].t_ms * 1000;
    } else {
      bool idle_after = k + 1 == n || rx[k + 1].t_ms * 1000 - stamp >= IDLE_FLUSH_US;
      s = stamp - dur - (idle_after ? IDLE_FLUSH_US : 0);
    }
    if (k > 0 && s < prev_end + MIN_GAP_US)
      s = prev_end + MIN_GAP_US;
    rx[k].start_us = s;
    prev_end = s + dur;
  }
}

// Completion time of byte j of rx frame k.
inline int64_t byte_time(const Frame &f, size_t j) {
  return f.start_us + static_cast<int64_t>(j + 1) * CHAR_US;
}

inline std::string hex(const std::vector<uint8_t> &b) {
  std::string s;
  char buf[4];
  for (size_t i = 0; i < b.size(); i++) {
    std::snprintf(buf, sizeof buf, i ? " %02X" : "%02X", b[i]);
    s += buf;
  }
  return s;
}

inline std::string fmt_ms(int64_t ms) {
  if (ms < 0)
    return "--:--:--.---";
  ms %= 24 * 3600 * 1000LL;
  char buf[16];
  std::snprintf(buf, sizeof buf, "%02d:%02d:%02d.%03d", static_cast<int>(ms / 3600000),
                static_cast<int>(ms / 60000 % 60), static_cast<int>(ms / 1000 % 60),
                static_cast<int>(ms % 1000));
  return buf;
}

}  // namespace replay
