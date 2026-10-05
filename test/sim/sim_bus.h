#pragma once

// Deterministic discrete-event pLAN bus for host tests (wave row B10).
//
// Time is in microseconds. Every byte occupies 12 bit-slots at 62500 baud
// (start, 8 data, 9th/address bit, 2 stop) = 192 us on the wire (R-LL-01) and
// is delivered to every other attached station at the END of its character
// time. A station never hears itself (the bridge's DE+RE are tied, so its
// receiver is off while it drives). Two stations driving at once is a
// collision: the overlapping bytes reach every receiver corrupted and the
// bus counts it (half-duplex arbitration is the stations' job, the bus only
// referees it).
//
// FAULT HOOK: `FaultHook` is the single place later rows inject faults
// (frames dropped or corrupted per direction, idle-level glitches). This row
// implements the hook points, not the faults; with no hook the bus is ideal.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <queue>
#include <string>
#include <vector>

namespace plan {
namespace sim {

static constexpr int64_t CHAR_US = 192;  // R-LL-01: 12 bit-slots @ 62500 baud

struct WireByte {
  uint8_t v;
  bool addr;         // 9th bit (address mark)
  bool err = false;  // framing error / collision seen by the receiver
};
using Frame = std::vector<WireByte>;

// --- frame helpers ----------------------------------------------------------
inline uint8_t sum8(const Frame &f, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; i++) s += f[i].v;
  return s;
}
inline uint16_t crc16(const Frame &f, size_t n) {  // CRC-16/Modbus
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= f[i].v;
    for (int k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}
inline bool crc_type(uint8_t t) { return t == 0x64 || t == 0x65 || t == 0x66; }

// Build `to' type b...` and append the type's check (R-SE-01, R-DI-02).
inline Frame mk(uint8_t to, const std::vector<uint8_t> &body) {
  Frame f{{to, true}};
  for (uint8_t b : body) f.push_back({b, false});
  if (f.size() >= 2 && crc_type(f[1].v)) {
    uint16_t c = crc16(f, f.size());
    f.push_back({static_cast<uint8_t>(c & 0xFF), false});
    f.push_back({static_cast<uint8_t>(c >> 8), false});
  } else {
    f.push_back({static_cast<uint8_t>(0xFF - sum8(f, f.size())), false});
  }
  return f;
}

// Expected length of a frame from its first bytes (0 = not known yet).
// 01 poll/link reply, 03 session ack: 4; 02 roll-call: 12 (R-RC-01);
// 1E keypad report: 7 (R-KP-01); every other type carries LEN at byte 2
// (R-SE-01).
inline size_t want_len(const Frame &f) {
  if (f.size() < 2) return 0;
  switch (f[1].v) {
    case 0x01: case 0x03: return 4;
    case 0x02: return 12;
    case 0x1E: return 7;
    default: return f.size() < 3 ? 0 : f[2].v;
  }
}
// R-DI-01: address mark on byte 0 only, length, check; else discard whole.
inline bool frame_ok(const Frame &f) {
  if (f.empty() || !f[0].addr || f.size() != want_len(f)) return false;
  for (size_t i = 0; i < f.size(); i++)
    if (f[i].err || (i > 0 && f[i].addr)) return false;
  if (crc_type(f[1].v)) return crc16(f, f.size()) == 0;  // residue 0 (R-DI-02)
  return sum8(f, f.size()) == 0xFF;
}

// Roll-call masks: address a is bit (a-1) of a uint32, so byte 0's MSB is
// address 32 and byte 3's LSB is address 1 (R-RC-01).
inline uint32_t bit(uint8_t a) { return 1u << (a - 1); }
inline void put_mask(std::vector<uint8_t> &b, uint32_t m) {
  for (int i = 3; i >= 0; i--) b.push_back(static_cast<uint8_t>(m >> (8 * i)));
}
inline uint32_t get_mask(const Frame &f, size_t at) {
  uint32_t m = 0;
  for (size_t i = 0; i < 4; i++) m = (m << 8) | f[at + i].v;
  return m;
}
inline Frame rollcall(uint8_t to, uint8_t from, uint32_t map, uint32_t claims) {
  std::vector<uint8_t> b{0x02, from};
  put_mask(b, map);
  put_mask(b, claims);
  return mk(to, b);
}

inline std::string hex(const Frame &f) {
  std::string s;
  char buf[8];
  for (const auto &b : f) {
    snprintf(buf, sizeof buf, "%s%02X%s", s.empty() ? "" : " ", b.v, b.addr ? "'" : "");
    s += buf;
  }
  return s;
}

// --- stations and the bus ---------------------------------------------------
class Bus;

class Station {
 public:
  explicit Station(uint8_t a) : addr(a) {}
  virtual ~Station() = default;
  virtual void on_rx(const WireByte &, int64_t /*t_end_us*/) {}
  virtual void on_timer(int /*id*/, int64_t /*now_us*/) {}
  const uint8_t addr;  // 0 = not a pLAN address (noise source)
  Bus *bus = nullptr;
};

// The fault-injection seam (later rows). Defaults are the ideal bus.
struct FaultHook {
  virtual ~FaultHook() = default;
  // Once per transmission, before it reaches the wire: may corrupt bytes
  // (the logged frame shows the mutation, i.e. what is on the wire).
  virtual void on_tx_frame(uint8_t /*from*/, Frame &, int64_t /*t_us*/) {}
  // Once per byte per receiver: return false to drop the byte at that
  // receiver only, or mutate it. Directional loss (terminal -> controller
  // only, R-LL-21) lives here.
  virtual bool on_rx_byte(uint8_t /*from*/, uint8_t /*to*/, WireByte &, int64_t /*t_us*/) {
    return true;
  }
  // When the bus falls idle: may call Bus::inject_glitch() to model the lost
  // idle level (A16: the undriven bus drifts toward SPACE within ~1 ms).
  virtual void on_idle(Bus &, int64_t /*t_us*/) {}
};

struct LogFrame {
  int64_t t;      // start of the first byte on the wire
  uint8_t from;   // sender station address (0 = glitch)
  Frame f;
  bool collided = false;
  int64_t end() const { return t + CHAR_US * static_cast<int64_t>(f.size()); }
};

class Bus {
 public:
  int64_t now = 0;
  FaultHook *hook = nullptr;
  std::vector<LogFrame> log;  // ideal probe: everything put on the wire
  uint32_t collisions = 0;

  void attach(Station *s) {
    s->bus = this;
    st_.push_back(s);
  }
  // Start driving `f` at t0 (>= now). Bytes go back to back.
  void transmit(Station *s, Frame f, int64_t t0) {
    if (t0 < now) t0 = now;
    if (hook) hook->on_tx_frame(s->addr, f, t0);
    int64_t t1 = t0 + CHAR_US * static_cast<int64_t>(f.size());
    LogFrame lf{t0, s->addr, f};
    for (auto &a : active_)
      if (a.from != s && t0 < a.t1 && a.t0 < t1) {
        collisions++;
        coll_.push_back({std::max(t0, a.t0), std::min(t1, a.t1)});
        lf.collided = true;
      }
    active_.push_back({s, t0, t1});
    log.push_back(lf);
    for (size_t i = 0; i < f.size(); i++)
      push_({t0 + CHAR_US * static_cast<int64_t>(i + 1), seq_++, s, -1, f[i]});
  }
  void at(Station *s, int64_t t, int id) { push_({t, seq_++, s, id, {0, false}}); }
  // A stray byte on an otherwise idle bus (fault rows): delivered to everyone.
  void inject_glitch(int64_t t, WireByte b) { transmit(&noise_, Frame{b}, t); }

  void run_until(int64_t t_end) {
    while (!q_.empty() && q_.top().t <= t_end) {
      Ev e = q_.top();
      q_.pop();
      now = e.t;
      if (e.id >= 0) {
        e.s->on_timer(e.id, now);
        continue;
      }
      WireByte b = e.b;
      for (auto &c : coll_)
        if (now - CHAR_US < c.second && c.first < now) b.err = true, b.v ^= 0xA5;
      for (Station *r : st_) {
        if (r == e.s) continue;  // DE+RE tied: never hears itself
        WireByte rb = b;
        if (hook && !hook->on_rx_byte(e.s->addr, r->addr, rb, now)) continue;
        r->on_rx(rb, now);
      }
      bool idle = true;
      for (auto &a : active_)
        if (a.t1 > now) idle = false;
      if (idle) {
        active_.clear();
        if (hook) hook->on_idle(*this, now);
      }
    }
    now = t_end;
  }

  // --- log queries for tests -------------------------------------------------
  std::vector<LogFrame> from(uint8_t a, int64_t t0 = 0, int64_t t1 = INT64_MAX) const {
    std::vector<LogFrame> r;
    for (const auto &l : log)
      if (l.from == a && l.t >= t0 && l.t < t1) r.push_back(l);
    return r;
  }
  void dump(int64_t t0, int64_t t1) const {
    for (const auto &l : log)
      if (l.t >= t0 && l.t < t1)
        printf("%10.3f ms  %02X: %s\n", l.t / 1000.0, l.from, hex(l.f).c_str());
  }

 private:
  struct Ev {
    int64_t t;
    uint64_t seq;
    Station *s;
    int id;  // -1 = byte end
    WireByte b;
    bool operator>(const Ev &o) const { return t != o.t ? t > o.t : seq > o.seq; }
  };
  struct Act {
    Station *from;
    int64_t t0, t1;
  };
  void push_(Ev e) { q_.push(e); }
  std::priority_queue<Ev, std::vector<Ev>, std::greater<Ev>> q_;
  std::vector<Station *> st_;
  std::vector<Act> active_;
  std::vector<std::pair<int64_t, int64_t>> coll_;
  uint64_t seq_ = 0;
  Station noise_{0};
};

// Splits a station's receive stream into frames (address mark = new frame).
// A frame cut short by the next address byte is "garbage" (heard, not
// valid); a lone `01'` is the controller's bare ack and complete by itself.
struct FrameAsm {
  Frame cur;
  // Returns 1 when `out` holds a complete frame, -1 when garbage was
  // discarded, 0 otherwise.
  int push(const WireByte &b, Frame &out) {
    int r = 0;
    if (b.addr && !cur.empty()) {
      if (!(cur.size() == 1 && cur[0].v == 0x01)) r = -1;
      cur.clear();
    }
    if (!b.addr && cur.empty()) return -1;  // stray data byte
    cur.push_back(b);
    size_t w = want_len(cur);
    if (w && cur.size() >= w) {
      out = cur;
      cur.clear();
      return 1;
    }
    return r;
  }
};

}  // namespace sim
}  // namespace plan
