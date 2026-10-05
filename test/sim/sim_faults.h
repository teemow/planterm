#pragma once

// Seeded physical-fault model for the pLAN simulator (wave C11), built on the
// FaultHook seam of sim_bus.h. Parameters model the measured regimes:
//
//   * p_to_ctrl: terminal -> controller frames rejected at the controller's
//     receiver only (A2 R-LL-21 / sect. G, A16 2.1: 10-02 ~1 % replies and
//     4-5 % acks, tonight 27-49 %; the controller's own frames still arrive).
//     Default mode = heard but rejected (mis-framed address byte, the A16
//     mechanism): the controller sees garbage, so it resends / re-polls like
//     the real one. `vanish` = the frame never reaches it (silent).
//   * p_from_ctrl: controller -> terminal rejection (A16: 0.1-3.8 % tonight).
//   * drift: the lost idle level (A16 2.2). A frame that starts after an
//     undriven gap of `idle` us loses its address byte (or, break_byte, gets a
//     glued `00` break byte before it) at receiver `rx` with probability
//     p_max * clamp((idle - idle_min) / (idle_full - idle_min), 0, 1).
//     Bursts (keypad report + link reply) only drift on their first frame.
//   * p_byte: random single-bit corruption per byte per receiver.
//   * ctrl_silent_at: R9 CTRL_SILENT_PGD_MASTER. From the first bare ack the
//     controller would send at/after this time, nothing it sends reaches the
//     wire (it stops mid-exchange, so the pGD's last reply goes unanswered).
//   * reset_at / reset_us: bridge reset with DE floating; random bytes back
//     to back (10 % with the address mark) for reset_us, colliding with
//     whatever else is on the bus (A7: the ROM banner on GPIO21 = DI).
//   * p_walk_stall: A2 T7 / R-LL-12, the controller stalls 2.00 s on its own
//     first walk frame after a session-ack fault (29/29 tonight passive).
//
// Every random draw comes from one splitmix64 stream seeded by `seed`, so a
// scenario is reproducible bit for bit.

#include "sim_bus.h"

#include <map>
#include <vector>

namespace plan {
namespace sim {

struct FaultSpec {
  uint64_t seed = 1;
  double p_to_ctrl = 0;
  double p_to_ctrl_reply = -1;  // type 01 link replies + 02 echoes (< 0: p_to_ctrl)
  double p_to_ctrl_sack = -1;   // type 03 session acks (< 0: p_to_ctrl)
  bool vanish = false;
  double p_from_ctrl = 0;
  struct Drift {
    uint8_t rx;
    double p_max;
    int64_t idle_min_us, idle_full_us;
    bool break_byte;
  };
  std::vector<Drift> drift;
  double p_byte = 0;
  int64_t ctrl_silent_at = -1;
  int64_t reset_at = -1, reset_us = 0;
  double p_walk_stall = 0;
};

class PhysFault : public FaultHook {
 public:
  FaultSpec spec;
  uint32_t rejected_to_ctrl = 0, rejected_from_ctrl = 0, drift_hits = 0, break_bytes = 0,
           bytes_corrupted = 0, suppressed = 0, noise_bytes = 0, stalls = 0;

  explicit PhysFault(const FaultSpec &s) : spec(s), rng_(s.seed) {}

  // Install on the bus and pre-schedule the reset noise (if any).
  void attach(Bus &b) {
    b.hook = this;
    if (spec.reset_at < 0) return;
    for (int64_t t = spec.reset_at; t < spec.reset_at + spec.reset_us; t += CHAR_US) {
      noise_bytes++;
      b.inject_glitch(t, {static_cast<uint8_t>(rng_.next()), rng_.chance(0.1)});
    }
  }

  bool tx_allowed(uint8_t from, const Frame &f, int64_t t) override {
    if (from != 0x01 || spec.ctrl_silent_at < 0 || t < spec.ctrl_silent_at) return true;
    if (f.size() == 1) silent_ = true;  // the first ack after the cut-off
    if (silent_) suppressed++;
    return !silent_;
  }
  void on_tx_frame(uint8_t from, Frame &f, int64_t t) override {
    tx_.push_back({from, t, f});
    if (tx_.size() > 64) tx_.erase(tx_.begin());
  }
  bool rx_prefix(uint8_t from, uint8_t to, const WireByte &b, int64_t t,
                 WireByte &extra) override {
    if (t != cur_t_) prev_end_ = cur_t_, cur_t_ = t;  // end of the previous byte on the bus
    if (!b.addr) return false;
    decide_(from, to, b, t);
    if (!dec_[key_(from, to)].brk) return false;
    break_bytes++;
    extra = {0x00, false, true};
    return true;
  }
  bool on_rx_byte(uint8_t from, uint8_t to, WireByte &b, int64_t) override {
    const Dec &d = dec_[key_(from, to)];
    if (b.addr && (d.reject || d.drop_addr)) {
      if (spec.vanish && d.reject) return false;
      b.err = true;  // mis-framed address byte: heard, rejected
      if (d.drop_addr) b.addr = false, b.v = 0x00;
    } else if (!b.addr && d.reject && spec.vanish) {
      return false;
    }
    if (rng_.chance(spec.p_byte)) {
      bytes_corrupted++;
      b.v ^= static_cast<uint8_t>(1u << (rng_.next() % 8));
    }
    return true;
  }
  bool ctrl_walk_stall(int cause, int64_t) override {
    // SimController::FC_SACK == 4 (A2 T7 follows T1).
    if (cause != 4 || !rng_.chance(spec.p_walk_stall)) return false;
    stalls++;
    return true;
  }

 private:
  struct Tx {
    uint8_t from;
    int64_t t0;
    Frame f;
  };
  struct Dec {
    bool reject = false, drop_addr = false, brk = false;
  };
  static uint16_t key_(uint8_t from, uint8_t to) { return static_cast<uint16_t>(from << 8 | to); }

  // One decision per frame per receiver, taken on its address byte.
  void decide_(uint8_t from, uint8_t to, const WireByte &, int64_t t) {
    int64_t start = t - CHAR_US;
    Dec d;
    uint8_t type = 0;
    bool first = true;
    for (auto it = tx_.rbegin(); it != tx_.rend(); ++it)
      if (it->from == from && it->t0 <= start &&
          start < it->t0 + CHAR_US * static_cast<int64_t>(it->f.size())) {
        size_t k = static_cast<size_t>((start - it->t0) / CHAR_US);
        if (k + 1 < it->f.size()) type = it->f[k + 1].v;
        first = k == 0;
        break;
      }
    if (to == 0x01 && from != 0x01 && from != 0) {  // R-LL-21: terminal -> controller
      double p = spec.p_to_ctrl;
      if ((type == 0x01 || type == 0x02) && spec.p_to_ctrl_reply >= 0) p = spec.p_to_ctrl_reply;
      if (type == 0x03 && spec.p_to_ctrl_sack >= 0) p = spec.p_to_ctrl_sack;
      if (rng_.chance(p)) d.reject = true, rejected_to_ctrl++;
    } else if (from == 0x01 && rng_.chance(spec.p_from_ctrl)) {
      d.reject = true, rejected_from_ctrl++;
    }
    int64_t idle = start - prev_end_;
    for (const auto &dr : spec.drift) {
      if (dr.rx != to || !first || idle < dr.idle_min_us) continue;
      double span = static_cast<double>(std::max<int64_t>(1, dr.idle_full_us - dr.idle_min_us));
      double p = dr.p_max * std::min(1.0, static_cast<double>(idle - dr.idle_min_us) / span);
      if (!rng_.chance(p)) continue;
      drift_hits++;
      if (dr.break_byte) d.brk = true;
      else d.drop_addr = true;
    }
    dec_[key_(from, to)] = d;
  }

  Rng rng_;
  std::vector<Tx> tx_;
  std::map<uint16_t, Dec> dec_;
  int64_t cur_t_ = 0, prev_end_ = 0;
  bool silent_ = false;
};

}  // namespace sim
}  // namespace plan
