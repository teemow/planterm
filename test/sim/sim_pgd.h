#pragma once

// Behavioural model of the real pGD1 terminal at pLAN address 0x20 (wave C11),
// implementing the A6 state machine (`wave/A6/pgd-model.md`, rows T1-T20) on
// the simulated bus. PgdStub (sim_terminals.h) stays the minimal healthy
// stub for the B10 tests; PgdModel adds everything the pGD does when the
// controller does not hear it:
//
//   OFF -> BOOTING (17-20 s silent, T1/T2) -> LISTEN
//   JOINING   echo the controller's roll-call twice (MAP verbatim, own claim
//             bit added), link reply on the 3rd (T3-T5) or on a poll (T5b);
//             nothing follows an echo within ~14 ms -> own walk (T6)
//   SERVED / SESSIONED  answer every poll ~245 us after its stop bit (T7),
//             ack every valid display frame (T10), ident `01' 51 07 20 0A 17
//             65` (T11), never ack a bad check (sect. 4), no duplicate
//             suppression; keypad report + link reply in one slot
//   PARKED    sessioned but no longer polled (T14; derived, same TX rules)
//   REPLY_RETRY  unacked link reply resent 14 ms, then 15 ms later: 3 tries
//             (T8, R-LL-05); a new poll restarts the exchange
//   MASTER    after the 3rd unacked reply (T9) or an unconfirmed echo (T6):
//             own FF-walk `NN' 02 20 MAP 80 00 00 00`, NN = 01..1F, 15 ms
//             apart, MAP clears each silent address, restart at 01' 12 ms
//             after 1F' (~458 ms/round, T15/T16), forever; the walk stops
//             when the probed address continues (T17/T18) or any controller
//             frame is heard; a roll-call re-adopts it (T3).
//   member-forwarded poll `20' 01 SRC` (SRC != 01): link reply to 0x01 as
//             itself, no retry (T12); member-forwarded roll-call: ignored (T13).
//
// Gaps (A6 sect. 8, not modelled): pGD poll forwarding and type-0x1F frames
// (T19/T20), NO LINK timer, idle-timer walks on a silent bus, release /
// combination key codes, 4th consecutive roll-call (we wrap to a new echo).

#include "sim_bus.h"

#include <array>
#include <string>

namespace plan {
namespace sim {

class PgdModel : public Station {
 public:
  enum State { OFF, BOOTING, LISTEN, JOINING, SERVED, SESSIONED, PARKED, REPLY_RETRY, MASTER };
  struct Params {
    int64_t boot_us = 18500000;  // T2: 17.4-19.7 s
    // T7 / A7: idle after the poll's stop bit (start-to-start 421 us with the
    // 176 us char of the ISR ring = 245 us idle), N(245, 6) clipped.
    double turn_mean_us = 245, turn_sd_us = 6;
    int64_t turn_min_us = 229, turn_max_us = 269;
    double outlier_p = 0.001;          // 0.5-1.4 ms "busy redrawing"
    int64_t resend1_us = 14000;        // R-LL-05: 1st resend 14 ms after the reply
    int64_t resend2_us = 15000;        // R-LL-05: 2nd resend 15 ms later
    int64_t walk_after_3rd_us = 16000; // A2 T2: self-walk 16 ms after the 3rd
    int64_t walk_after_echo_us = 14000;// A2 T3: self-walk 14 ms after the echo
    int64_t t_slot_us = 15000;         // T15: probe spacing
    int64_t round_restart_us = 12000;  // T16: 1F' -> 01'
    int64_t park_us = 500000;          // T14: no poll for this long = PARKED
    uint64_t seed = 0x20;
  };
  Params prm;
  std::array<std::string, 8> screen;
  uint32_t polls = 0, fwd_polls = 0, replies = 0, resends = 0, acks_rx = 0, echoes = 0,
           sacks = 0, bad_ck = 0, inits = 0, idents = 0, keys_sent = 0, master_walks = 0,
           master_probes = 0, handoffs = 0, joins = 0, fwd_rollcalls_ignored = 0;

  // booted = true: start in LISTEN (skip the 18 s boot for steady-state tests).
  explicit PgdModel(bool booted = true, uint8_t a = 0x20) : Station(a), rng_(0x20) {
    st_ = booted ? LISTEN : OFF;
    for (auto &r : screen) r.assign(22, ' ');
  }
  void seed(uint64_t s) { rng_ = Rng(s); }

  // T1: plug-in / power-up, optionally with the stray `00` byte.
  void power_on(int64_t t, bool glitch = true) {
    st_ = BOOTING;
    sessioned_ = false;
    if (glitch) bus->inject_glitch(t, {0x00, false});
    arm_(t + prm.boot_us, T_BOOT);
  }
  void power_off() {
    st_ = OFF;
    ++gen_;
  }
  // Keypad (A6 sect. 5): report NN=01 at the first poll, while held NN=0A
  // at +1.0 s then +2 per 200 ms (max C8), only when NN changes; release
  // sends nothing.
  void press(uint8_t key, int64_t hold_us = 0) {
    key_ = key;
    hold_us_ = hold_us;
    key_t0_ = -1;
    key_nn_ = 0;
  }

  State state(int64_t now) const {
    if ((st_ == SERVED || st_ == SESSIONED) && now - last_poll_t_ > prm.park_us) return PARKED;
    return st_;
  }
  static const char *name(State s) {
    static const char *n[] = {"OFF", "BOOTING", "LISTEN", "JOINING", "SERVED",
                              "SESSIONED", "PARKED", "REPLY_RETRY", "MASTER"};
    return n[s];
  }

  void on_rx(const WireByte &b, int64_t t) override {
    if (st_ == OFF || st_ == BOOTING) return;  // T2: nothing while booting
    // The bare `01'` ack of our link reply (sect. 4: consumed, never answered).
    if (await_ack_ && b.addr && !b.err && b.v == 0x01) {
      await_ack_ = false;
      acks_rx++;
      ++gen_;
      st_ = sessioned_ ? SESSIONED : SERVED;
    }
    Frame fr;
    int r = asm_.push(b, fr);
    if (r != 1) return;
    if (!frame_ok(fr)) {
      if (fr[0].v == addr) bad_ck++;  // sect. 4: a bad check is never acked
      return;
    }
    frame_(fr, t);
  }

  void on_timer(int id, int64_t t) override {
    if (id != gen_) return;
    switch (tk_) {
      case T_BOOT: st_ = LISTEN; break;
      case T_RESEND:
        if (attempt_ < 3) {  // T8 / R-LL-05: byte-identical resend
          attempt_++;
          resends++;
          st_ = REPLY_RETRY;
          tx_(mk(0x01, {0x01, addr}), t);
          arm_(t + (attempt_ == 2 ? prm.resend2_us : prm.walk_after_3rd_us), T_RESEND);
        } else {  // T9: 3rd link reply unanswered -> own walk
          await_ack_ = false;
          master_(t);
        }
        break;
      case T_ECHO: master_(t); break;  // T6: nothing followed the echo
      case T_PROBE:                    // T15: probed address silent
        mmap_ &= ~bit(nn_);
        if (++nn_ >= addr) {  // T16: wrap after 1F' with an all-ones MAP
          nn_ = 1;
          mmap_ = 0xFFFFFFFFu;
          master_walks++;
        }
        probe_(t);
        break;
    }
  }

 private:
  enum Tk { T_BOOT, T_RESEND, T_ECHO, T_PROBE };

  static uint8_t src_of(const Frame &f) {
    uint8_t ty = f[1].v;
    return (ty <= 0x03) ? f[2].v : (f.size() > 3 ? f[3].v : 0);
  }
  int64_t turn_() {
    if (rng_.chance(prm.outlier_p)) return 500 + static_cast<int64_t>(rng_.uni() * 900);
    double v = prm.turn_mean_us + prm.turn_sd_us * rng_.gauss();
    return std::min<int64_t>(prm.turn_max_us,
                             std::max<int64_t>(prm.turn_min_us, static_cast<int64_t>(v)));
  }
  void arm_(int64_t t, Tk k) {
    tk_ = k;
    bus->at(this, t, ++gen_);
  }
  void tx_(const Frame &f, int64_t t) { bus->transmit(this, f, t); }

  void frame_(const Frame &fr, int64_t t) {
    uint8_t to = fr[0].v, ty = fr[1].v, src = src_of(fr);
    if (st_ == MASTER && (src == nn_ || src == 0x01)) {  // T17/T18: token taken
      handoffs++;
      st_ = LISTEN;
      ++gen_;
    }
    if (st_ == JOINING && src == 0x01) ++gen_;  // the controller continued: no T6
    if (to != addr) return;
    int64_t t0 = t + turn_();
    if (ty == 0x02) {
      if (src != 0x01) {  // T13: member-forwarded roll-call
        fwd_rollcalls_ignored++;
        return;
      }
      rc_n_ = rc_n_ % 3 + 1;
      if (rc_n_ == 3) {  // T5: the 3rd consecutive roll-call gets the link reply
        joins++;
        return reply_(t0, true, Frame{});
      }
      echoes++;  // T3/T4: MAP verbatim, CLAIMS | own bit
      st_ = JOINING;
      tx_(rollcall(0x01, addr, get_mask(fr, 3), get_mask(fr, 7) | bit(addr)), t0);
      arm_(t0 + prm.walk_after_echo_us, T_ECHO);
      return;
    }
    if (ty == 0x01) {  // T7 poll / T12 member-forwarded poll
      if (rc_n_ == 2) joins++;  // T5b: poll after two echoes
      rc_n_ = 0;
      last_poll_t_ = t;
      bool direct = src == 0x01;
      (direct ? polls : fwd_polls)++;
      return reply_(t0, direct, key_report_(t));
    }
    rc_n_ = 0;
    if (ty == 0x50) {  // T11, no sack, answered every time
      idents++;
      return tx_(mk(0x01, {0x51, 0x07, addr, 0x0A, 0x17}), t0);
    }
    if ((ty >= 0x0A && ty <= 0x0F) || (ty >= 0x64 && ty <= 0x66)) {  // T10
      sacks++;
      sessioned_ = true;
      if (st_ != REPLY_RETRY && st_ != MASTER) st_ = SESSIONED;
      if (ty == 0x66) inits++;
      if (ty == 0x0B && fr[4].v < 8)
        for (size_t c = 0; c < 22 && 5 + c < fr.size() - 1; c++)
          screen[fr[4].v][c] = static_cast<char>(fr[5 + c].v);
      if (ty == 0x0C && fr[4].v < 8 && fr[5].v < 22)
        screen[fr[4].v][fr[5].v] = static_cast<char>(fr[6].v);
      tx_(mk(0x01, {0x03, addr}), t0);
    }
  }

  Frame key_report_(int64_t t) {
    if (!key_) return Frame{};
    uint8_t nn;
    if (key_t0_ < 0) {
      key_t0_ = t;
      nn = 0x01;
    } else if (t - key_t0_ >= hold_us_) {  // released: no report
      key_ = 0;
      return Frame{};
    } else if (t - key_t0_ < 1000000) {
      nn = 0x01;
    } else {
      int64_t steps = (t - key_t0_ - 1000000) / 200000;
      nn = static_cast<uint8_t>(std::min<int64_t>(0xC8, 0x0A + 2 * steps));
    }
    if (nn == key_nn_) return Frame{};
    key_nn_ = nn;
    keys_sent++;
    Frame k = mk(0x01, {0x1E, 0x07, addr, key_, nn});
    if (hold_us_ == 0) key_ = 0;  // a tap reports once
    return k;
  }
  // One response slot: [keypad report] + link reply; `retry` arms T8.
  void reply_(int64_t t0, bool retry, Frame f) {
    Frame r = mk(0x01, {0x01, addr});
    f.insert(f.end(), r.begin(), r.end());
    replies++;
    tx_(f, t0);
    st_ = sessioned_ ? SESSIONED : SERVED;
    if (!retry) return;
    await_ack_ = true;
    attempt_ = 1;
    arm_(t0 + prm.resend1_us, T_RESEND);
  }
  void master_(int64_t t) {  // T6/T9 -> MASTER at 01' with FF FF FF FF
    st_ = MASTER;
    master_walks++;
    nn_ = 1;
    mmap_ = 0xFFFFFFFFu;
    probe_(t);
  }
  void probe_(int64_t t) {
    master_probes++;
    tx_(rollcall(nn_, addr, mmap_, bit(addr)), t);
    arm_(t + (nn_ == addr - 1 ? prm.round_restart_us : prm.t_slot_us), T_PROBE);
  }

  Rng rng_;
  FrameAsm asm_;
  State st_;
  Tk tk_ = T_BOOT;
  int gen_ = 0, rc_n_ = 0, attempt_ = 0;
  bool await_ack_ = false, sessioned_ = false;
  uint8_t nn_ = 1, key_ = 0, key_nn_ = 0;
  uint32_t mmap_ = 0;
  int64_t last_poll_t_ = 0, hold_us_ = 0, key_t0_ = -1;
};

}  // namespace sim
}  // namespace plan
