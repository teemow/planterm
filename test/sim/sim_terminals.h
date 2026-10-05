#pragma once

// Terminals for the pLAN simulator: an adapter that puts planterm's real
// PlanTerminal (the code the UART ISR runs) on the simulated bus, and a
// minimal scripted pGD stub.
//
// EXTENSION POINTS for the pGD-model row (wave A6): PgdStub's virtual hooks
// on_link_reply_unacked() (R-RC-24 / R-LL-05: resend 14 ms, 3x, then the
// pGD's own FF-walk R-RC-25 / R-LL-18), on_lost_controller(), and the boot
// silence (R-RC-11, 17-20 s) are where the full state machine
// OFF/BOOTING/LISTEN/JOINING/SERVED/SESSIONED/PARKED/REPLY_RETRY/TOKEN_LOST
// plugs in. The stub implements only the healthy answers.

#include "../../src/plan_terminal.h"
#include "sim_bus.h"

namespace plan {
namespace sim {

class PlanTerminalStation : public Station {
 public:
  PlanTerminal term;
  int64_t turnaround_us = 250;  // firmware reply delay after the last byte
  uint32_t tx_count = 0;

  PlanTerminalStation() : Station(ENROLL_ADDR) {}

  void on_rx(const WireByte &b, int64_t t) override {
    TxAction act = term.on_byte(b.v, b.addr ? 1 : 0, t);
    if (act.kind == TxAction::NONE) return;
    Frame f;
    for (size_t i = 0; i < act.len; i++)
      f.push_back({act.frame[i], ((act.bit9_mask >> i) & 1) != 0});
    int64_t t0 = t + turnaround_us;
    bus->transmit(this, f, t0);
    tx_count++;
    pending_ = act;
    bus->at(this, t0 + CHAR_US * static_cast<int64_t>(act.len), 0);
  }
  void on_timer(int, int64_t t) override { term.tx_sent(pending_, t); }

  // Inject a keypad press into the enrolled slot (the bridge task's job).
  void press(uint8_t key) {
    uint8_t f[REPLY9_LEN];
    encode_reply9(key, 0x01, f);
    for (size_t i = 0; i < REPLY9_LEN; i++) term.tx_frame_[i] = f[i];
    term.tx_fired_ = false;
    term.tx_pending_ = true;
  }

 private:
  TxAction pending_{};
};

// Healthy pGD at 0x20 (A6 / R-RC-23, R-SE-09): silent until probed
// (R-RC-11), echoes probes with its claim in both halves (R-RC-12), answers
// the third frame of a changed-mask join with its link reply (CeCeCL,
// R-SE-09), answers polls and forwarded polls, acks session frames
// (R-SE-02), answers the ident (R-SE-03), and keeps an 8x22 screen.
class PgdStub : public Station {
 public:
  int64_t turnaround_us = 245;  // A7: ~245-250 us after the poll's stop bit
  std::array<std::string, 8> screen;
  uint32_t polls = 0, fwd_polls = 0, sacks = 0, inits = 0, idents = 0;
  bool powered = true;

  explicit PgdStub(uint8_t a = 0x20) : Station(a) {
    for (auto &r : screen) r.assign(22, ' ');
  }
  void press(uint8_t key) { key_ = key; }

  virtual void on_link_reply_unacked(int64_t) {}  // R-RC-24 (pGD-model row)
  virtual void on_lost_controller(int64_t) {}      // R-RC-25 (pGD-model row)

  void on_rx(const WireByte &b, int64_t t) override {
    if (!powered) return;
    Frame fr;
    if (asm_.push(b, fr) != 1 || !frame_ok(fr) || fr[0].v != addr) return;
    uint8_t ty = fr[1].v, me = addr;
    int64_t t0 = t + turnaround_us;
    if (ty == 0x02) {  // roll-call
      uint32_t m = get_mask(fr, 3), c = get_mask(fr, 7);
      bool had = c & bit(me);
      rc_n_ = had ? (rc_n_ ? rc_n_ + 1 : 0) : 1;
      if (rc_n_ == 3) {  // R-SE-09: third controller frame -> link reply
        rc_n_ = 0;
        return send_(mk(0x01, {0x01, me}), t0);
      }
      return send_(rollcall(0x01, me, m | bit(me), c | bit(me)), t0);  // R-RC-12
    }
    if (ty == 0x01) {  // poll `20' 01 01 DD` or forward `20' 01 1F BF`
      (fr[2].v == 0x01 ? polls : fwd_polls)++;
      Frame r = mk(0x01, {0x01, me});
      if (key_) {  // R-KP-04: report + link reply in one burst
        Frame k = mk(0x01, {0x1E, 0x07, me, key_, 0x01});
        k.insert(k.end(), r.begin(), r.end());
        r = k;
        key_ = 0;
      }
      return send_(r, t0);
    }
    if (ty == 0x50) {  // R-SE-03
      idents++;
      return send_(mk(0x01, {0x51, 0x07, me, 0x0A, 0x17}), t0);
    }
    if (ty >= 0x03) {  // R-SE-02: ack every session frame
      sacks++;
      if (ty == 0x66) inits++;
      if (ty == 0x0B && fr[4].v < 8)
        for (size_t c = 0; c < 22 && 5 + c < fr.size() - 1; c++)
          screen[fr[4].v][c] = static_cast<char>(fr[5 + c].v);
      if (ty == 0x0C && fr[4].v < 8 && fr[5].v < 22)
        screen[fr[4].v][fr[5].v] = static_cast<char>(fr[6].v);
      return send_(mk(0x01, {0x03, me}), t0);
    }
  }

 private:
  void send_(const Frame &f, int64_t t) { bus->transmit(this, f, t); }
  FrameAsm asm_;
  int rc_n_ = 0;
  uint8_t key_ = 0;
};

}  // namespace sim
}  // namespace plan
