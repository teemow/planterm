#pragma once

// Byte-level model of the CAREL uPC controller at pLAN address 0x01, built
// from the wave-A spec chapters (roll-call R-RC-*, link layer R-LL-*,
// session R-SE-*, display R-DI-*, keypad R-KP-*). Every behaviour carries its
// rule ID; the rule coverage table (tested / implemented / not covered) is in
// the PR that added this file (planterm, wave row B10).
//
// EXTENSION POINTS for later rows:
//   * fault injection: sim_bus.h FaultHook (no fault logic lives here);
//   * the pGD master walk / warm FF-walk (R-RC-07, R-LL-18): a terminal's
//     01' walk probe while DOWN makes ff_start() carry its claims (C11);
//   * slow FF-walk (R-RC-08), announce form (R-RC-06), planned rebuild
//     `01' 00` (R-LL-13), member-forwarded tokens (R-RC-16/17): not modelled.

#include "sim_bus.h"

#include <array>
#include <deque>
#include <string>

namespace plan {
namespace sim {

// --- scripted application screen (R-SE-15: one page state for all) --------
struct Page {
  std::array<std::string, 8> rows;  // 22 columns each (R-DI-05)
  uint16_t band_y;                  // lit selection band (R-DI-11)
};
struct ScreenScript {
  std::vector<Page> pages;
  size_t cur = 0;
  // Navigation for a few keys (R-KP-02 codes): Down 0x10 / Up 0x0F step
  // through the pages, Esc 0x01 returns to the anchor (page 0).
  size_t nav(uint8_t key) const {
    size_t n = pages.size();
    if (key == 0x10) return (cur + 1) % n;
    if (key == 0x0F) return (cur + n - 1) % n;
    if (key == 0x01) return 0;
    return cur;
  }
  static std::string pad(std::string s) {
    s.resize(22, ' ');
    return s;
  }
  static ScreenScript demo() {
    ScreenScript s;
    auto P = [](std::array<const char *, 8> r, uint16_t y) {
      Page p;
      for (size_t i = 0; i < 8; i++) p.rows[i] = pad(r[i]);
      p.band_y = y;
      return p;
    };
    s.pages.push_back(P({"10:00 06/10/26 Ekobe", "", "    Hotwater:   35.5\xDF" "C",
                         "    Buffer:     30.1\xDF" "C", "    Outside:    12.0\xDF" "C", "",
                         "  STATUS:  AUTO", ""}, 0));
    s.pages.push_back(P({"Main menu          1/8", " A.Unit On/Off", " B.Setpoint",
                         " C.Clock/Scheduler", " D.Input/Output", " E.Data logger",
                         " F.Change config.", " G.Service"}, 16));
    s.pages.push_back(P({"Main menu          2/8", " A.Unit On/Off", " B.Setpoint",
                         " C.Clock/Scheduler", " D.Input/Output", " E.Data logger",
                         " F.Change config.", " G.Service"}, 24));
    return s;
  }
};

class SimController : public Station {
 public:
  // Spec timings (us).
  struct Timing {
    int64_t poll_period = 24070;       // R-LL-02
    int64_t poll_window = 10000;       // R-LL-03e: accepted < ~10 ms
    int64_t ack_turn = 400;            // R-LL-03c: bare ack >= ~380 us after reply
    int64_t confirm_turn = 1100;       // R-LL-03d: confirm 1.0-1.2 ms after echo
    int64_t join_poll_turn = 540;      // R-LL-14: poll 2.84 ms after 3rd echo start
    int64_t rc_window = 7700;          // R-LL-11: answer within ~7.7 ms of frame end
    int64_t ff_step = 10000;           // R-RC-05 / R-LL-11
    int64_t ff_idle_repeat = 2300000;  // R-RC-05.4
    int64_t repoll_rejected = 19000;   // R-LL-06
    int64_t repoll_silent = 20000;     // R-LL-06
    int64_t fwd_closure = 1000;        // R-LL-19a: re-poll ~1 ms after fwd answer
    int64_t sack_window = 10000;
    int64_t sack_resend = 14000;       // R-LL-08: 14 ms after the SACK start
    int64_t sess_spacing = 14000;      // R-DI-16: 0B->0B p50 14.1 ms
    int64_t key_latency = 30000;       // R-DI-20: first content 27-395 ms after key
    int64_t link_fault = 2000000;      // R-LL-10
    int64_t gap_period = 12000000;     // R-RC-03
    int64_t gap_step = 36000;          // R-RC-03: one probe per ~36 ms macro-cycle
    int64_t exch_gap = 1000;
    int64_t takeover = 9000;           // A6 T17: 02' ~9 ms after a pGD's 01' probe
  } tm;
  // Why the last link fault happened (FaultHook::ctrl_walk_stall, A2 T1-T6).
  enum FaultCause { FC_NONE, FC_RC, FC_JOIN, FC_POLL, FC_SACK, FC_KEY };

  ScreenScript screen = ScreenScript::demo();
  uint32_t map = bit(1), claims = 0;  // settled masks (R-RC-02)
  uint8_t focus = 0;                  // R-RC-19
  bool sessioned[33] = {};
  std::array<std::array<std::string, 8>, 33> shadow{};  // R-DI-17 per terminal
  uint32_t ff_walks = 0, gap_walks = 0, link_faults = 0, polls = 0, acks = 0, joins = 0,
           sacks = 0, resends = 0, idents = 0, fwd_answered = 0, fwd_lost = 0, takeovers = 0,
           stalls = 0;
  int last_fault = FC_NONE;
  std::vector<uint8_t> keys;  // accepted key codes (R-KP-11)

  SimController() : Station(0x01) {}

  void start(int64_t t) { arm(t, K_FF); }  // cold boot = FF-walk (R-RC-05)

  // Served set = CLAIMS that are also link members (R-RC-18), in
  // terminal-table order: 32 (Trm1) before 31 (Trm2) (R-SE-11).
  std::vector<uint8_t> served() const {
    std::vector<uint8_t> r;
    for (uint8_t a = 32; a >= 2; a--)
      if ((claims & map & bit(a)) && sessioned[a]) r.push_back(a);
    return r;
  }

  // Scripted value refresh (R-DI-18): delta without 0x65 to all served.
  void set_row(size_t page, size_t row, const std::string &text, int64_t t) {
    screen.pages[page].rows[row] = ScreenScript::pad(text);
    if (page == screen.cur) paint_delta(false, t);
  }

  void on_rx(const WireByte &b, int64_t t) override {
    last_rx_end_ = t;
    if (b.addr) rx_t0_ = t - CHAR_US;
    Frame fr;
    int r = asm_.push(b, fr);
    if (r == 1 && !frame_ok(fr)) r = -1;
    if (r == -1) {
      if (!garbage_) garbage_t0_ = rx_t0_;
      garbage_ = true;
    }
    if (r == 1) frame_(fr, t);
  }

  void on_timer(int id, int64_t t) override {
    if (id != gen_) return;
    switch (kind_) {
      case K_FF:
        // A2 T7 / R-LL-12: the controller faults on its own first walk frame
        // (nothing visible on the wire), quiet 2.00 s, then walks again.
        if (last_fault != FC_NONE && bus->hook && bus->hook->ctrl_walk_stall(last_fault, t)) {
          ff_walks++;
          stalls++;
          last_fault = FC_NONE;
          Frame w = rollcall(2, 0x01, 0xFFFFFFFFu, 0);
          bus->transmit(this, w, t);
          last_tx_end_ = t + CHAR_US * static_cast<int64_t>(w.size());
          cur_tx_t0_ = t;
          phase_ = DOWN;
          await_ = A_NONE;
          arm(last_tx_end_ + tm.link_fault, K_FF);
          break;
        }
        last_fault = FC_NONE;
        ff_start(t, 0);
        break;
      case K_WAKE: schedule_(t); break;
      case K_DEADLINE: deadline_(t); break;
    }
  }

  // R-RC-05 / R-LL-11. `carried_claims` != 0 is the warm walk (R-RC-07),
  // reserved for the pGD-model row.
  void ff_start(int64_t t, uint32_t carried_claims) {
    ff_walks++;
    phase_ = FF;
    ff_t0_ = t;
    ff_map_ = 0xFFFFFFFFu;
    ff_claims_ = carried_claims;  // R-RC-05.2: cold walk starts empty
    probe_(2, t, false);
  }

 private:
  enum Kind { K_FF, K_WAKE, K_DEADLINE };
  enum Await { A_NONE, A_RC, A_JOIN, A_POLL, A_FWD, A_SACK, A_IDENT, A_REPROBE };
  enum Phase { DOWN, FF, JOIN, SERVED };
  struct Step {
    int kind;  // 0 re-probe, 1 close, 2 gap restart
    uint8_t a;
  };

  void arm(int64_t t, Kind k) {
    kind_ = k;
    bus->at(this, t, ++gen_);
  }
  void send_(const Frame &f, int64_t t, Await a, int64_t window) {
    bus->transmit(this, f, t);
    last_tx_end_ = t + CHAR_US * static_cast<int64_t>(f.size());
    cur_tx_t0_ = t;
    await_ = a;
    garbage_ = false;
    asm_.cur.clear();
    arm(last_tx_end_ + window, K_DEADLINE);
  }
  void idle_after_(const Frame &f, int64_t t) {  // a frame nobody answers
    bus->transmit(this, f, t);
    last_tx_end_ = t + CHAR_US * static_cast<int64_t>(f.size());
    cur_tx_t0_ = t;
    await_ = A_NONE;
    arm(last_tx_end_ + tm.exch_gap, K_WAKE);
  }

  // --- roll-call ---------------------------------------------------------
  void probe_(uint8_t a, int64_t t, bool gap) {
    probe_addr_ = a;
    probe_t_ = t;
    probe_gap_ = gap;
    sent_map_ = gap ? map : ff_map_;
    sent_claims_ = gap ? claims : ff_claims_;
    send_(rollcall(a, 0x01, sent_map_, sent_claims_), t, A_RC, tm.rc_window);
  }
  void probe_silent_(int64_t t) {
    if (probe_gap_) {  // R-RC-03: next gap address one macro-cycle later
      await_ = A_NONE;
      gap_addr_ = probe_addr_ + 1;
      if (gap_addr_ > gap_end_()) gap_addr_ = 0;
      gap_due_t_ = probe_t_ + tm.gap_step;
      arm(t + tm.exch_gap, K_WAKE);
      return;
    }
    ff_map_ &= ~bit(probe_addr_);  // R-RC-05.1: clear each silent address
    if (probe_addr_ >= 32) {       // R-RC-05.4: unanswered, repeat ~2.3 s
      phase_ = DOWN;
      await_ = A_NONE;
      arm(ff_t0_ + tm.ff_idle_repeat, K_FF);
      return;
    }
    probe_(probe_addr_ + 1, probe_t_ + tm.ff_step, false);  // R-LL-11
  }
  // R-RC-03: the controller's gap is 2 .. (lowest member above 1) - 1.
  uint8_t gap_end_() const {
    for (uint8_t a = 2; a <= 32; a++)
      if (map & bit(a)) return a - 1;
    return 32;
  }
  // R-RC-09: the first accepted answer ends the walk; its masks win.
  void accept_(const Frame &fr, int64_t t) {
    uint32_t m = get_mask(fr, 3) | bit(1), c = get_mask(fr, 7) & ~bit(1);  // R-RC-02
    confirms_left_ = (m == sent_map_ && c == sent_claims_) ? 1 : 2;      // R-RC-13
    map = m;
    claims = c;
    join_addr_ = probe_addr_;
    join_from_gap_ = probe_gap_;
    gap_addr_ = 0;
    phase_ = JOIN;
    confirm_(t + tm.confirm_turn);
  }
  void confirm_(int64_t t) {  // R-RC-13: re-send the probe with merged masks
    send_(rollcall(join_addr_, 0x01, map, claims), t, A_JOIN, tm.rc_window);
  }
  void join_answer_(const Frame &fr, int64_t t) {
    if (fr[1].v == 0x02) {  // the joiner echoes each confirm identically
      map = get_mask(fr, 3) | bit(1);
      claims = get_mask(fr, 7) & ~bit(1);
    }
    if (--confirms_left_ > 0) return confirm_(t + tm.confirm_turn);
    join_poll_ = true;  // R-LL-14: then poll, link reply, `01'`
    poll_(join_addr_, t + tm.join_poll_turn, true);
  }
  void join_done_(int64_t t) {
    joins++;
    join_poll_ = false;
    phase_ = SERVED;
    focus = join_addr_;  // R-RC-19: the most recent joiner is the poll focus
    next_poll_t_ = poll_t0_ + tm.poll_period;
    next_gap_t_ = t + tm.gap_period;
    post_.clear();
    if (join_from_gap_) {  // R-RC-14: re-probe other claimed members, joiner, close
      for (uint8_t a = 32; a >= 2; a--)
        if ((claims & bit(a)) && a != focus) post_.push_back({0, a});
      post_.push_back({0, focus});
      post_.push_back({1, 0});
      post_.push_back({2, 0});
    }
    start_sessions_(focus, t);
  }

  // --- poll ----------------------------------------------------------------
  void poll_(uint8_t a, int64_t t, bool fresh) {
    polls++;
    if (fresh) poll_attempt_ = 1, poll_t0_ = t;
    polled_ = a;
    key_ = -1;
    send_(mk(a, {0x01, 0x01}), t, A_POLL, tm.poll_window);
  }
  void poll_ok_(int64_t t) {  // R-LL-04: the bare `01'` acks the link reply
    acks++;
    Frame ack{{0x01, true}};
    bus->transmit(this, ack, t + tm.ack_turn);
    last_tx_end_ = t + tm.ack_turn + CHAR_US;
    cur_tx_t0_ = t + tm.ack_turn;
    await_ = A_NONE;
    if (key_ >= 0) apply_key_(static_cast<uint8_t>(key_), t);  // R-KP-11
    if (join_poll_) join_done_(t);
    arm(last_tx_end_ + tm.exch_gap, K_WAKE);
  }
  void apply_key_(uint8_t k, int64_t t) {  // R-KP-17 / R-SE-15: one shared page
    keys.push_back(k);
    size_t to = screen.nav(k);
    if (to == screen.cur) return;
    screen.cur = to;
    if (sess_due_t_ < t + tm.key_latency) sess_due_t_ = t + tm.key_latency;  // R-DI-20
    paint_delta(true, t);
  }

  // --- session / display -----------------------------------------------------
  void start_sessions_(uint8_t joiner, int64_t t) {
    // R-RC-15 / R-SE-11: existing claimed terminals re-init first (no new
    // ident), then the joiner gets 0A, ident, init, repaint (R-SE-10/12).
    for (uint8_t a = 32; a >= 2; a--)
      if ((claims & map & bit(a)) && a != joiner && sessioned[a]) full_init_(a, false);
    for (uint8_t a = 32; a >= 2; a--)
      if ((claims & map & bit(a)) && (a == joiner || !sessioned[a])) full_init_(a, true);
    if (sess_due_t_ < t) sess_due_t_ = t;
  }
  void full_init_(uint8_t a, bool ident) {  // R-SE-05 / R-DI-15
    sessioned[a] = true;
    if (ident) {
      sq_.push_back({a, mk(a, {0x0A, 0x06, 0x01, 0x00})});  // R-SE-04 (VV = 00)
      sq_.push_back({a, mk(a, {0x50, 0x05, 0x01})});        // R-SE-03
    }
    sq_.push_back({a, mk(a, {0x66, 0x08, 0x01, 0x00, 0x01})});
    sq_.push_back({a, mk(a, {0x65, 0x0F, 0x01, 0x01, 0, 0, 0, 0, 0, 0, 0, 0})});
    sq_.push_back({a, mk(a, {0x0D, 0x08, 0x01, 0x00, 0x00, 0x00})});
    sq_.push_back({a, mk(a, {0x0E, 0x08, 0x01, 0x80, 0x00, 0x00})});  // R-SE-07
    sq_.push_back({a, mk(a, {0x0F, 0x06, 0x01, 0x00})});              // R-SE-08
    const Page &p = screen.pages[screen.cur];
    for (uint8_t r = 0; r < 8; r++) {  // R-DI-15: all eight rows, blank included
      sq_.push_back({a, row_(a, r, p.rows[r])});
      shadow[a][r] = p.rows[r];
    }
    for (auto &g : band_(a, p.band_y)) sq_.push_back({a, g});  // graphics last
  }
  Frame row_(uint8_t a, uint8_t r, const std::string &s) {  // R-DI-05
    std::vector<uint8_t> b{0x0B, 0x1C, 0x01, r};
    for (char c : s) b.push_back(static_cast<uint8_t>(c));
    return mk(a, b);
  }
  // R-DI-10: a 132x16 band = 264 bytes in fragments of <= 128 data bytes.
  std::vector<Frame> band_(uint8_t a, uint16_t y) {
    std::vector<Frame> out;
    const uint16_t w = 132, h = 16, total = w * ((h + 7) / 8), id = y ? 0x1504 : 0x0080;
    for (uint16_t off = 0; off < total; off += 128) {
      uint16_t n = std::min<uint16_t>(128, total - off);
      std::vector<uint8_t> b{0x64, static_cast<uint8_t>(21 + n), 0x01,
                             static_cast<uint8_t>(id >> 8), static_cast<uint8_t>(id),
                             static_cast<uint8_t>(n >> 8), static_cast<uint8_t>(n),
                             static_cast<uint8_t>(off + n < total ? 1 : 0),
                             static_cast<uint8_t>(off >> 8), static_cast<uint8_t>(off),
                             0, 0, static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(y),
                             0, static_cast<uint8_t>(w), 0, static_cast<uint8_t>(h)};
      for (uint16_t i = 0; i < n; i++) b.push_back(0xFF);
      out.push_back(mk(a, b));
    }
    return out;
  }

 public:
  // R-DI-12/16/17/08 + R-DI-31: a navigation (with_65) or value refresh,
  // delta against each served terminal's shadow, mirrored 0x20 first.
  void paint_delta(bool with_65, int64_t t) {
    const Page &p = screen.pages[screen.cur];
    std::vector<std::pair<uint8_t, std::vector<Frame>>> per;
    for (uint8_t a : served()) {
      std::vector<Frame> l;
      if (with_65) l.push_back(mk(a, {0x65, 0x0F, 0x01, 0x01, 0, 0, 0, 0, 0, 0, 0, 0}));
      for (uint8_t r = 0; r < 8; r++) {
        const std::string &old = shadow[a][r];
        int diff = 0, col = 0;
        for (int c = 0; c < 22; c++)
          if (old.size() != 22 || old[c] != p.rows[r][c]) diff++, col = c;
        if (diff == 1)  // R-DI-08: exactly one changed character -> 0x0C
          l.push_back(mk(a, {0x0C, 0x08, 0x01, r, static_cast<uint8_t>(col),
                             static_cast<uint8_t>(p.rows[r][col])}));
        else if (diff > 1)
          l.push_back(row_(a, r, p.rows[r]));
        shadow[a][r] = p.rows[r];
      }
      if (with_65)
        for (auto &g : band_(a, p.band_y)) l.push_back(g);
      per.push_back({a, l});
    }
    for (size_t i = 0;; i++) {  // R-DI-31: interleave 20, 1F, 20, 1F
      bool any = false;
      for (auto &pl : per)
        if (i < pl.second.size()) sq_.push_back({pl.first, pl.second[i]}), any = true;
      if (!any) break;
    }
    if (await_ == A_NONE && phase_ == SERVED) arm(std::max(t, bus->now) + tm.exch_gap, K_WAKE);
  }

 private:
  // --- link fault (R-LL-10) ------------------------------------------------
  void fault_(int cause) {
    link_faults++;
    last_fault = cause;
    join_poll_ = false;
    phase_ = DOWN;
    await_ = A_NONE;
    focus = 0;
    gap_addr_ = 0;
    post_.clear();
    sq_.clear();
    for (bool &s : sessioned) s = false;
    arm(std::max(last_tx_end_, last_rx_end_) + tm.link_fault, K_FF);
  }

  // --- dispatch ----------------------------------------------------------------
  void frame_(const Frame &fr, int64_t t) {
    uint8_t to = fr[0].v, ty = fr[1].v;
    // R-LL-05/06: a frame that began before our latest transmission answers
    // an older exchange (the pGD's 14 ms resend before our 19 ms re-poll):
    // never accepted.
    if (rx_t0_ < cur_tx_t0_) return;
    // A6 T17 / R-LL-18 / R-RC-07: a terminal's own walk probe to 01' while we
    // are down: take the token and continue the walk with its claims.
    if (to == 0x01 && ty == 0x02 && fr[2].v != 0x01 && phase_ == DOWN) {
      takeovers++;
      return ff_start(t + tm.takeover, get_mask(fr, 7) & ~bit(1));
    }
    if (to != 0x01) {
      // R-RC-19 / R-LL-19a: the focus hands its poll token on (`20' 01 1F BF`).
      // R-LL-17: the first poll after a join must get the joiner's own link
      // reply; a forward there is not an answer (the join dies).
      if (await_ == A_POLL && join_poll_) {
        garbage_ = true;
        return;
      }
      if (await_ == A_POLL && ty == 0x01 && fr[2].v == polled_) {
        await_ = A_FWD;
        fwd_to_ = to;
        key_ = -1;
        arm(t + tm.poll_window, K_DEADLINE);
      }
      return;
    }
    switch (await_) {
      case A_RC:
        if (ty == 0x02 && fr[2].v == probe_addr_) return accept_(fr, t);
        garbage_ = true;
        return;
      case A_JOIN:
        if ((ty == 0x02 || ty == 0x01) && fr[2].v == join_addr_) return join_answer_(fr, t);
        garbage_ = true;
        return;
      case A_REPROBE:
        if (ty == 0x02 && fr[2].v == probe_addr_) {
          await_ = A_NONE;
          arm(t + tm.exch_gap, K_WAKE);
        }
        return;
      case A_POLL:
        if (ty == 0x1E && fr[3].v == polled_) key_ = fr[4].v;  // R-KP-01/04
        else if (ty == 0x01 && fr[2].v == polled_) poll_ok_(t);
        else garbage_ = true;
        return;
      case A_FWD:
        if (ty == 0x1E && fr[3].v == fwd_to_) key_ = fr[4].v;
        else if (ty == 0x01 && fr[2].v == fwd_to_) {
          // R-LL-19a: the answer to a forwarded poll is NOT acked; the
          // controller re-polls the forwarder ~1 ms later (R-RC-19).
          fwd_answered++;
          if (key_ >= 0) apply_key_(static_cast<uint8_t>(key_), t);  // R-KP-17
          poll_(focus, t + tm.fwd_closure, true);
        }
        return;
      case A_SACK:
        if (ty == 0x03 && fr[2].v == sess_to_) return sess_ok_(t);
        garbage_ = true;
        return;
      case A_IDENT:  // R-SE-03: `01' 51 07 <t> 0A 17 CK`, no ack follows
        if (ty == 0x51 && fr[3].v == sess_to_ && fr[4].v == 0x0A && fr[5].v == 0x17) {
          idents++;
          return sess_ok_(t);
        }
        garbage_ = true;
        return;
      default:
        return;
    }
  }
  void sess_ok_(int64_t t) {
    sacks++;
    await_ = A_NONE;
    sq_.pop_front();
    sess_due_t_ = sess_t0_ + tm.sess_spacing;
    arm(t + tm.exch_gap, K_WAKE);
  }

  void deadline_(int64_t t) {
    bool heard = garbage_ || !asm_.cur.empty();
    switch (await_) {
      case A_RC:
        if (heard) return fault_(FC_RC);  // R-RC-10: a rejected answer aborts the walk
        return probe_silent_(t);
      case A_JOIN:
        return fault_(FC_JOIN);  // R-RC-13 / R-LL-16: an unaccepted echo is never retried
      case A_REPROBE:
        if (heard) return fault_(FC_RC);
        await_ = A_NONE;
        return schedule_(t);
      case A_POLL:
        if (key_ >= 0) return fault_(FC_KEY);  // R-KP-04: report without link reply
        if (join_poll_) return fault_(FC_JOIN);  // R-RC-13: any join stage lost -> fault
        if (poll_attempt_ >= 3) return fault_(FC_POLL);  // R-LL-06 -> R-LL-10
        poll_attempt_++;
        return poll_(polled_, poll_t0_ + (poll_attempt_ - 1) *
                                  (heard ? tm.repoll_rejected : tm.repoll_silent), false);
      case A_FWD:  // nobody answered the forward: close the cycle ourselves
        fwd_lost++;
        return poll_(focus, t, true);
      case A_SACK:
      case A_IDENT:
        // R-LL-08: an ack that was heard but not accepted -> identical copy
        // 14 ms after the SACK start, at most 3 copies. No ack at all, or
        // the 3rd copy rejected -> link fault (R-SE-14, R-LL-08).
        if (!heard || copies_ >= 3) return fault_(FC_SACK);
        copies_++;
        resends++;
        send_(sq_.front().f, std::max(t, garbage_t0_ + tm.sack_resend),
              await_, tm.sack_window);
        return;
      default:
        return;
    }
  }

  // The steady-state scheduler: post-join steps, the 24.07 ms poll, the
  // ~12 s gap walk, then session frames.
  void schedule_(int64_t t) {
    if (phase_ != SERVED || await_ != A_NONE) return;
    if (!post_.empty()) {
      Step s = post_.front();
      post_.pop_front();
      if (s.kind == 0) {  // R-RC-14.1/.2: members echo verbatim
        probe_addr_ = s.a;
        return send_(rollcall(s.a, 0x01, map, claims), t, A_REPROBE, tm.rc_window);
      }
      if (s.kind == 1)  // R-RC-14.3: closing frame to itself
        return idle_after_(rollcall(0x01, 0x01, map, claims), t);
      gap_walks++;  // R-RC-14.4: restart the gap walk at 02 straight away
      next_gap_t_ = t + tm.gap_period;
      gap_addr_ = 2;
      return probe_(2, t, true);
    }
    if (t >= next_poll_t_) {  // R-LL-02
      next_poll_t_ += tm.poll_period;
      if (next_poll_t_ < t) next_poll_t_ = t + tm.poll_period;
      return poll_(focus, t, true);
    }
    if (!gap_addr_ && t >= next_gap_t_) {  // R-RC-03: every 12.0 s
      gap_walks++;
      next_gap_t_ += tm.gap_period;
      gap_addr_ = gap_end_() >= 2 ? 2 : 0;
      gap_due_t_ = t;
    }
    if (gap_addr_ && t >= gap_due_t_) return probe_(gap_addr_, t, true);
    if (!sq_.empty() && t >= sess_due_t_) {
      const auto &it = sq_.front();
      if (!sessioned[it.to]) {
        sq_.pop_front();
        return schedule_(t);
      }
      sess_to_ = it.to;
      sess_t0_ = t;
      copies_ = 1;
      return send_(it.f, t, it.f[1].v == 0x50 ? A_IDENT : A_SACK, tm.sack_window);
    }
    int64_t w = next_poll_t_;
    if (gap_addr_) w = std::min(w, gap_due_t_);
    else w = std::min(w, next_gap_t_);
    if (!sq_.empty()) w = std::min(w, sess_due_t_);
    arm(std::max(w, t + 1), K_WAKE);
  }

  struct SItem {
    uint8_t to;
    Frame f;
  };
  FrameAsm asm_;
  int gen_ = 0;
  Kind kind_ = K_WAKE;
  Await await_ = A_NONE;
  Phase phase_ = DOWN;
  bool garbage_ = false, probe_gap_ = false, join_from_gap_ = false, join_poll_ = false;
  int64_t rx_t0_ = 0, cur_tx_t0_ = 0, garbage_t0_ = 0, last_tx_end_ = 0, last_rx_end_ = 0;
  int64_t ff_t0_ = 0, probe_t_ = 0, poll_t0_ = 0, next_poll_t_ = 0, next_gap_t_ = 0,
          gap_due_t_ = 0, sess_due_t_ = 0, sess_t0_ = 0;
  uint32_t ff_map_ = 0, ff_claims_ = 0, sent_map_ = 0, sent_claims_ = 0;
  uint8_t probe_addr_ = 0, join_addr_ = 0, polled_ = 0, fwd_to_ = 0, sess_to_ = 0,
          gap_addr_ = 0;
  int confirms_left_ = 0, poll_attempt_ = 0, copies_ = 0, key_ = -1;
  std::deque<Step> post_;
  std::deque<SItem> sq_;
};

}  // namespace sim
}  // namespace plan
