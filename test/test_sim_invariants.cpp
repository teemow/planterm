// Scenario + invariant suite for the pLAN bridge (wave row D12).
//
// The real PlanTerminal at 0x1F, driven exactly like plan_bridge_isr.cpp
// drives it (one on_byte per received byte; an action goes on the wire only
// if no further byte arrived during the 250 us turnaround; the receiver is
// muted while DE is up), runs against the simulated controller
// (sim_controller.h) and the pGD model (sim_pgd.h) at 0x20 under the seeded
// physical-fault model (sim_faults.h). Every scenario runs for several seeds
// and, for the comparative invariants, a second time with the bridge absent
// (same seed, same faults).
//
// Invariants (rule IDs from the wave chapters A1-A8):
//   I1 pGD served: never locked out by us. The pGD's longest unpainted span
//      with the bridge stays within the bridge-absent span + one gap-walk
//      period (R-RC-03, 12 s), floor 15 s (R-RC-30, R-RC-32, A8 R3/R4).
//   I2 walks: the bridge never causes more FF-walks than the same bus without
//      it (A8 R5-R7, R-RC-30); slack: 20 % of the absent count (RNG streams
//      diverge once the bridge transmits).
//   I3 TX discipline: no transmission while passive, every transmission
//      starts within 2 ms of the end of a frame addressed to 0x1F and never
//      overlaps another station's frame (R-LL-03, A7).
//   I4 recovery: once loss is below the healthy threshold (p <= 5 %), the
//      bridge is served again (sessioned, claimed) within 30 s (R-RC-03 gap
//      walk + FF-walk repeat).
//   I5 published values: at every settle instant (600 ms quiet after a
//      display frame to 0x1F, the SettleGate rule) every row of the bridge's
//      own PlanScreen equals a row the controller displayed on its current
//      page within the last 6 s (A4 W-01, W-02, W-05, W-09).
//   I6 keys and edits: (a) a key request goes on the wire at most once and
//      executes at most once (A5 F-06/F-07); (b) never in the first poll
//      after a walk (F-03); (c) an edit / alarm acknowledge reports success
//      only when the controller holds the committed value (F-13, F-19).
//   I7 scrape walk speed: the walk on a healthy bus never gets slower than
//      today's (the owner's hard rule). Today's durations are the limits.
//
// The task-side key pump and the edit step engine are host models of
// plan_bridge.cpp (task_main key path, repeat 1, tx_mode 2) and of the
// NavEngine step semantics (settle 600 ms quiet, 6 s cap fall-through, 2 s
// verify): those parts are not host-compilable today. Expected failures of
// today's firmware live in test/sim/invariants_expected.tsv: the test fails
// on a NEW failure and on an expected failure that now PASSES (flip the row).
//
//   test_sim_invariants              check against the table
//   test_sim_invariants --emit       print today's table rows
//   test_sim_invariants --matrix     print the scenario x invariant matrix
//   test_sim_invariants --detail S N print every verdict of scenario S, seed N

#include "../src/plan_screen.h"
#include "sim/sim_bus.h"
#include "sim/sim_controller.h"
#include "sim/sim_faults.h"
#include "sim/sim_pgd.h"
#include "sim/sim_terminals.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace plan;
using namespace plan::sim;

static const int64_t S = 1000000, MS = 1000;
static const uint64_t SEEDS[] = {1, 2, 3};

// --- the bridge as the ISR drives PlanTerminal -------------------------------
class IsrBridge : public Station {
 public:
  std::unique_ptr<PlanTerminal> term{new PlanTerminal};
  PlanScreen scr;
  bool powered = true;
  int64_t turnaround_us = 250;
  uint32_t tx_passive = 0, stale = 0, muted = 0;
  int64_t last_disp_t = -1;  // last display frame to 0x1F heard
  bool disp_dirty = false;

  IsrBridge() : Station(ENROLL_ADDR) {}
  // Power-on / reboot: fresh state machine, the device's boot config.
  void boot() {
    term.reset(new PlanTerminal);
    term->enroll_ = true;
    term->fwd_polls_ = 1;
    scr = PlanScreen();
    has_ = false;
    ++gen_;
    powered = true;
  }
  void power_off() {
    powered = false;
    has_ = false;
    ++gen_;
  }
  void on_rx(const WireByte &b, int64_t t) override {
    if (!powered) return;
    if (t - CHAR_US < tx_end_) {  // DE+RE tied: deaf while driving
      muted++;
      return;
    }
    if (has_) {  // a byte landed in the RX FIFO during the turnaround
      has_ = false;
      stale++;
      term->tx_not_sent(act_);
    }
    if (b.addr) to_ = b.v, idx_ = 0;
    else if (++idx_ == 1 && to_ == ENROLL_ADDR &&
             (b.v == 0x0B || b.v == 0x0C || b.v == 0x64 || b.v == 0x65))
      last_disp_t = t, disp_dirty = true;
    scr.feed(b.v, b.addr ? 1 : 0, static_cast<uint32_t>(t / 1000));
    TxAction a = term->on_byte(b.v, b.addr ? 1 : 0, t);
    if (a.kind == TxAction::NONE) return;
    act_ = a;
    has_ = true;
    bus->at(this, t + turnaround_us, 2 * (++gen_));
  }
  void on_timer(int id, int64_t t) override {
    if (id == 1) {  // DE drop
      if (sent_gen_ == gen_) term->tx_sent(sent_, t);
      return;
    }
    if (!has_ || id != 2 * gen_) return;
    has_ = false;
    if (!term->enroll_ && !term->drain_) tx_passive++;
    Frame f;
    for (size_t i = 0; i < act_.len; i++) f.push_back({act_.frame[i], ((act_.bit9_mask >> i) & 1) != 0});
    bus->transmit(this, f, t);
    tx_end_ = t + CHAR_US * static_cast<int64_t>(act_.len);
    sent_ = act_;
    sent_gen_ = gen_;
    bus->at(this, tx_end_, 1);
  }

 private:
  TxAction act_{}, sent_{};
  bool has_ = false;
  int gen_ = 0, sent_gen_ = -1;
  int64_t tx_end_ = 0;
  uint8_t to_ = 0;
  int idx_ = 0;
};

// --- task model: the key pump of plan_bridge.cpp task_main (tx_mode 2) ------
struct KeyPump : Station {
  struct Req {
    uint8_t key;
    int64_t t_req, t_start = -1, t_done = -1;
    bool ok = false;
    size_t keys_before = 0, keys_after = 0;
  };
  IsrBridge *br = nullptr;
  SimController *ctl = nullptr;
  std::vector<Req> reqs;
  enum { IDLE, ARMED, VERDICT, WAIT } st = IDLE;
  size_t cur = 0;
  int retries = 0, vticks = 0;
  int64_t deadline = 0, vt = 0, wait_until = 0;

  KeyPump() : Station(0) {}
  void start(int64_t t) { bus->at(this, t, 0); }
  void request(uint8_t k, int64_t t) { reqs.push_back({k, t}); }
  bool idle() const { return st == IDLE && cur == reqs.size(); }
  void reset() {  // the task restarts with the bridge
    for (size_t i = cur; i < reqs.size(); i++) reqs[i].t_done = -2;
    cur = reqs.size();
    st = IDLE;
  }
  void arm_(PlanTerminal &T) {
    uint8_t f[REPLY9_LEN];
    encode_reply9(reqs[cur].key, 0x01, f);
    for (size_t i = 0; i < REPLY9_LEN; i++) T.tx_frame_[i] = f[i];
    T.tx_fired_ = false;
    T.tx_pending_ = true;
  }
  void done_(bool ok, int64_t t) {
    reqs[cur].ok = ok;
    reqs[cur].t_done = t;
    reqs[cur].keys_after = ctl->keys.size();
    cur++;
    st = IDLE;
  }
  void on_timer(int, int64_t t) override {
    bus->at(this, t + 5 * MS, 0);
    if (!br->powered) return;
    PlanTerminal &T = *br->term;
    switch (st) {
      case IDLE:
        if (cur == reqs.size() || reqs[cur].t_req > t) return;
        reqs[cur].t_start = t;
        reqs[cur].keys_before = ctl->keys.size();
        retries = 0;
        T.tx_rejected_ = false;
        T.link_reset_ = false;
        deadline = t + 2 * S;
        arm_(T);
        st = ARMED;
        return;
      case ARMED:
        if (T.tx_fired_) {
          T.tx_fired_ = false;
          st = VERDICT;
          vt = t;
          vticks = 0;
        } else if (t >= deadline) {  // "no clean slot": tx_pending_ stays armed
          if (++retries <= 10) deadline = t + 2 * S;
          else T.tx_pending_ = false, done_(false, t);
        }
        return;
      case VERDICT:
        if (t - vt < 20 * MS * (vticks + 1)) return;
        vticks++;
        if (T.tx_rejected_ || T.link_reset_) {
          bool rs = T.link_reset_;
          T.tx_rejected_ = false;
          T.link_reset_ = false;
          if (++retries <= 10) wait_until = t + (rs ? 400 : 600) * MS, st = WAIT;
          else done_(false, t);
        } else if (vticks >= 2) {
          done_(true, t);  // "accepted": 40 ms without a re-poll or walk marker
        }
        return;
      case WAIT:
        if (t < wait_until) return;
        deadline = t + 2 * S;
        arm_(T);
        st = ARMED;
        return;
    }
  }
};

// --- task model: NavEngine-style step engine (press, settle, verify) --------
struct Step {
  uint8_t key;
  int row;  // -1: no read-back (a blind predicate)
  const char *expect;
};
struct Macro {
  const std::vector<Step> *steps = nullptr;
  size_t i = 0;
  enum { OFF, PRESS, SETTLE, VERIFY, DONE } st = OFF;
  int64_t t_begin = 0, t_end = -1, step_t = 0, verify_t = 0;
  bool ok = false;
  size_t req = 0;

  void start(const std::vector<Step> *s, int64_t t) {
    steps = s;
    i = 0;
    st = PRESS;
    t_begin = t;
  }
  void tick(int64_t t, KeyPump &p, const IsrBridge &br) {
    switch (st) {
      case PRESS:
        req = p.reqs.size();
        p.request((*steps)[i].key, t);
        st = SETTLE;
        step_t = -1;
        return;
      case SETTLE:
        if (step_t < 0) {
          if (p.reqs[req].t_done == -1) return;
          if (!p.reqs[req].ok) return finish_(false, t);
          step_t = t;
        }
        // NAV_QUIET_MS after the last display frame, capped by
        // NAV_SETTLE_CAP_MS (W-05: the cap falls through to the read).
        if ((t - step_t >= 600 * MS && t - br.last_disp_t >= 600 * MS) || t - step_t >= 6 * S)
          st = VERIFY, verify_t = t;
        return;
      case VERIFY: {
        const Step &s = (*steps)[i];
        bool pass = s.row < 0 || strstr(br.scr.row(SCR_TERM_ESP, s.row), s.expect) != nullptr;
        if (!pass && t - verify_t < 2 * S) return;  // NAV_VERIFY_MS
        if (!pass) return finish_(false, t);
        if (++i == steps->size()) return finish_(true, t);
        st = PRESS;
        return;
      }
      default:
        return;
    }
  }
  void finish_(bool k, int64_t t) {
    ok = k;
    t_end = t;
    st = DONE;
  }
};

// --- scenarios ------------------------------------------------------------------
enum MacroKind { MK_NONE, MK_KEYS, MK_EDIT, MK_ALARM, MK_WALK };
struct Scenario {
  const char *name;
  double p = 0;            // terminal -> controller loss
  int64_t dur = 150 * S;
  int64_t eval_t0 = 30 * S;
  int64_t calm_t = -1;     // I4: loss healthy from here on (-1: not applicable)
  MacroKind mk = MK_NONE;
  int kind = 0;            // 0 plain, 1 cold boot, 2 bridge boot, 3 reset, 4 pGD cycle,
                           // 5 ctrl silent, 6 drift, 7 enrol toggles, 8 recovery,
                           // 9 the commit Enter's keypad report rejected once
};

static std::vector<Scenario> scenarios() {
  std::vector<Scenario> v;
  v.push_back({"cold_boot", 0, 120 * S, 0, 0, MK_NONE, 1});
  v.push_back({"bridge_boot", 0, 150 * S, 30 * S, 30 * S, MK_NONE, 2});
  v.push_back({"bridge_reset_noise", 0, 180 * S, 60 * S, 62 * S, MK_NONE, 3});
  v.push_back({"pgd_power_cycle", 0, 180 * S, 60 * S, 85 * S, MK_NONE, 4});
  v.push_back({"ctrl_silent_r9", 0, 180 * S, 90 * S, 90 * S, MK_NONE, 5});
  static const double P[] = {0, 0.01, 0.05, 0.15, 0.28, 0.45};
  static const char *PN[] = {"00", "01", "05", "15", "28", "45"};
  static std::string names[3][6];
  const char *pre[3] = {"loss_p", "edit_p", "alarm_p"};
  MacroKind mks[3] = {MK_NONE, MK_EDIT, MK_ALARM};
  for (int k = 0; k < 3; k++)
    for (int i = 0; i < 6; i++) {
      names[k][i] = std::string(pre[k]) + PN[i];
      v.push_back({names[k][i].c_str(), P[i], 150 * S, 30 * S, P[i] <= 0.05 ? 0 : -1, mks[k], 0});
    }
  v.push_back({"idle_drift", 0, 150 * S, 30 * S, -1, MK_NONE, 6});
  v.push_back({"enrol_toggle", 0, 220 * S, 30 * S, 160 * S, MK_NONE, 7});
  v.push_back({"keys_during_walks_p15", 0.15, 150 * S, 30 * S, -1, MK_KEYS, 0});
  v.push_back({"edit_commit_lost", 0, 150 * S, 30 * S, 0, MK_EDIT, 9});
  v.push_back({"recovery_p28_to_p01", 0.28, 180 * S, 30 * S, 90 * S, MK_NONE, 8});
  v.push_back({"scrape_walk_p00", 0, 120 * S, 30 * S, 0, MK_WALK, 0});
  v.push_back({"scrape_walk_p01", 0.01, 120 * S, 30 * S, 0, MK_WALK, 0});
  return v;
}

static const std::vector<Step> EDIT = {
    {KEY_DOWN, -1, nullptr}, {KEY_DOWN, -1, nullptr}, {KEY_DOWN, 0, "Setpoint"},
    {KEY_ENTER, -1, nullptr},             // focus: blind (F-12)
    {KEY_UP, 3, "46.0"},                  // candidate shown
    {KEY_ENTER, 3, "46.0"},               // commit: read-back of the shown value (F-13)
    {KEY_ESC, -1, nullptr}, {KEY_ESC, 0, "10:00"}};
static const std::vector<Step> ALARM = {
    {KEY_ALARM, 0, "Alarms"}, {KEY_ALARM, 2, "No active alarm"}, {KEY_ESC, 0, "10:00"}};
static const std::vector<Step> WALK = {
    {KEY_DOWN, 0, "Main menu          1/8"}, {KEY_DOWN, 0, "Main menu          2/8"},
    {KEY_ESC, 0, "10:00"}};

// The keypad report of the 2nd Enter (the edit's commit) is rejected at the
// controller once while the link reply in the same burst is accepted: the
// per-frame rejection of R-LL-21 hitting exactly one key (A5 F-02/F-13).
class CommitLoss : public PhysFault {
 public:
  using PhysFault::PhysFault;
  int enters = 0;
  bool armed = false;
  void on_tx_frame(uint8_t from, Frame &f, int64_t t) override {
    PhysFault::on_tx_frame(from, f, t);
    if (from == ENROLL_ADDR && f.size() > 4 && f[1].v == 0x1E && f[4].v == KEY_ENTER && ++enters == 2)
      armed = true;
  }
  bool on_rx_byte(uint8_t from, uint8_t to, WireByte &b, int64_t t) override {
    if (armed && from == ENROLL_ADDR && to == 0x01 && b.addr) {
      armed = false;
      b.err = true;
      return true;
    }
    return PhysFault::on_rx_byte(from, to, b, t);
  }
};

static std::string pad22(const std::string &s) {
  std::string r = s;
  r.resize(22, ' ');
  return r;
}

// --- one run -------------------------------------------------------------------
struct Snap {
  int64_t t;
  std::array<std::string, 8> rows;
};
struct Result {
  int64_t dark_max = 0;
  int walks = 0;
  uint32_t tx_passive = 0, tx_outside = 0;
  std::string i3_msg;
  int64_t recover = -1;
  int i5_bad = 0;
  std::string i5_msg;
  int i6_multi = 0, i6_first = 0, i6_false_ok = 0;
  std::string i6_msg;
  bool mac_ok = false, mac_done = false;
  int64_t walk_us = -1;
};

static bool is_ffwalk(const Frame &f) {
  return f.size() == 12 && f[0].v == 0x02 && f[1].v == 0x02 && f[2].v == 0x01 && f[3].v == 0xFF &&
         f[4].v == 0xFF && f[5].v == 0xFF && f[6].v == 0xFF;
}

static Result run(const Scenario &sc, uint64_t seed, bool with_bridge) {
  Bus bus;
  SimController ctl;
  PgdModel pgd(sc.kind != 1);
  IsrBridge br;
  KeyPump pump;
  Macro mac;
  FaultSpec fs;
  fs.seed = seed;
  fs.p_to_ctrl = sc.p;
  if (sc.p >= 0.28) fs.p_walk_stall = 1.0;  // A2 T7, calibrated with R8 (C11)
  if (sc.kind == 6) fs.drift.push_back({0x01, 0.3, 2 * MS, 8 * MS, false});
  if (sc.kind == 3 && with_bridge) fs.reset_at = 60 * S, fs.reset_us = 50 * MS;
  if (sc.kind == 5) fs.ctrl_silent_at = 60 * S;
  CommitLoss f(fs);
  if (sc.kind != 9) f.enters = -1000;  // never arms
  bus.attach(&ctl);
  bus.attach(&pgd);
  if (with_bridge) bus.attach(&br);
  f.attach(bus);
  pgd.seed(seed);
  pump.bus = &bus;
  pump.br = &br;
  pump.ctl = &ctl;

  // Application pages: 3 = a setpoint, 4 = the alarm list (A5 edit model:
  // the candidate is shown before the commit; Esc restores the committed).
  ctl.screen.pages[0].rows[6] = pad22("  STATUS:  Alarm-On");
  ctl.screen.pages.push_back(ctl.screen.pages[1]);
  ctl.screen.pages[3].rows = {pad22("Setpoint          B01"), pad22(""), pad22(""),
                              pad22("    Heating:    45.0 C"), pad22(""), pad22(""), pad22(""),
                              pad22("")};
  ctl.screen.pages.push_back(ctl.screen.pages[1]);
  ctl.screen.pages[4].rows = {pad22("Alarms            AL1"), pad22(""),
                              pad22("AL04 Low superheat"), pad22(""), pad22(""), pad22(""),
                              pad22(""), pad22("")};
  double committed = 45.0, cand = 45.0;
  bool focus = false, alarm = true;
  auto sp_row = [&](int64_t t) {
    char b[32];
    snprintf(b, sizeof b, "    Heating:    %4.1f C", focus ? cand : committed);
    ctl.set_row(3, 3, b, t);
  };
  ctl.on_key = [&](uint8_t k, int64_t t) {
    if (ctl.screen.cur == 3) {
      if (k == KEY_ENTER) {
        if (focus) committed = cand, focus = false;
        else focus = true, cand = committed;
        sp_row(t);
        return true;
      }
      if (focus && (k == KEY_UP || k == KEY_DOWN)) {
        cand += k == KEY_UP ? 1.0 : -1.0;
        sp_row(t);
        return true;
      }
      if (focus && k == KEY_ESC) {
        focus = false;
        cand = committed;
        sp_row(t);
        return true;
      }
    }
    if (k == KEY_ALARM) {
      if (ctl.screen.cur != 4) {
        ctl.screen.cur = 4;
        ctl.paint_delta(true, t);
      } else if (alarm) {
        alarm = false;
        ctl.screen.pages[0].rows[6] = pad22("  STATUS:  AUTO");
        ctl.set_row(4, 2, "No active alarm", t);
      }
      return true;
    }
    return false;
  };

  if (sc.kind == 1) pgd.power_on(0, true);
  ctl.start(0);
  // The bridge powers up with the controller only in cold_boot; everywhere
  // else it boots into a running pGD-only bus (5 s, bridge_boot: 30 s) and
  // joins on the gap walk, the July path to DUAL_SERVED (R-RC-14).
  const int64_t br_boot = sc.kind == 1 ? 0 : sc.kind == 2 ? 30 * S : 5 * S;
  if (with_bridge) {
    br.boot();
    if (br_boot > 0) br.power_off();
    pump.start(0);
  }

  std::vector<Snap> hist;
  std::vector<int64_t> served31;
  int value = 0;
  int64_t next_val = 2 * S, drain_t0 = 0;
  Result r;
  for (int64_t t = 10 * MS; t <= sc.dur; t += 10 * MS) {
    bus.run_until(t);
    // scenario events
    if (with_bridge && t == br_boot) br.boot();
    if (with_bridge && sc.kind == 3 && t == 60 * S) br.power_off(), pump.reset();
    if (with_bridge && sc.kind == 3 && t == 61500 * MS) br.boot();
    if (sc.kind == 4 && t == 60 * S) pgd.power_off();
    if (sc.kind == 4 && t == 65 * S) pgd.power_on(t, true);
    if (sc.kind == 5 && t == 90 * S) f.spec.ctrl_silent_at = -1;
    if (sc.kind == 8 && t == 90 * S) f.spec.p_to_ctrl = 0.01, f.spec.p_walk_stall = 0;
    if (with_bridge && sc.kind == 7 && (t == 40 * S || t == 120 * S)) {  // set_enroll(false): drain
      br.term->drain_replied_ = false;
      br.term->claim_mask_ = 0;
      br.term->drain_ = true;
      drain_t0 = t;
    }
    if (with_bridge && sc.kind == 7 && (t == 80 * S || t == 160 * S)) {  // set_enroll(true)
      br.term->drain_ = false;
      br.term->drain_replied_ = false;
      br.term->claim_mask_ = OWN_BIT;
      br.term->enroll_ = true;
    }
    if (with_bridge && br.powered && br.term->drain_) {
      if (br.term->drain_replied_ || t - drain_t0 > 15 * S) br.term->enroll_ = false, br.term->drain_ = false;
    }
    // ~0.5 Hz value drift on the anchor (one changed character most times)
    if (t >= next_val) {
      next_val += 2 * S;
      char b[32];
      value++;
      snprintf(b, sizeof b, "    Hotwater:   %4.1f C", 30.0 + 0.1 * value);
      ctl.set_row(0, 2, b, t);
    }
    // controller ground truth (current page content)
    const auto &rows = ctl.screen.pages[ctl.screen.cur].rows;
    if (hist.empty() || hist.back().rows != rows) hist.push_back({t, rows});
    if (with_bridge) {
      auto sv = ctl.served();
      if (std::find(sv.begin(), sv.end(), ENROLL_ADDR) != sv.end()) served31.push_back(t);
      // macros
      if (t == 40 * S && sc.mk != MK_NONE && sc.mk != MK_KEYS)
        mac.start(sc.mk == MK_EDIT ? &EDIT : sc.mk == MK_ALARM ? &ALARM : &WALK, t);
      if (sc.mk == MK_KEYS && t >= 30 * S && t % (1500 * MS) == 0 && pump.idle())
        pump.request((t / (1500 * MS)) % 2 ? KEY_DOWN : KEY_UP, t);
      mac.tick(t, pump, br);
      // I5: publish instant = settle after display activity
      if (br.disp_dirty && t - br.last_disp_t >= 600 * MS) {
        br.disp_dirty = false;
        if (t < sc.eval_t0) continue;
        for (int row = 0; row < 8; row++) {
          std::string got = br.scr.row(SCR_TERM_ESP, row);
          if (got.empty()) continue;
          bool seen = false;
          for (size_t i = hist.size(); i-- > 0;) {
            int64_t until = i + 1 < hist.size() ? hist[i + 1].t : t;
            if (until < t - 6 * S) break;
            if (hist[i].rows[row] == got) seen = true;
          }
          if (!seen) {
            if (!r.i5_bad) {
              char m[160];
              snprintf(m, sizeof m, "t=%.3fs row %d [%s] never shown in the last 6 s (now [%s])",
                       t / 1e6, row, got.c_str(), rows[row].c_str());
              r.i5_msg = m;
            }
            r.i5_bad++;
          }
        }
      }
    }
  }

  // Debugging aid: D12_DUMP=t0,t1 (seconds) prints the wire log of the run
  // with the bridge, plus the controller's settled masks.
  if (const char *d = getenv("D12_DUMP"))
    if (with_bridge) {
      double d0 = atof(d), d1 = strchr(d, ',') ? atof(strchr(d, ',') + 1) : d0 + 1;
      bus.dump(static_cast<int64_t>(d0 * S), static_cast<int64_t>(d1 * S));
      printf("map %08X claims %08X focus %02X sessioned32 %d sessioned31 %d walks %u\n", ctl.map, ctl.claims,
             ctl.focus, ctl.sessioned[32], ctl.sessioned[31], ctl.ff_walks);
    }
  // ---- verdict inputs from the wire log ------------------------------------
  std::vector<LogFrame> L = bus.log;
  std::stable_sort(L.begin(), L.end(), [](const LogFrame &a, const LogFrame &b) { return a.t < b.t; });
  std::vector<int64_t> walks, polls31;
  int64_t last_paint = sc.eval_t0;
  for (const auto &l : L) {
    if (l.from == 0x01 && is_ffwalk(l.f)) {
      walks.push_back(l.t);
      if (l.t >= sc.eval_t0) r.walks++;
    }
    if (l.from == 0x01 && l.f.size() == 4 && l.f[0].v == ENROLL_ADDR && l.f[1].v == 0x01) polls31.push_back(l.t);
    if (l.from == 0x01 && l.f.size() > 2 && l.f[0].v == 0x20 && l.t >= sc.eval_t0 &&
        (l.f[1].v == 0x0B || l.f[1].v == 0x0C || l.f[1].v == 0x64 || l.f[1].v == 0x65)) {
      r.dark_max = std::max(r.dark_max, l.t - last_paint);
      last_paint = l.t;
    }
  }
  r.dark_max = std::max(r.dark_max, sc.dur - last_paint);
  if (!with_bridge) return r;

  // I3: passive TX (counted at the ISR) and reply-window / overlap checks.
  r.tx_passive = br.tx_passive;
  for (size_t i = 0; i < L.size(); i++) {
    if (L[i].from != ENROLL_ADDR) continue;
    const LogFrame *prev = nullptr;  // the latest other frame that started before ours
    bool overlap = false;
    for (size_t j = i; j-- > 0 && L[j].t + 300 * MS > L[i].t;) {
      if (L[j].from == ENROLL_ADDR || L[j].from == 0) continue;
      if (L[j].end() > L[i].t) overlap = true;
      if (!prev) prev = &L[j];
    }
    for (size_t j = i + 1; j < L.size() && L[j].t < L[i].end(); j++)
      if (L[j].from != ENROLL_ADDR && L[j].from != 0) overlap = true;
    bool in_window = prev && prev->f[0].v == ENROLL_ADDR && L[i].t - prev->end() <= 2 * MS &&
                     L[i].t >= prev->end();
    if (!in_window || overlap) {
      if (!r.tx_outside) {
        char m[200];
        snprintf(m, sizeof m, "t=%.3fs TX [%s] after [%s] (%s)", L[i].t / 1e6, hex(L[i].f).c_str(),
                 prev ? hex(prev->f).c_str() : "-", overlap ? "overlaps another frame" : "no reply window");
        r.i3_msg = m;
      }
      r.tx_outside++;
    }
  }
  if (r.tx_passive && r.i3_msg.empty()) r.i3_msg = "transmitted while passive";

  // I4: first served sample after calm_t.
  if (sc.calm_t >= 0)
    for (int64_t ts : served31)
      if (ts >= sc.calm_t) {
        r.recover = ts - sc.calm_t;
        break;
      }

  // I6: per key request.
  for (size_t q = 0; q < pump.reqs.size(); q++) {
    const KeyPump::Req &rq = pump.reqs[q];
    if (rq.t_start < 0) continue;
    int64_t t1 = q + 1 < pump.reqs.size() && pump.reqs[q + 1].t_start >= 0 ? pump.reqs[q + 1].t_start : sc.dur;
    int copies = 0;
    for (const auto &l : L) {
      if (l.from != ENROLL_ADDR || l.t < rq.t_start || l.t >= t1 || l.f.size() < 2 || l.f[1].v != 0x1E) continue;
      copies++;
      // (b) the poll this key answers vs the last walk start before it
      int64_t w = -1;
      for (int64_t x : walks)
        if (x < l.t) w = x;
      int polls_since = 0;
      for (int64_t p : polls31)
        if (p > w && p < l.t) polls_since++;
      if (w >= 0 && polls_since <= 1) {
        if (!r.i6_first && r.i6_msg.empty()) {
          char m[120];
          snprintf(m, sizeof m, "key 0x%02X at t=%.3fs in the first poll after the walk at %.3fs", rq.key,
                   l.t / 1e6, w / 1e6);
          r.i6_msg = m;
        }
        r.i6_first++;
      }
    }
    size_t exec_after = rq.t_done >= 0 ? (q + 1 < pump.reqs.size() && pump.reqs[q + 1].t_start >= 0
                                              ? pump.reqs[q + 1].keys_before
                                              : ctl.keys.size())
                                       : rq.keys_before;
    size_t execs = exec_after - std::min(exec_after, rq.keys_before);
    if (copies > 1 || execs > 1) {
      if (!r.i6_multi && (r.i6_msg.empty() || r.i6_first)) {
        char m[120];
        snprintf(m, sizeof m, "key 0x%02X (request %zu) sent %d times, executed %zu times", rq.key, q,
                 copies, execs);
        r.i6_msg = m;
      }
      r.i6_multi++;
    }
  }
  r.mac_done = mac.st == Macro::DONE;
  r.mac_ok = mac.ok;
  if (sc.mk == MK_WALK && mac.ok) r.walk_us = mac.t_end - mac.t_begin;
  bool truth = sc.mk == MK_EDIT ? committed == 46.0 : sc.mk == MK_ALARM ? !alarm : true;
  if (mac.ok && !truth) {
    r.i6_false_ok++;
    char m[120];
    snprintf(m, sizeof m, "macro reported success, controller holds %s",
             sc.mk == MK_EDIT ? (committed == 45.0 ? "45.0 (commit lost)" : "a different value")
                              : "the alarm still active");
    r.i6_msg = m;
  }
  return r;
}

// --- verdicts ------------------------------------------------------------------
struct Verdict {
  std::string inv, scenario;
  uint64_t seed;
  bool pass;
  std::string msg;
  int64_t value = -1;  // I7: measured duration
  std::string rule = "";  // root-cause tag of a failure (the rework row)
};

static std::vector<Verdict> evaluate(const Scenario &sc, uint64_t seed) {
  Result b = run(sc, seed, true), a = run(sc, seed, false);
  std::vector<Verdict> v;
  char m[300];
  int64_t bound = std::max(a.dark_max + 12 * S, 15 * S);
  snprintf(m, sizeof m, "pGD unpainted for %.1f s (bridge absent: %.1f s, bound %.1f s)", b.dark_max / 1e6,
           a.dark_max / 1e6, bound / 1e6);
  v.push_back({"I1", sc.name, seed, b.dark_max <= bound, m, -1,
               sc.kind == 1 ? "R-RC-30/V3+R-RC-32" : "D3/R-LL-17+R-RC-30/V3+R-RC-32"});
  int slack = a.walks / 5;
  snprintf(m, sizeof m, "%d FF-walks with the bridge vs %d without (slack %d)", b.walks, a.walks, slack);
  v.push_back({"I2", sc.name, seed, b.walks <= a.walks + slack, m, -1,
               sc.kind == 3   ? "A7-DE-floating+D3/R-LL-17"
               : sc.p >= 0.15 ? "A8-R7/R-RC-30(no-backoff)"
                              : "D3/R-LL-17"});
  snprintf(m, sizeof m, "%u TX while passive, %u outside a reply window: %s", b.tx_passive, b.tx_outside,
           b.i3_msg.c_str());
  v.push_back({"I3", sc.name, seed, b.tx_passive == 0 && b.tx_outside == 0, m, -1, "R-LL-03"});
  if (sc.calm_t >= 0) {
    snprintf(m, sizeof m, "bridge served %.1f s after the bus turned healthy (bound 30 s)",
             b.recover < 0 ? -1.0 : b.recover / 1e6);
    v.push_back({"I4", sc.name, seed, b.recover >= 0 && b.recover <= 30 * S, m, -1, "R-RC-03"});
  }
  snprintf(m, sizeof m, "%d wrong rows at settle instants: %s", b.i5_bad, b.i5_msg.c_str());
  v.push_back({"I5", sc.name, seed, b.i5_bad == 0, m, -1, "W-01/R-DI-15"});
  if (sc.mk != MK_NONE) {
    snprintf(m, sizeof m, "%d multi-sent/executed, %d first-poll keys, %d false successes (macro %s): %s",
             b.i6_multi, b.i6_first, b.i6_false_ok,
             !b.mac_done ? "-" : b.mac_ok ? "ok" : "failed", b.i6_msg.c_str());
    std::string tag;
    if (b.i6_multi) tag += "F-06/F-07";
    if (b.i6_first) tag += std::string(tag.empty() ? "" : "+") + "F-03";
    if (b.i6_false_ok) tag += std::string(tag.empty() ? "" : "+") + "F-02/F-13";
    v.push_back({"I6", sc.name, seed, b.i6_multi == 0 && b.i6_first == 0 && b.i6_false_ok == 0, m, -1, tag});
  }
  if (sc.mk == MK_WALK) {
    snprintf(m, sizeof m, "scrape walk %s in %.3f s", b.walk_us >= 0 ? "done" : "FAILED", b.walk_us / 1e6);
    Verdict w{"I7", sc.name, seed, b.walk_us >= 0, m};
    w.value = b.walk_us;
    v.push_back(w);
  }
  return v;
}

// --- expected table ---------------------------------------------------------------
struct Expect {
  std::string status, rule;
};
static std::map<std::string, Expect> load_expected(const char *path) {
  std::map<std::string, Expect> e;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string inv, scn, seed, status, rule;
    std::getline(ss, inv, '\t');
    std::getline(ss, scn, '\t');
    std::getline(ss, seed, '\t');
    std::getline(ss, status, '\t');
    std::getline(ss, rule, '\t');
    e[inv + "/" + scn + "/" + seed] = {status, rule};
  }
  return e;
}

static std::string ascii(std::string s) {
  for (char &c : s)
    if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) > 0x7E) c = '?';
  return s;
}

int main(int argc, char **argv) {
  bool emit = argc > 1 && !strcmp(argv[1], "--emit");
  bool matrix = argc > 1 && !strcmp(argv[1], "--matrix");
  bool detail = argc > 3 && !strcmp(argv[1], "--detail");
  std::vector<Verdict> all;
  for (const auto &sc : scenarios()) {
    if (detail && sc.name != std::string(argv[2])) continue;
    for (uint64_t seed : SEEDS) {
      if (detail && seed != strtoull(argv[3], nullptr, 0)) continue;
      auto v = evaluate(sc, seed);
      all.insert(all.end(), v.begin(), v.end());
    }
  }
  if (detail) {
    for (const auto &v : all) printf("%s %s seed %llu: %s  %s\n", v.inv.c_str(), v.scenario.c_str(),
                                     static_cast<unsigned long long>(v.seed), v.pass ? "pass" : "FAIL", v.msg.c_str());
    return 0;
  }
  if (emit) {
    printf("# invariant\tscenario\tseed\tstatus\trule\tmessage (generated by test_sim_invariants --emit)\n");
    for (const auto &v : all) {
      if (v.inv == "I7" && v.pass)
        printf("%s\t%s\t%llu\tlimit=%lld\towner-walk-never-slower\ttoday's walk duration in us\n", v.inv.c_str(),
               v.scenario.c_str(), static_cast<unsigned long long>(v.seed), static_cast<long long>(v.value));
      else if (!v.pass)
        printf("%s\t%s\t%llu\txfail\t%s\t%s\n", v.inv.c_str(), v.scenario.c_str(),
               static_cast<unsigned long long>(v.seed), v.rule.c_str(), ascii(v.msg).c_str());
    }
    return 0;
  }
  auto exp = load_expected("test/sim/invariants_expected.tsv");
  if (exp.empty()) {
    fprintf(stderr, "test_sim_invariants: test/sim/invariants_expected.tsv missing (run from the repo root)\n");
    return 1;
  }
  int bad = 0, xfail = 0, pass = 0;
  std::map<std::string, std::map<std::string, std::string>> mx;
  std::vector<std::string> order;
  for (const auto &v : all) {
    std::string key = v.inv + "/" + v.scenario + "/" + std::to_string(v.seed);
    auto it = exp.find(key);
    bool expected_fail = it != exp.end() && it->second.status == "xfail";
    bool ok = v.pass;
    if (v.inv == "I7" && v.pass) {
      long long lim = it != exp.end() && it->second.status.rfind("limit=", 0) == 0
                          ? atoll(it->second.status.c_str() + 6)
                          : -1;
      ok = lim >= 0 && v.value <= lim;
      if (!ok)
        fprintf(stderr, "I7 %s seed %llu: scrape walk %.3f s is slower than today's %.3f s (owner's hard rule)\n",
                v.scenario.c_str(), static_cast<unsigned long long>(v.seed), v.value / 1e6, lim / 1e6);
    }
    if (ok && expected_fail) {
      fprintf(stderr, "%s %s seed %llu: expected failure now PASSES -- flip the row in invariants_expected.tsv (%s)\n",
              v.inv.c_str(), v.scenario.c_str(), static_cast<unsigned long long>(v.seed), it->second.rule.c_str());
      bad++;
    } else if (!ok && !expected_fail) {
      if (v.inv != "I7" || !v.pass)
        fprintf(stderr, "%s %s seed %llu: FAILED: %s\n", v.inv.c_str(), v.scenario.c_str(),
                static_cast<unsigned long long>(v.seed), v.msg.c_str());
      bad++;
    } else if (!ok) {
      xfail++;
    } else {
      pass++;
    }
    if (!mx.count(v.scenario)) order.push_back(v.scenario);
    std::string &cell = mx[v.scenario][v.inv];
    cell += ok ? "P" : "x";
  }
  for (const auto &e : exp) {
    bool seen = false;
    for (const auto &v : all)
      if (e.first == v.inv + "/" + v.scenario + "/" + std::to_string(v.seed)) seen = true;
    if (!seen) fprintf(stderr, "invariants_expected.tsv: row %s matches no verdict\n", e.first.c_str()), bad++;
  }
  if (matrix) {
    const char *inv[] = {"I1", "I2", "I3", "I4", "I5", "I6", "I7"};
    printf("| scenario | I1 | I2 | I3 | I4 | I5 | I6 | I7 |\n|---|---|---|---|---|---|---|---|\n");
    for (const auto &s : order) {
      printf("| %s |", s.c_str());
      for (const char *i : inv) {
        auto it = mx[s].find(i);
        if (it == mx[s].end()) {
          printf(" n/a |");
          continue;
        }
        int x = static_cast<int>(std::count(it->second.begin(), it->second.end(), 'x'));
        int n = static_cast<int>(it->second.size());
        if (x == 0) printf(" pass |");
        else if (x == n) printf(" **xfail** |");
        else printf(" xfail %d/%d |", x, n);
      }
      printf("\n");
    }
  }
  printf("test_sim_invariants: %zu verdicts, %d pass, %d expected failures, %d unexpected\n", all.size(), pass,
         xfail, bad);
  return bad ? 1 : 0;
}
