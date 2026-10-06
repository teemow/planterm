// Host tests for the wave E5 bus-health telemetry (plan_terminal.h:
// BusWindow / take_window / bus_health / format_bus_window and the ISR feeds
// uart_errs / tx_hold): crafted byte streams for every counter, then the
// simulator's physical-fault profiles (C11 sim_faults.h) with the bridge
// PASSIVE, where tonight's fault has to be visible:
//   p = 0      R2 PGD_ONLY_HEALTHY      -> health 0 in every window
//   p ~ 5 %    10-02 passive            -> resend-after-ack ~5 %, no loop
//   p ~ 28 %   R8 PASSIVE_DEGRADED (T7) -> health 2 (loop), ra ~28 %
// The device counters are checked against the same measures computed from
// the simulator's ideal wire log (test_sim_pgd.cpp's A8 definitions).

#include "../src/plan_terminal.h"
#include "sim/sim_bus.h"
#include "sim/sim_controller.h"
#include "sim/sim_faults.h"
#include "sim/sim_pgd.h"
#include "sim/sim_terminals.h"

#include <cstdio>
#include <cstring>
#include <vector>

using namespace plan;
using plan::sim::CHAR_US;

static int fails = 0;
#define CHECK(c)                                                            \
  do {                                                                      \
    if (!(c)) {                                                             \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
      fails++;                                                              \
    }                                                                       \
  } while (0)
static const int64_t S = 1000000, MS = 1000;

// --- crafted byte streams ---------------------------------------------------

// Feeds whole frames (9th bit on the first byte) one char time per byte.
struct Feed {
  PlanTerminal term;
  int64_t t = 1 * S;
  void frame(std::vector<uint8_t> f, int64_t gap_us = 300) {
    t += gap_us;
    for (size_t i = 0; i < f.size(); i++) {
      term.on_byte(f[i], i == 0 ? 1 : 0, t);
      t += CHAR_US;
    }
  }
  // The address byte that closes the previous run (a lone controller 01').
  void close() { frame({0x01}); }
  BusWindow take() { return term.take_window(t); }
};

static std::vector<uint8_t> ck(std::vector<uint8_t> f) {
  uint8_t s = 0;
  for (uint8_t b : f) s += b;
  f.push_back(static_cast<uint8_t>(0xFF - s));
  return f;
}
// A controller session frame: TO' TYPE LEN 01 data.. CK (src = byte 3).
static std::vector<uint8_t> sess(uint8_t to, uint8_t type, uint8_t v) {
  return ck({to, type, 0x07, 0x01, 0x05, v});
}
static const std::vector<uint8_t> POLL20 = ck({0x20, 0x01, 0x01});
static const std::vector<uint8_t> ACK20 = ck({0x01, 0x03, 0x20});
static std::vector<uint8_t> walk(uint8_t c1) { return ck({0x02, 0x02, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, c1, 0, 0, 0}); }
static std::vector<uint8_t> rc(uint8_t to, uint8_t from) { return ck({to, 0x02, from, 0xC0, 0, 0, 1, 0xC0, 0, 0, 0}); }

// The pGD acks, the controller sends the same frame again: one resend after
// the ack. The copy is acked too and NOT resent; a repeat without an ack in
// between and a different frame after the ack are not counted.
static void test_resend_after_ack_pgd() {
  Feed f;
  f.frame(sess(0x20, 0x0C, 0x41));
  f.frame(ACK20);
  f.frame(sess(0x20, 0x0C, 0x41), 14 * MS);  // controller did not hear the ack
  f.frame(ACK20);
  f.frame(sess(0x20, 0x0C, 0x42), 14 * MS);  // next row: different frame
  f.frame(sess(0x20, 0x0C, 0x42), 14 * MS);  // pGD never acked: a re-send, not "after ack"
  f.frame(ACK20);
  f.frame(sess(0x20, 0x64, 0x43));           // graphic family (0x60-0x6F) counts too
  f.frame(ACK20);
  f.frame(sess(0x20, 0x64, 0x43), 14 * MS);
  f.frame(POLL20);
  f.close();
  BusWindow w = f.take();
  CHECK(w.ack20 == 4);  // 41, its copy, the 2nd 42, 43
  CHECK(w.ra20 == 2);
  CHECK(w.ack1f == 0 && w.ra1f == 0);
  // per window: the next window starts from zero
  f.close();
  w = f.take();
  CHECK(w.ack20 == 0 && w.ra20 == 0);
}

// Our own session ack (invisible on the wire, known from tx_sent) followed by
// the identical frame = the controller missed OUR ack.
static void test_resend_after_ack_us() {
  Feed f;
  TxAction ack;
  ack.kind = TxAction::SESSION_ACK;
  ack.len = 4;
  ack.bit9_mask = 1;
  const uint8_t a[4] = {0x01, 0x03, ENROLL_ADDR, static_cast<uint8_t>(0xFF - 0x01 - 0x03 - ENROLL_ADDR)};
  memcpy(ack.frame, a, 4);
  f.frame(sess(ENROLL_ADDR, 0x0C, 0x41));
  f.term.tx_sent(ack, f.t);
  f.frame(sess(ENROLL_ADDR, 0x0C, 0x41), 14 * MS);
  f.term.tx_sent(ack, f.t);
  f.frame(sess(ENROLL_ADDR, 0x0C, 0x42), 14 * MS);  // not acked by us (passive): not watched
  f.frame(sess(ENROLL_ADDR, 0x0C, 0x42), 14 * MS);
  f.close();
  BusWindow w = f.take();
  CHECK(w.ack1f == 2);
  CHECK(w.ra1f == 1);
  CHECK(w.ack20 == 0 && w.ra20 == 0);
}

// XX' 02 20 = the pGD walking the ring itself; its answer to the controller
// (01' 02 20) and the controller's own probes (XX' 02 01) are not.
static void test_pgd_rollcalls() {
  Feed f;
  for (uint8_t a = 0x02; a < 0x07; a++) f.frame(rc(a, 0x20), 15 * MS);
  f.frame(rc(0x01, 0x20));
  f.frame(rc(0x05, 0x01));
  f.close();
  BusWindow w = f.take();
  CHECK(w.pgd_rc == 5);
  CHECK(w.health == BUS_DEGRADED);  // any pGD roll-call
}

// Controller silences: gaps between the starts of its frames, the open one
// at the window's end included; a window without any controller frame = loop.
static void test_controller_silence() {
  Feed f;
  for (int i = 0; i < 20; i++) f.frame(POLL20, 24 * MS);
  BusWindow w = f.take();
  CHECK(w.ctrl_frames == 19);  // the 20th poll's run is still open
  CHECK(w.ctrl_sil == 0 && w.ctrl_gap_max_ms < 30);
  CHECK(w.health == BUS_HEALTHY);
  f.frame(POLL20, 1500 * MS);  // closes the last poll's run
  for (int i = 0; i < 5; i++) f.frame(POLL20, 24 * MS);
  w = f.take();
  CHECK(w.ctrl_sil == 1);
  CHECK(w.ctrl_gap_max_ms >= 1500 && w.ctrl_gap_max_ms <= 1530);
  CHECK(w.health == BUS_DEGRADED);
  f.t += 3 * S;  // the controller stops; only the pGD's ack arrives
  f.frame(ACK20);
  w = f.take();
  CHECK(w.ctrl_frames == 1);  // the last poll, closed by the ack
  CHECK(w.ctrl_sil == 0 && w.ctrl_gap_max_ms >= 3000);  // still open: counted in max only
  f.frame(ACK20, 2 * S);
  w = f.take();
  CHECK(w.ctrl_frames == 0);
  CHECK(w.health == BUS_LOOP);  // a controller-silent window
}

// FF-walks: cold (claims 00) vs warm (claims carried, 80..).
static void test_walks_cold_warm() {
  Feed f;
  f.frame(walk(0x00));
  f.frame(walk(0x80), 4 * S);  // > 3 s apart: not folded
  f.close();
  BusWindow w = f.take();
  CHECK(w.walks == 2 && w.walks_warm == 1);
  CHECK(f.term.tel_walks_ == 0);  // take_window owns the walks reset now
}

// A 9th bit on a byte above 0x20 is a parity-flipped data byte.
static void test_addr_garble() {
  Feed f;
  f.frame({0x41, 0x42});
  f.frame({0xFE});
  f.frame(POLL20);
  f.frame({0x00});
  f.close();
  BusWindow w = f.take();
  CHECK(w.addr_bad == 2);
  CHECK(f.take().addr_bad == 0);
}

// The ISR feeds: UART receive-error bits and the per-frame DE hold.
static void test_isr_feeds() {
  Feed f;
  f.term.uart_errs(true, false, false);
  f.term.uart_errs(true, true, false);
  f.term.uart_errs(false, false, true);
  f.term.tx_hold(12, 2320, 192, 0);           // A7 1.4: exactly 12 x 192 + 16
  f.term.tx_hold(4, 784 + 10, 192, 0);        // +10 us
  f.term.tx_hold(12, 2320 + 64 - 900, 192, 64);  // de_tail 64, cut 900 us short
  f.term.tel_tx_early_idle_ = 3;              // tx_9bit's counter
  BusWindow w = f.take();
  CHECK(w.uart_frm == 2 && w.uart_brk == 1 && w.uart_glitch == 1);
  CHECK(w.tx == 3);
  CHECK(w.hold_dev_min_us == -900);
  CHECK(w.hold_dev_max_us == 10);
  CHECK(w.tx_early_idle == 3);
  w = f.take();
  CHECK(w.uart_frm == 0 && w.uart_brk == 0 && w.uart_glitch == 0 && w.tx_early_idle == 0);
  CHECK(w.tx == 0 && w.hold_dev_min_us == 0 && w.hold_dev_max_us == 0);
}

// The `ekobeescope health` rules, one by one.
static void test_health_rules() {
  BusWindow ok{};
  ok.ctrl_frames = 800;
  ok.ctrl_gap_max_ms = 30;
  BusSpan s0{};
  CHECK(bus_health(ok, s0) == BUS_HEALTHY);
  BusWindow w = ok;
  w.ctrl_frames = 0;
  CHECK(bus_health(w, s0) == BUS_LOOP);
  CHECK(bus_health(ok, BusSpan{3, 0, 0, 0, 0}) == BUS_LOOP);
  CHECK(bus_health(ok, BusSpan{2, 0, 0, 0, 0}) == BUS_HEALTHY);
  w = ok, w.pgd_rc = 1;
  CHECK(bus_health(w, s0) == BUS_DEGRADED);
  w = ok, w.ctrl_gap_max_ms = 1000;
  CHECK(bus_health(w, s0) == BUS_DEGRADED);
  w = ok, w.paint_age_s = 60;
  CHECK(bus_health(w, s0) == BUS_DEGRADED);
  w = ok, w.joins = 3;
  CHECK(bus_health(w, s0) == BUS_DEGRADED);
  w.joins_ok = 1;
  CHECK(bus_health(w, s0) == BUS_HEALTHY);
  CHECK(bus_health(ok, BusSpan{0, 30, 4, 0, 0}) == BUS_HEALTHY);   // 13 %
  CHECK(bus_health(ok, BusSpan{0, 30, 5, 0, 0}) == BUS_DEGRADED);  // 17 %
  CHECK(bus_health(ok, BusSpan{0, 29, 29, 0, 0}) == BUS_HEALTHY);  // too few acked
  CHECK(bus_health(ok, BusSpan{0, 0, 0, 40, 7}) == BUS_DEGRADED);  // ours
}

// The rate rules run over the trailing 3 windows: one walk per 10 s reads
// LOOP from the third such window on, until fewer than 3 walks remain in
// the trailing 30 s.
static void test_health_span() {
  Feed f;
  std::vector<uint8_t> got;
  for (int win = 0; win < 7; win++) {
    for (int i = 0; i < 400; i++) f.frame(POLL20, 24 * MS);  // ~9.7 s of polls
    if (win < 5) f.frame(walk(0x00), 10 * MS);
    f.frame(sess(0x20, 0x0C, static_cast<uint8_t>(0x41 + win)), 10 * MS);  // a paint (< 60 s old)
    f.frame(ACK20);
    f.frame(POLL20, 10 * MS);
    got.push_back(f.take().health);
  }
  CHECK(got[0] == BUS_HEALTHY && got[1] == BUS_HEALTHY);
  CHECK(got[2] == BUS_LOOP && got[3] == BUS_LOOP && got[4] == BUS_LOOP);
  CHECK(got[5] == BUS_HEALTHY && got[6] == BUS_HEALTHY);
}

// format_bus_window: health first; the busiest plausible window fits the
// diag buffer behind the pre-E5 bus10s line (worst ~263 chars, #52), and any
// window whose counters stay below what 10 s of 62500 baud can carry (< 36k
// bytes: 5 digits; gaps/ages up to days) fits plan_bridge's 288-byte block.
static void test_format() {
  BusWindow w{};
  w.health = 2, w.ra20 = 50, w.ack20 = 180, w.ra1f = 40, w.ack1f = 150, w.pgd_rc = 74;
  w.ctrl_frames = 900, w.ctrl_sil = 3, w.ctrl_gap_max_ms = 10000, w.paint_age_s = 3600;
  w.walks_warm = 4, w.uart_frm = 666, w.uart_brk = 666, w.uart_glitch = 12, w.addr_bad = 12;
  w.tx = 600, w.hold_dev_min_us = -2000, w.hold_dev_max_us = 120, w.tx_early_idle = 3;
  char b[512];
  int n = format_bus_window(b, sizeof b, w);
  CHECK(strncmp(b, " health=2 ra20=50 ack20=180", 27) == 0);
  CHECK(n <= 512 - 263 - 1);
  BusWindow m{};
  m.health = 2, m.ra20 = m.ack20 = m.ra1f = m.ack1f = m.pgd_rc = m.ctrl_frames = 99999;
  m.ctrl_sil = m.walks_warm = m.uart_frm = m.uart_brk = m.uart_glitch = 99999;
  m.addr_bad = m.tx = m.tx_early_idle = 99999;
  m.ctrl_gap_max_ms = 999999999, m.paint_age_s = 9999999;  // 11 days
  m.hold_dev_min_us = m.hold_dev_max_us = -99999;
  n = format_bus_window(b, sizeof b, m);
  CHECK(n < 288);
}

// --- simulator fault profiles (bridge passive) ------------------------------

using namespace plan::sim;

static bool is_session(uint8_t ty) { return (ty >= 0x0A && ty <= 0x0F) || (ty >= 0x60 && ty <= 0x6F); }

struct Profile {
  std::vector<BusWindow> win;
  int log_acked = 0, log_ra = 0;
  uint32_t ack20 = 0, ra20 = 0, walks = 0, pgd_rc = 0, loops = 0, healthy = 0;
  double ra() const { return ack20 ? static_cast<double>(ra20) / ack20 : 0; }
  double log_ra_pct() const { return log_acked ? static_cast<double>(log_ra) / log_acked : 0; }
};

static Profile run_profile(double p, double stall, uint64_t seed, int windows) {
  Bus bus;
  SimController ctl;
  PgdModel pgd;
  PlanTerminalStation br;  // enroll_ = false: passive, never transmits
  bus.attach(&ctl);
  bus.attach(&pgd);
  bus.attach(&br);
  FaultSpec fs;
  fs.seed = seed;
  fs.p_to_ctrl = p;
  fs.p_walk_stall = stall;
  PhysFault f(fs);
  f.attach(bus);
  pgd.seed(seed);
  ctl.start(0);
  Profile r;
  const int64_t t0 = 20 * S;  // past the boot walk and the pGD's join
  // ~8 display frames/s like the live controller (test_sim_pgd World).
  for (int64_t t = 125 * MS; t <= t0 + windows * 10 * S; t += 125 * MS) {
    bus.run_until(t);
    char v[24];
    snprintf(v, sizeof v, "    Hotwater:   35.%d\xDF" "C", static_cast<int>((t / (125 * MS)) % 10));
    ctl.set_row(0, 2, v, t);
    if (t >= t0 && (t - t0) % (10 * S) == 0) {
      BusWindow w = br.term.take_window(t);
      br.term.tel_walks_warm_ = 0;  // rollcall10s owns these resets
      br.term.tel_joins_ = br.term.tel_joins_ok_ = 0;
      if (t == t0) continue;  // the boot window
      r.win.push_back(w);
      r.ack20 += w.ack20, r.ra20 += w.ra20, r.walks += w.walks, r.pgd_rc += w.pgd_rc;
      r.loops += w.health == BUS_LOOP, r.healthy += w.health == BUS_HEALTHY;
    }
  }
  CHECK(br.tx_count == 0);
  // The same measure from the ideal wire log (A8 "ra").
  const auto &L = bus.log;
  for (size_t i = 0; i < L.size(); i++) {
    const LogFrame &l = L[i];
    if (l.t < t0 || l.from != 0x01 || l.f.size() < 5 || l.f[0].v != 0x20 || !is_session(l.f[1].v)) continue;
    if (i + 2 >= L.size() || L[i + 1].from != 0x20 || L[i + 1].f.size() != 4 || L[i + 1].f[1].v != 0x03)
      continue;
    r.log_acked++;
    if (hex(L[i + 2].f) == hex(l.f)) r.log_ra++;
  }
  printf("  p=%.2f: %zu windows, health 0/2 = %u/%u, ra %.1f %% (%u/%u; wire log %.1f %%), walks %u, pgd_rc %u\n",
         p, r.win.size(), r.healthy, r.loops, 100 * r.ra(), r.ra20, r.ack20, 100 * r.log_ra_pct(), r.walks,
         r.pgd_rc);
  return r;
}

static void test_sim_profiles() {
  const int N = 18;  // 180 s
  Profile h = run_profile(0, 0, 1, N);
  CHECK(h.healthy == static_cast<uint32_t>(N));
  CHECK(h.ra20 == 0 && h.walks == 0 && h.pgd_rc == 0);
  CHECK(h.ack20 >= 30u * N);  // the pGD is painted and acks: the rate rule is live
  for (const BusWindow &w : h.win) {
    CHECK(w.ctrl_frames > 0 && w.ctrl_gap_max_ms < 100 && w.paint_age_s < 2);
    CHECK(w.addr_bad == 0 && w.ra1f == 0 && w.ack1f == 0);
  }

  Profile d = run_profile(0.05, 0, 1, N);
  CHECK(d.ra() >= 0.02 && d.ra() <= 0.08);  // 10-02 passive: 2-5 % (A8), sim 3.9-5.4 %
  CHECK(d.ra20 >= static_cast<uint32_t>(d.log_ra * 0.9) && d.ra20 <= static_cast<uint32_t>(d.log_ra * 1.1) + 1);
  CHECK(d.loops <= 2);

  Profile l = run_profile(0.28, 1.0, 1, N);
  CHECK(l.ra() >= 0.20 && l.ra() <= 0.36);  // tonight 27-30 %
  CHECK(l.loops * 10 >= static_cast<uint32_t>(N) * 8);  // LOOP in >= 80 % of the windows
  CHECK(l.healthy == 0);
  CHECK(l.pgd_rc > 0);
}

int main() {
  test_resend_after_ack_pgd();
  test_resend_after_ack_us();
  test_pgd_rollcalls();
  test_controller_silence();
  test_walks_cold_warm();
  test_addr_garble();
  test_isr_feeds();
  test_health_rules();
  test_health_span();
  test_format();
  test_sim_profiles();
  if (fails) {
    fprintf(stderr, "test_bus_health: %d check(s) failed\n", fails);
    return 1;
  }
  printf("test_bus_health: all checks passed\n");
  return 0;
}
