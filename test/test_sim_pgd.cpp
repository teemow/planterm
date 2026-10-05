// Host tests for the pGD model (sim_pgd.h) and the physical-fault model
// (sim_faults.h), wave row C11: the A6 state machine on the simulated bus,
// seeded fault injection, and calibration against the measured regimes of
// A8 (R2 PGD_ONLY_HEALTHY, 10-02 passive, R8 PASSIVE_DEGRADED, R9) with the
// classifier thresholds of A8 sect. 4 / `ekobeescope health`.

#include "sim/sim_bus.h"
#include "sim/sim_controller.h"
#include "sim/sim_faults.h"
#include "sim/sim_pgd.h"

#include <cstdio>
#include <cstdlib>
#include <string>

using namespace plan::sim;

static int fails = 0;
#define CHECK(c)                                                            \
  do {                                                                      \
    if (!(c)) {                                                             \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
      fails++;                                                              \
    }                                                                       \
  } while (0)
#define CHECK_HEX(f, s)                                                                     \
  do {                                                                                      \
    std::string h_ = hex(f);                                                                \
    if (h_ != (s)) {                                                                        \
      fprintf(stderr, "%s:%d: got [%s] want [%s]\n", __FILE__, __LINE__, h_.c_str(), s); \
      fails++;                                                                              \
    }                                                                                       \
  } while (0)
static const int64_t S = 1000000, MS = 1000;
static bool near(int64_t v, int64_t want, int64_t tol) { return llabs(v - want) <= tol; }

// A scripted station that puts given frames on the wire (member forwards).
struct Script : Station {
  explicit Script(uint8_t a) : Station(a) {}
};

struct World {
  Bus bus;
  SimController ctl;
  PgdModel pgd;
  explicit World(bool booted = true) : pgd(booted) {
    bus.attach(&ctl);
    bus.attach(&pgd);
  }
  // ~8 display frames/s like the live controller (R-LL-09): one changed
  // character (0x0C) every 125 ms on the anchor page.
  void run_with_values(int64_t t_end, int64_t step = 125 * MS) {
    for (int64_t t = bus.now + step; t <= t_end; t += step) {
      bus.run_until(t);
      char v[24];
      snprintf(v, sizeof v, "    Hotwater:   35.%d\xDF" "C", static_cast<int>((t / step) % 10));
      ctl.set_row(0, 2, v, t);
    }
    bus.run_until(t_end);
  }
};

static bool is_walk(const Frame &f) {  // R-LL-11: any claims
  return f.size() == 12 && f[0].v == 0x02 && f[1].v == 0x02 && f[2].v == 0x01 && f[3].v == 0xFF &&
         f[4].v == 0xFF && f[5].v == 0xFF && f[6].v == 0xFF;
}
static bool is_session(uint8_t ty) { return (ty >= 0x0B && ty <= 0x0F) || (ty >= 0x64 && ty <= 0x66); }

// The A8 / `ekobeescope health` measures over [t0, t1).
struct Measures {
  double secs = 0;
  int walks = 0, warm = 0, pgd_master_probes = 0, polls20 = 0, acked = 0, resent_after_ack = 0;
  double per10(int n) const { return n * 10.0 / secs; }
  double walks10() const { return per10(walks); }
  double pgdw10() const { return per10(pgd_master_probes) / 30.0; }  // 30 probes = 1 pGD walk
  double p20() const { return per10(polls20); }
  double ra() const { return acked ? static_cast<double>(resent_after_ack) / acked : 0; }
  // A8 sect. 4 step 6 (passive bridge) / `ekobeescope health`.
  bool healthy() const { return walks10() < 1.0 && pgdw10() < 0.5 && p20() >= 150 && ra() < 0.15; }
  bool degraded() const {
    return walks10() >= 1.0 || pgdw10() >= 0.5 || (ra() >= 0.15 && walks > 0);
  }
  bool loop() const { return walks >= 3 && walks10() >= 1.0; }  // health LOOP
  void print(const char *n) const {
    printf("  %-22s walks/10s %.2f (+%.2f warm)  ra %.1f%% (%d/%d)  pGD-master walks/10s %.2f  polls20/10s %.0f\n",
           n, walks10(), per10(warm), 100 * ra(), resent_after_ack, acked, pgdw10(), p20());
  }
};
static Measures measure(const Bus &b, int64_t t0, int64_t t1) {
  Measures m;
  m.secs = (t1 - t0) / 1e6;
  const auto &L = b.log;
  for (size_t i = 0; i < L.size(); i++) {
    const LogFrame &l = L[i];
    if (l.t < t0 || l.t >= t1 || l.f.size() < 2) continue;
    // A8 sect. 1: `walks` = cold FF-walk starts (claims 00 = planterm
    // tel_walks_ / bus10s walks=); the claims-80 warm walk after a pGD
    // handoff is ffw_pass2 (A2 R-LL-11: 57 of 120 tonight).
    if (l.from == 0x01 && is_walk(l.f)) (l.f[7].v ? m.warm : m.walks)++;
    if (l.from == 0x20 && l.f[1].v == 0x02 && l.f[0].v != 0x01) m.pgd_master_probes++;
    if (l.from == 0x01 && l.f[0].v == 0x20 && l.f[1].v == 0x01) m.polls20++;
    // ra: a session frame to 0x20, the pGD's `01' 03 20 DB`, then the
    // identical frame again from the controller.
    if (l.from == 0x01 && l.f[0].v == 0x20 && is_session(l.f[1].v)) {
      bool acked = false;
      for (size_t j = i + 1; j < L.size(); j++) {
        if (L[j].from == 0x20 && L[j].f.size() == 4 && L[j].f[1].v == 0x03) acked = true;
        if (L[j].from == 0x01) {
          if (acked) {
            m.acked++;
            if (hex(L[j].f) == hex(l.f)) m.resent_after_ack++;
          }
          break;
        }
      }
    }
  }
  return m;
}

// --- A6 state machine ---------------------------------------------------------

// T1/T2 boot silence, T3-T5 join (E E L), T10/T11 session, R-RC-11.
static void test_boot_and_join() {
  World w(false);
  w.pgd.power_on(0);
  w.ctl.start(5 * MS);
  w.bus.run_until(18 * S);
  CHECK(w.pgd.state(w.bus.now) == PgdModel::BOOTING);
  CHECK(w.bus.from(0x20).empty());                      // T2: silent while booting
  CHECK(w.bus.log[0].from == 0 && w.bus.log[0].f.size() == 1);  // T1 glitch byte
  w.bus.run_until(25 * S);
  int unanswered = 0;
  for (auto &l : w.bus.from(0x01, 0, 18 * S))
    if (l.f[0].v == 0x20 && l.f[1].v == 0x02) unanswered++;
  CHECK(unanswered >= 7);  // A6 BOOT: 16 ignored probes in 19 s (2.3 s idle walks)
  CHECK(w.ctl.joins == 1 && w.ctl.link_faults == 0);
  auto p = w.bus.from(0x20, 18 * S);
  CHECK(p.size() > 20);
  // T3/T4: two echoes, MAP verbatim, own claim added; T5: link reply.
  CHECK_HEX(p[0].f, "01' 02 20 80 00 00 01 80 00 00 00 DB");
  CHECK_HEX(p[1].f, "01' 02 20 80 00 00 01 80 00 00 00 DB");
  CHECK_HEX(p[2].f, "01' 01 20 DD");
  bool ident = false;
  for (auto &l : p) ident |= hex(l.f) == "01' 51 07 20 0A 17 65";  // T11
  CHECK(ident);
  CHECK(w.pgd.inits == 1 && w.pgd.sacks >= 15 && w.pgd.master_walks == 0);
  CHECK(w.pgd.state(w.bus.now) == PgdModel::SESSIONED);
  CHECK(w.pgd.screen[2].find("Hotwater") != std::string::npos);
}

// T7: poll -> link reply ~245 us after the poll's stop bit.
static void test_turnaround() {
  World w;
  w.ctl.start(0);
  w.bus.run_until(30 * S);
  auto c = w.bus.from(0x01, 1 * S);
  int n = 0, in = 0;
  double sum = 0;
  for (auto &l : c) {
    if (l.f[0].v != 0x20 || l.f[1].v != 0x01) continue;
    for (auto &r : w.bus.from(0x20, l.end(), l.end() + 2 * MS)) {
      int64_t idle = r.t - l.end();
      n++;
      sum += static_cast<double>(idle);
      in += idle >= 229 && idle <= 269;
      break;
    }
  }
  CHECK(n > 1000 && in >= n - n / 100);
  CHECK(near(static_cast<int64_t>(sum / n), 246, 4));
}

// T8/T9 reply retry, T15/T16 master walk, T17 controller takes the token at
// 01' and continues the walk with the pGD's claims (R-RC-07), re-adoption.
static void test_retry_master_readoption() {
  World w;
  w.ctl.start(0);
  w.bus.run_until(5 * S);
  FaultSpec fs;
  fs.p_to_ctrl = 1.0;  // the controller hears nothing the pGD sends
  PhysFault f(fs);
  f.attach(w.bus);
  // Run until the pGD's own walk has done one full round.
  auto walk_at = [&]() -> size_t {
    auto q = w.bus.from(0x20, 5 * S);
    for (size_t j = 0; j < q.size(); j++)
      if (hex(q[j].f) == "01' 02 20 FF FF FF FF 80 00 00 00 60") return j;
    return SIZE_MAX;
  };
  while (walk_at() == SIZE_MAX && w.bus.now < 8 * S) w.bus.run_until(w.bus.now + 10 * MS);
  w.bus.run_until(w.bus.now + 470 * MS);
  auto p = w.bus.from(0x20, 5 * S);
  size_t i = walk_at();
  CHECK(i >= 3 && i + 31 < p.size());
  if (i < 3 || i + 31 >= p.size()) return;
  // T8/T9: the reply to the controller's last poll, resent 14 ms and 15 ms
  // later, then the walk 16 ms after the 3rd (A2 T2, R-LL-05).
  for (size_t k = i - 3; k < i; k++) CHECK_HEX(p[k].f, "01' 01 20 DD");
  CHECK(near(p[i - 2].t - p[i - 3].t, 14 * MS, 100) && near(p[i - 1].t - p[i - 2].t, 15 * MS, 100));
  CHECK(near(p[i].t - p[i - 1].t, 16 * MS, 100));
  // T15/T16: 15 ms probe spacing, MAP shrinking, wrap after 1F'.
  CHECK_HEX(p[i + 1].f, "02' 02 20 FF FF FF FE 80 00 00 00 60");
  CHECK(near(p[i + 1].t - p[i].t, 15 * MS, 100));
  CHECK_HEX(p[i + 30].f, "1F' 02 20 C0 00 00 00 80 00 00 00 7E");
  CHECK_HEX(p[i + 31].f, "01' 02 20 FF FF FF FF 80 00 00 00 60");
  CHECK(near(p[i + 31].t - p[i].t, 462 * MS, 10 * MS));  // ~458 ms round
  // The controller hears again just before the 3rd round's 01' probe (it is
  // still inside its 2.00 s link-fault quiet) and takes the token there.
  w.bus.run_until(p[i].t + 2 * 462 * MS - 1 * MS);
  CHECK(w.pgd.state(w.bus.now) == PgdModel::MASTER);
  // The controller hears again: it takes the token at the next 01' probe.
  CHECK(w.ctl.takeovers == 0);
  w.bus.hook = nullptr;
  int64_t t_heal = w.bus.now;
  w.bus.run_until(12 * S);
  CHECK(w.ctl.takeovers == 1);
  auto c = w.bus.from(0x01, t_heal);
  size_t k = 0;
  while (k < c.size() && !is_walk(c[k].f)) k++;
  CHECK(k < c.size());
  if (k < c.size()) CHECK_HEX(c[k].f, "02' 02 01 FF FF FF FF 80 00 00 00 7E");  // warm walk
  CHECK(w.pgd.handoffs >= 1 && w.pgd.state(w.bus.now) == PgdModel::SESSIONED);
  CHECK(w.ctl.served().size() == 1);
}

// T12 member-forwarded poll answered as itself to 0x01 (no retry), T13
// member-forwarded roll-call ignored, sect. 4 bad check never acked.
static void test_forwards_and_bad_check() {
  Bus bus;
  Script m(0x1F);
  PgdModel pgd;
  bus.attach(&m);
  bus.attach(&pgd);
  bus.transmit(&m, mk(0x20, {0x01, 0x1F}), 1 * MS);
  bus.transmit(&m, rollcall(0x20, 0x1F, bit(32) | bit(31) | bit(1), bit(31)), 10 * MS);
  Frame bad = mk(0x20, {0x0C, 0x08, 0x01, 0x02, 0x05, 0x41});
  bad.back().v ^= 0x10;
  bus.transmit(&m, bad, 20 * MS);
  bus.transmit(&m, mk(0x20, {0x0C, 0x08, 0x01, 0x02, 0x05, 0x41}), 30 * MS);
  bus.run_until(200 * MS);
  auto p = bus.from(0x20);
  CHECK(p.size() == 2);
  if (p.size() == 2) {
    CHECK_HEX(p[0].f, "01' 01 20 DD");
    CHECK_HEX(p[1].f, "01' 03 20 DB");
    CHECK(p[1].t > 30 * MS);
  }
  CHECK(pgd.fwd_polls == 1 && pgd.fwd_rollcalls_ignored == 1 && pgd.bad_ck == 1);
  CHECK(pgd.resends == 0 && pgd.master_walks == 0);  // forwarded reply: no T8
}

// Keypad: report + link reply in one slot; NN 01, 0A at +1 s, +2 / 200 ms.
static void test_keypad_hold() {
  World w;
  w.ctl.start(0);
  w.bus.run_until(3 * S);
  w.pgd.press(0x10, 1500 * MS);
  w.bus.run_until(6 * S);
  std::vector<uint8_t> nn;
  for (auto &l : w.bus.from(0x20, 3 * S))
    if (l.f.size() >= 11 && l.f[1].v == 0x1E) nn.push_back(l.f[5].v);
  CHECK(nn.size() == 4);  // 01, 0A (1.0 s), 0C (1.2 s), 0E (1.4 s), released
  if (nn.size() == 4) CHECK(nn[0] == 0x01 && nn[1] == 0x0A && nn[2] == 0x0C && nn[3] == 0x0E);
  CHECK(w.ctl.keys.size() == 4);
}

// --- fault model ----------------------------------------------------------------

static std::string run_digest(uint64_t seed) {
  World w;
  FaultSpec fs;
  fs.seed = seed;
  fs.p_to_ctrl = 0.3;
  fs.p_byte = 0.001;
  PhysFault f(fs);
  f.attach(w.bus);
  w.ctl.start(0);
  w.run_with_values(20 * S);
  std::string d;
  for (auto &l : w.bus.log) d += std::to_string(l.t) + hex(l.f) + ";";
  return d;
}
static void test_seeded_reproducible() {
  CHECK(run_digest(7) == run_digest(7));
  CHECK(run_digest(7) != run_digest(8));
}

// A16 2.2: drift hits only frames after a long undriven gap; break-byte form.
struct Rec : Station {
  explicit Rec(uint8_t a) : Station(a) {}
  int breaks = 0, frames = 0;
  FrameAsm a;
  void on_rx(const WireByte &b, int64_t) override {
    if (!b.addr && b.err && b.v == 0x00) breaks++;
    Frame f;
    if (a.push(b, f) == 1 && frame_ok(f)) frames++;
  }
};
static void test_drift() {
  World w;
  Rec rec(0x00);
  w.bus.attach(&rec);
  FaultSpec fs;
  fs.drift.push_back({0x00, 1.0, 5 * MS, 6 * MS, true});  // listener: break byte after >= 6 ms
  fs.drift.push_back({0x01, 1.0, 1 * MS, 1 * MS, false}); // controller: address lost after >= 1 ms
  PhysFault f(fs);
  f.attach(w.bus);
  w.ctl.start(0);
  w.bus.run_until(60 * MS);
  // The FF-walk probes are 10 ms apart (> 6 ms idle): every probe after the
  // first carries a break byte at the listener; nothing reaches 0x01 late.
  CHECK(rec.breaks >= 4 && f.break_bytes == static_cast<uint32_t>(rec.breaks));
  w.bus.run_until(20 * S);
  // Poll replies follow after ~245 us: never hit; the pGD (after 0.3 ms)
  // is still served, the controller's walk answers (>1 ms) are not.
  CHECK(w.ctl.joins == 1 && w.pgd.polls > 500);
}

// R9 CTRL_SILENT_PGD_MASTER: the controller stops mid-exchange; the pGD
// retries 3x and masters the empty net (~66 frames/s) forever.
static void test_ctrl_silent_r9() {
  World w;
  FaultSpec fs;
  fs.ctrl_silent_at = 5 * S;
  PhysFault f(fs);
  f.attach(w.bus);
  w.ctl.start(0);
  w.bus.run_until(25 * S);
  CHECK(w.bus.from(0x01, 5 * S + 30 * MS).empty());
  Measures m = measure(w.bus, 6 * S, 25 * S);
  CHECK(m.walks == 0 && m.polls20 == 0);
  double fps = w.bus.from(0x20, 6 * S, 25 * S).size() / 19.0;
  CHECK(fps > 60 && fps < 72);  // A8 R9: ~66 frames/s
  CHECK(m.pgdw10() >= 1.0);     // A8 step 3: pgdw >= 1.0, walks < 0.2, p20 < 5
  CHECK(w.pgd.state(w.bus.now) == PgdModel::MASTER);
}

// Bridge reset with DE floating: garbage collides with the bus for 50 ms;
// the bus recovers on its own.
static void test_bridge_reset_noise() {
  World w;
  FaultSpec fs;
  fs.reset_at = 5 * S;
  fs.reset_us = 50 * MS;
  PhysFault f(fs);
  w.ctl.start(0);
  w.bus.run_until(1 * S);
  f.attach(w.bus);
  w.run_with_values(30 * S);
  CHECK(f.noise_bytes == 261 && w.bus.collisions > 0);
  CHECK(w.ctl.link_faults >= 1);
  CHECK(w.pgd.state(w.bus.now) == PgdModel::SESSIONED);
  CHECK(measure(w.bus, 25 * S, 30 * S).polls20 > 100);
}

// --- calibration (A8 regimes, bridge absent = passive without the bridge) ------

static Measures calib(double p, uint64_t seed, int64_t dur, double stall = 0, double p_reply = -1,
                      double drift = 0) {
  World w;
  FaultSpec fs;
  fs.seed = seed;
  fs.p_to_ctrl = p;
  fs.p_to_ctrl_reply = p_reply;
  fs.p_walk_stall = stall;
  if (drift > 0) fs.drift.push_back({0x01, drift, 2 * MS, 8 * MS, false});
  PhysFault f(fs);
  f.attach(w.bus);
  w.pgd.seed(seed);
  w.ctl.start(0);
  w.run_with_values(20 * S + dur);
  return measure(w.bus, 20 * S, 20 * S + dur);
}

// R2 PGD_ONLY_HEALTHY (p = 0), 10-02 passive (p ~ 5 %), R8 PASSIVE_DEGRADED
// (p ~ 28 %, the A2 T7 first-walk stall after every session-ack fault as
// measured 29/29). Assertions = the A8 sect. 4 passive classifier and the
// `ekobeescope health` LOOP rule, plus tolerance bands around the measured
// numbers (A8 sect. 3 table).
static void test_calibration() {
  const int64_t D = 200 * S;
  Measures r2 = calib(0, 1, D);
  r2.print("R2 p=0");
  CHECK(r2.walks == 0 && r2.warm == 0 && r2.pgd_master_probes == 0 && r2.resent_after_ack == 0);
  CHECK(r2.healthy() && !r2.degraded() && r2.p20() >= 260 && r2.p20() <= 450);
  Measures h = calib(0.05, 1, D);
  h.print("10-02 p=0.05");
  CHECK(h.healthy() && !h.loop());  // walks ~0 (measured 0-0.67 / 10 s)
  CHECK(h.ra() >= 0.02 && h.ra() <= 0.08);  // measured 3.9-5.4 %
  CHECK(h.pgdw10() < 0.5);
  Measures r8 = calib(0.28, 1, D, 1.0);
  r8.print("R8 p=0.28");
  CHECK(r8.degraded() && !r8.healthy() && r8.loop());  // A8 R8 / health LOOP
  CHECK(r8.walks10() >= 1.5 && r8.walks10() <= 4.0);  // measured 2.7-2.9 (sim ~2.1)
  CHECK(r8.ra() >= 0.20 && r8.ra() <= 0.35);          // measured 27-30 %
  CHECK(r8.pgdw10() >= 0.5);                           // pGD masters (51-112 probes / 10 s)
  CHECK(r8.p20() < 150);                               // measured 8-22 / 10 s
}

int main(int argc, char **argv) {
  if (argc > 1) {  // calibration sweep: test_sim_pgd p seed secs [stall]
    Measures m = calib(atof(argv[1]), strtoull(argv[2], nullptr, 0), atoll(argv[3]) * S,
                       argc > 4 ? atof(argv[4]) : 0, argc > 5 ? atof(argv[5]) : -1,
                       argc > 6 ? atof(argv[6]) : 0);
    m.print(argv[1]);
    return 0;
  }
  test_boot_and_join();
  test_turnaround();
  test_retry_master_readoption();
  test_forwards_and_bad_check();
  test_keypad_hold();
  test_seeded_reproducible();
  test_drift();
  test_ctrl_silent_r9();
  test_bridge_reset_noise();
  test_calibration();
  if (fails) {
    fprintf(stderr, "test_sim_pgd: %d check(s) failed\n", fails);
    return 1;
  }
  printf("test_sim_pgd: all checks passed\n");
  return 0;
}
