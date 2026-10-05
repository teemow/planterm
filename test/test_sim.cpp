// Host tests for the byte-level pLAN controller simulator (test/sim/):
// planterm's real PlanTerminal and a scripted pGD stub on a discrete-event
// bus against the uPC controller model, replayed against the healthy
// reference sequences of the wave-A spec chapters (rule IDs inline).
//   c++ -std=c++17 -Wall -Wextra -Werror test/test_sim.cpp -o /tmp/t && /tmp/t

#include "sim/sim_bus.h"
#include "sim/sim_controller.h"
#include "sim/sim_terminals.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace plan::sim;

static int fails = 0;
#define CHECK(c)                                                  \
  do {                                                            \
    if (!(c)) {                                                   \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
      fails++;                                                    \
    }                                                             \
  } while (0)
#define CHECK_HEX(f, s)                                                         \
  do {                                                                          \
    std::string h_ = hex(f);                                                    \
    if (h_ != (s)) {                                                            \
      fprintf(stderr, "%s:%d: got [%s] want [%s]\n", __FILE__, __LINE__, h_.c_str(), s); \
      fails++;                                                                  \
    }                                                                           \
  } while (0)
static bool near(int64_t v, int64_t want, int64_t tol) { return llabs(v - want) <= tol; }

static const int64_t S = 1000000, MS = 1000;

struct World {
  Bus bus;
  SimController ctl;
  PgdStub pgd;
  PlanTerminalStation br;
  World(bool with_pgd, bool with_br) {
    bus.attach(&ctl);
    if (with_pgd) bus.attach(&pgd);
    if (with_br) bus.attach(&br);
  }
  std::vector<LogFrame> to(uint8_t a, uint8_t type, int64_t t0 = 0, int64_t t1 = INT64_MAX) {
    std::vector<LogFrame> r;
    for (auto &l : bus.from(0x01, t0, t1))
      if (l.f[0].v == a && l.f.size() > 1 && l.f[1].v == type) r.push_back(l);
    return r;
  }
};
static bool is_walk(const Frame &f) {  // R-RC-07/R-LL-11: claims not required to be 00
  return f.size() == 12 && f[0].v == 0x02 && f[1].v == 0x02 && f[2].v == 0x01 &&
         f[3].v == 0xFF && f[4].v == 0xFF && f[5].v == 0xFF && f[6].v == 0xFF;
}
static std::vector<int64_t> walk_starts(const Bus &b) {
  std::vector<int64_t> r;
  for (auto &l : b.from(0x01))
    if (is_walk(l.f)) r.push_back(l.t);
  return r;
}
static size_t idx_after(const std::vector<LogFrame> &v, int64_t t) {
  size_t i = 0;
  while (i < v.size() && v[i].t < t) i++;
  return i;
}

// R-RC-01, R-SE-01/03/04/05, R-DI-02/05/10, R-KP-01: encoders reproduce
// captured frames byte for byte; any single-byte flip fails the check.
static void test_codec() {
  CHECK_HEX(rollcall(0x1F, 0x01, bit(32) | bit(1), bit(32)), "1F' 02 01 80 00 00 01 80 00 00 00 DC");
  CHECK_HEX(rollcall(0x02, 0x01, 0xFFFFFFFF, 0), "02' 02 01 FF FF FF FF 00 00 00 00 FE");
  CHECK_HEX(rollcall(0x01, 0x01, 0xC0000001, 0xC0000000), "01' 02 01 C0 00 00 01 C0 00 00 00 7A");
  CHECK_HEX(mk(0x20, {0x66, 0x08, 0x01, 0x00, 0x01}), "20' 66 08 01 00 01 9D 13");
  CHECK_HEX(mk(0x1F, {0x50, 0x05, 0x01}), "1F' 50 05 01 8A");
  CHECK_HEX(mk(0x1F, {0x0A, 0x06, 0x01, 0x00}), "1F' 0A 06 01 00 CF");
  CHECK_HEX(mk(0x01, {0x51, 0x07, 0x1F, 0x0A, 0x17}), "01' 51 07 1F 0A 17 66");
  CHECK_HEX(mk(0x01, {0x1E, 0x07, 0x1F, 0x01, 0x01}), "01' 1E 07 1F 01 01 B8");
  CHECK_HEX(mk(0x20, {0x0C, 0x08, 0x01, 0x02, 0x13, 0x35}), "20' 0C 08 01 02 13 35 80");
  CHECK(bit(31) == 0x40000000u && bit(1) == 1u);
  for (Frame f : {rollcall(0x1F, 1, 0xC0000001, 0x40000000), mk(0x20, {0x66, 0x08, 0x01, 0x00, 0x01})}) {
    CHECK(frame_ok(f));
    for (size_t i = 1; i < f.size(); i++) {
      Frame g = f;
      g[i].v ^= 0x10;
      CHECK(!frame_ok(g));
    }
  }
}

// pGD-only healthy bus (10-02 passive reference): one cold FF-walk, the
// CeCeCL adoption, then 0 walks, 24.07 ms polls, 12 s gap walks 02..1F.
static void test_pgd_only_healthy() {
  World w(true, false);
  w.ctl.start(0);
  w.bus.run_until(62 * S);
  CHECK(w.bus.collisions == 0);
  CHECK(w.ctl.ff_walks == 1 && w.ctl.link_faults == 0);  // R-LL-10: no faults, no walks
  auto c = w.bus.from(0x01);
  // R-RC-05 / R-LL-11: probes 02..20, 10 ms apart, MAP shrinking.
  for (int i = 0; i < 31; i++) {
    CHECK(c[i].f[0].v == 2 + i && c[i].f[1].v == 0x02);
    if (i) CHECK(near(c[i].t - c[i - 1].t, 10 * MS, 2 * MS));
  }
  CHECK_HEX(c[29].f, "1F' 02 01 C0 00 00 01 00 00 00 00 1C");
  CHECK_HEX(c[30].f, "20' 02 01 80 00 00 01 00 00 00 00 5B");
  auto p = w.bus.from(0x20);
  // R-RC-12 / R-SE-09 / R-LL-14 (c00): echo, echo, link reply; ctl confirms x2.
  CHECK_HEX(p[0].f, "01' 02 20 80 00 00 01 80 00 00 00 DB");
  CHECK_HEX(c[31].f, "20' 02 01 80 00 00 01 80 00 00 00 DB");
  CHECK_HEX(p[1].f, "01' 02 20 80 00 00 01 80 00 00 00 DB");
  CHECK_HEX(c[32].f, "20' 02 01 80 00 00 01 80 00 00 00 DB");
  CHECK_HEX(p[2].f, "01' 01 20 DD");
  CHECK_HEX(c[33].f, "20' 01 01 DD");
  CHECK_HEX(p[3].f, "01' 01 20 DD");
  CHECK_HEX(c[34].f, "01'");
  CHECK(near(c[32].t - c[31].t, 6 * MS, 300));  // R-LL-15: echo/confirm loop ~6.0 ms
  // R-SE-10/13: one ident, one init, the page painted, nothing re-sessioned.
  CHECK(w.pgd.idents == 1 && w.pgd.inits == 1);
  for (int r = 0; r < 8; r++) CHECK(w.pgd.screen[r] == w.ctl.screen.pages[0].rows[r]);
  // R-LL-02: 24.07 ms cadence -> 50 s / 24.07 ms polls, every one acked (R-LL-04).
  auto polls = w.to(0x20, 0x01, 10 * S, 60 * S);
  CHECK(near(static_cast<int64_t>(polls.size()), 2077, 3));
  CHECK(w.ctl.acks == w.ctl.polls);
  // R-RC-03: gap walks every 12.0 s, 02..1F with the settled masks, never 20.
  auto g1 = w.to(0x02, 0x02, 1 * S);
  CHECK(g1.size() == 5);
  for (size_t i = 1; i < g1.size(); i++) CHECK(near(g1[i].t - g1[i - 1].t, 12 * S, 40 * MS));
  CHECK_HEX(g1[0].f, "02' 02 01 80 00 00 01 80 00 00 00 F9");
  CHECK(w.to(0x1F, 0x02, 1 * S).size() == 5 && w.to(0x20, 0x02, 1 * S).empty());
  auto t1f = w.to(0x1F, 0x02, 1 * S);
  CHECK(near(t1f[0].t - g1[0].t, 29 * 36 * MS, 30 * MS));  // one probe per ~36 ms
}

// Drops every byte one station sends toward one receiver inside a window.
struct DropHook : FaultHook {
  uint8_t from, to;
  int64_t t0, t1;
  DropHook(uint8_t f, uint8_t t, int64_t a, int64_t b) : from(f), to(t), t0(a), t1(b) {}
  bool on_rx_byte(uint8_t f, uint8_t t, WireByte &, int64_t now) override {
    return !(f == from && t == to && now >= t0 && now < t1);
  }
};

// Link fault -> FF-walk (R-LL-06, R-LL-10, R-RC-05) and the re-adoption
// re-session that repaints the page the application is on (R-SE-10/16).
// Also a pGD key in its own poll slot (R-KP-04/17, R-DI-12).
static void test_link_fault_walk() {
  World w(true, false);
  DropHook h(0x20, 0x01, 30 * S, 30 * S + 70 * MS);
  w.bus.hook = &h;
  w.ctl.start(0);
  w.bus.run_until(20 * S);
  w.pgd.press(0x10);
  w.bus.run_until(21 * S);
  CHECK(w.ctl.keys.size() == 1 && w.ctl.keys[0] == 0x10);
  auto f65 = w.to(0x20, 0x65, 20 * S);
  CHECK(f65.size() == 1);
  CHECK(w.pgd.screen[0] == w.ctl.screen.pages[1].rows[0]);
  w.bus.run_until(40 * S);
  CHECK(w.ctl.link_faults == 1 && w.ctl.ff_walks == 2);
  auto polls = w.to(0x20, 0x01, 30 * S, 31 * S);
  // The first poll in the window gets no ack; 3 polls ~20 ms apart.
  size_t i = 0;
  while (i < polls.size() && polls[i].t < 30 * S) i++;
  CHECK(polls.size() >= i + 3);
  CHECK(near(polls[i + 1].t - polls[i].t, 20 * MS, 2 * MS));
  CHECK(near(polls[i + 2].t - polls[i].t, 40 * MS, 2 * MS));
  int64_t last_end = 0;
  for (auto &l : w.bus.log)
    if (l.t < 31 * S && l.t > 30 * S) last_end = std::max(last_end, l.end());
  auto ws = walk_starts(w.bus);
  CHECK(ws.size() == 2);
  CHECK(near(ws[1] - last_end, 2 * S, 10 * MS));  // R-LL-10: 2.00 s of silence
  // R-SE-10/16: re-adopted, re-identified, repainted on page 1 (not page 0).
  CHECK(w.pgd.idents == 2 && w.pgd.inits == 2);
  CHECK(w.pgd.screen[0] == w.ctl.screen.pages[1].rows[0]);
}

// Corrupts the checksum of every session ack 0x20 sends to the controller
// inside a window (the frame is heard, not accepted).
struct SackCorrupt : FaultHook {
  int64_t t0, t1;
  int idx = 0;
  bool sack = false;
  SackCorrupt(int64_t a, int64_t b) : t0(a), t1(b) {}
  bool on_rx_byte(uint8_t f, uint8_t t, WireByte &b, int64_t now) override {
    if (f != 0x20 || t != 0x01) return true;
    idx = b.addr ? 0 : idx + 1;
    if (idx == 1) sack = b.v == 0x03;
    if (idx == 3 && sack && now >= t0 && now < t1) b.v ^= 0x01;
    return true;
  }
};

// R-LL-08 / R-DI-04: an unaccepted SACK -> identical copy 14 ms after the
// SACK start, at most 3 copies, then the link fault (2.00 s, FF-walk).
static void test_sack_retry() {
  World w(true, false);
  SackCorrupt h(20 * S, 20 * S + 200 * MS);
  w.bus.hook = &h;
  w.ctl.start(0);
  w.bus.run_until(20 * S);
  w.ctl.set_row(0, 2, "    Hotwater:   36.0\xDF" "C", w.bus.now);  // R-DI-08: 2 chars -> 0x0B
  w.bus.run_until(25 * S);
  auto rows = w.to(0x20, 0x0B, 20 * S, 21 * S);
  CHECK(rows.size() == 3);
  CHECK(rows.size() == 3 && hex(rows[0].f) == hex(rows[1].f) && hex(rows[1].f) == hex(rows[2].f));
  auto sk = w.bus.from(0x20, 20 * S, 21 * S);
  if (rows.size() == 3 && !sk.empty()) {
    size_t j = idx_after(sk, rows[0].t);
    CHECK(near(rows[1].t - sk[j].t, 14 * MS, 2 * MS));
  }
  CHECK(w.ctl.resends == 2 && w.ctl.link_faults == 1);
  auto ws = walk_starts(w.bus);
  CHECK(ws.size() == 2 && rows.size() == 3);
  if (ws.size() == 2) CHECK(near(ws[1] - rows[2].end(), 2 * S, 10 * MS));
  CHECK(w.to(0x20, 0x0C, 20 * S, 21 * S).empty());
}

// The July reference join (dualterm-soak-1120): pGD served alone, the bridge
// joins on the controller's normal 12 s gap walk and both end up served.
static void test_july_gap_walk_join() {
  World w(true, true);
  w.ctl.start(0);
  w.bus.run_until(5 * S);
  w.br.term.enroll_ = true;
  w.bus.run_until(30 * S);
  CHECK(w.bus.collisions == 0);
  CHECK(w.ctl.ff_walks == 1 && w.ctl.link_faults == 0 && w.ctl.joins == 2);
  CHECK(w.ctl.map == 0xC0000001u && w.ctl.claims == 0xC0000000u && w.ctl.focus == 0x1F);
  auto c = w.bus.from(0x01, 5 * S);
  size_t i = 0;
  while (i < c.size() && hex(c[i].f) != "1F' 02 01 80 00 00 01 80 00 00 00 DC") i++;
  CHECK(i + 8 < c.size());
  if (i + 8 >= c.size()) return;
  int64_t tj = c[i].t;
  auto b = w.bus.from(0x1F);
  CHECK_HEX(b[0].f, "01' 02 1F C0 00 00 01 C0 00 00 00 5C");  // R-RC-12
  // R-RC-13 / R-LL-14 (bridge, July): e, e, e, then the link reply.
  CHECK_HEX(b[1].f, "01' 02 1F C0 00 00 01 C0 00 00 00 5C");
  CHECK_HEX(b[2].f, "01' 02 1F C0 00 00 01 C0 00 00 00 5C");
  CHECK_HEX(b[3].f, "01' 01 1F DE");
  // R-RC-14: confirm x2, poll 1F, ack, re-probe 20, re-probe 1F, close, gap restart.
  const char *want[] = {"1F' 02 01 C0 00 00 01 C0 00 00 00 5C", "1F' 02 01 C0 00 00 01 C0 00 00 00 5C",
                        "1F' 01 01 DE", "01'", "20' 02 01 C0 00 00 01 C0 00 00 00 5B",
                        "1F' 02 01 C0 00 00 01 C0 00 00 00 5C", "01' 02 01 C0 00 00 01 C0 00 00 00 7A",
                        "02' 02 01 C0 00 00 01 C0 00 00 00 79"};
  for (int k = 0; k < 8; k++) CHECK_HEX(c[i + 1 + k].f, want[k]);
  CHECK(c[i + 3].t - tj < 20 * MS);  // R-RC-13: stages complete within ~20 ms
  // R-RC-15 / R-SE-11: the existing pGD is re-inited first, then the joiner's ident.
  auto i66 = w.to(0x20, 0x66, tj), i0a = w.to(0x1F, 0x0A, tj);
  CHECK(!i66.empty() && !i0a.empty() && i66[0].t < i0a[0].t);
  CHECK(w.pgd.idents == 1 && w.pgd.inits == 2);
  bool ident_ok = false;
  for (auto &l : w.bus.from(0x1F)) ident_ok |= hex(l.f) == "01' 51 07 1F 0A 17 66";
  CHECK(ident_ok);
  // R-RC-03: after the join the gap walk ends at 1E; 1F and 20 never probed.
  CHECK(w.to(0x1E, 0x02, tj).size() >= 2);
  CHECK(w.to(0x1F, 0x02, c[i + 7].t).empty() && w.to(0x20, 0x02, c[i + 7].t).empty());
  // R-RC-19 / R-LL-19a: with forwarding on, 31 hands every other token to
  // 32, the pGD answers as itself (not acked), the controller re-polls 31.
  w.br.term.fwd_polls_ = 1;
  uint32_t a0 = w.ctl.acks;
  w.bus.run_until(31 * S);
  CHECK(w.pgd.fwd_polls > 15 && w.ctl.fwd_answered == w.pgd.fwd_polls && w.ctl.fwd_lost == 0);
  CHECK(w.ctl.acks - a0 >= 35 && w.ctl.link_faults == 0 && w.bus.collisions == 0);
  // R-KP-17 / R-DI-31: a bridge key moves the shared page; mirrored, 0x20 first.
  int64_t tk = w.bus.now;
  w.br.press(0x10);
  w.bus.run_until(33 * S);
  CHECK(w.ctl.keys.size() == 1);
  auto k20 = w.to(0x20, 0x65, tk), k1f = w.to(0x1F, 0x65, tk);
  CHECK(k20.size() == 1 && k1f.size() == 1 && k20[0].t < k1f[0].t);
  size_t n20 = 0, n1f = 0;
  for (auto &l : w.bus.from(0x01, tk))
    if (l.f.size() > 4 && l.f[1].v >= 0x0B) (l.f[0].v == 0x20 ? n20 : n1f)++;
  CHECK(n20 == n1f && n20 > 0);
  for (int r = 0; r < 8; r++) CHECK(w.pgd.screen[r] == w.ctl.screen.pages[1].rows[r]);
}

// KNOWN DEVIATION (A2 D3 / R-LL-17): planterm forwards the FIRST poll after
// its join when forwarding is on, so the strict controller drops the join.
// This pins today's behaviour; it flips when planterm answers that poll itself.
static void test_known_deviation_forward_on_join_poll() {
  World w(true, true);
  w.ctl.start(0);
  w.bus.run_until(5 * S);
  w.br.term.enroll_ = true;
  w.br.term.fwd_polls_ = 1;
  w.bus.run_until(20 * S);
  auto b = w.bus.from(0x1F);
  CHECK(b.size() >= 4);
  if (b.size() >= 4) CHECK_HEX(b[3].f, "20' 01 1F BF");  // spec wants 01' 01 1F DE
  CHECK(w.ctl.link_faults >= 1 && w.ctl.joins >= 1);
}

// Corrupts the first roll-call echo 0x1F sends to the controller.
struct FirstEchoCorrupt : FaultHook {
  int idx = 0, frames = 0;
  bool on_rx_byte(uint8_t f, uint8_t t, WireByte &b, int64_t) override {
    if (f != 0x1F || t != 0x01) return true;
    if (b.addr) idx = 0, frames++;
    else idx++;
    if (frames == 1 && idx == 11) b.v ^= 0x01;
    return true;
  }
};

// R-RC-09 / R-RC-18 / R-RC-20: an FF-walk the bridge ends at 31 leaves
// CLAIMS = {31}: 0x20 is never probed and never painted. R-RC-10: a rejected
// answer aborts the walk (no 20' probe), 2.00 s, a new walk.
static void test_walk_ends_at_31() {
  for (int rejected = 0; rejected < 2; rejected++) {
    World w(true, true);
    FirstEchoCorrupt h;
    if (rejected) w.bus.hook = &h;
    w.br.term.enroll_ = true;
    w.ctl.start(0);
    w.bus.run_until(30 * S);
    CHECK(w.ctl.claims == bit(31) && w.ctl.focus == 0x1F);
    CHECK(w.to(0x20, 0x02).empty() && w.pgd.sacks == 0 && w.pgd.polls == 0);
    CHECK(w.ctl.ff_walks == 1u + rejected && w.ctl.link_faults == static_cast<uint32_t>(rejected));
    auto c = w.bus.from(0x01);
    size_t k = 0;
    while (k < c.size() && c[k].f[0].v != 0x1F) k++;
    // R-RC-09: confirm, confirm, poll 1F; no higher address probed.
    if (!rejected) {
      CHECK(c[k + 1].f[0].v == 0x1F && c[k + 2].f[0].v == 0x1F);
      CHECK_HEX(c[k + 3].f, "1F' 01 01 DE");
    } else {
      auto ws = walk_starts(w.bus);
      CHECK(ws.size() == 2 && c[k + 1].t == ws[1]);  // nothing between the echo and the walk
      auto e = w.bus.from(0x1F);
      CHECK(ws.size() == 2 && near(ws[1] - e[0].end(), 2 * S, 10 * MS));
    }
  }
}

// R-KP-01/04/11, R-DI-08/09/12/16/17: a bridge key burst, the controller's
// `01'` within 1 ms, then 0x65, delta rows ascending, 0x0C for one char.
static void test_bridge_keypad() {
  World w(false, true);
  w.br.term.enroll_ = true;
  w.ctl.start(0);
  w.bus.run_until(5 * S);
  CHECK(w.ctl.focus == 0x1F && w.ctl.idents == 1);
  for (int n = 0; n < 2; n++) {
    int64_t tk = w.bus.now;
    w.br.press(0x10);
    w.bus.run_until(tk + 1 * S);
    auto b = w.bus.from(0x1F, tk);
    size_t j = 0;
    while (j < b.size() && b[j].f.size() != 11) j++;
    CHECK(j < b.size());
    if (j >= b.size()) return;
    CHECK_HEX(b[j].f, "01' 1E 07 1F 10 01 A9 01' 01 1F DE");
    auto c = w.bus.from(0x01, b[j].t);
    CHECK_HEX(c[0].f, "01'");
    CHECK(c[0].t - b[j].end() < 1 * MS);  // R-KP-11
    auto d = w.bus.from(0x01, b[j].t + 1 * MS);
    std::vector<std::string> types;
    for (auto &l : d)
      if (l.f.size() > 4 && l.f[1].v >= 0x0B) types.push_back(hex(Frame(l.f.begin(), l.f.begin() + 2)));
    CHECK(!types.empty() && types[0] == "1F' 65");
    if (n == 1) {  // 1/8 -> 2/8: one 0x0C (r0 c19 '2'), no rows, then the band
      bool cell = false;
      for (auto &l : d) cell |= hex(l.f) == hex(mk(0x1F, {0x0C, 0x08, 0x01, 0x00, 0x13, '2'}));
      CHECK(cell);
      CHECK(w.to(0x1F, 0x0B, b[j].t).empty());
    }
  }
  CHECK(w.ctl.keys.size() == 2 && w.ctl.screen.cur == 2 && w.ctl.link_faults == 0);
}

int main() {
  test_codec();
  test_pgd_only_healthy();
  test_link_fault_walk();
  test_sack_retry();
  test_july_gap_walk_join();
  test_known_deviation_forward_on_join_poll();
  test_walk_ends_at_31();
  test_bridge_keypad();
  if (fails) {
    fprintf(stderr, "test_sim: %d check(s) failed\n", fails);
    return 1;
  }
  printf("test_sim: all checks passed\n");
  return 0;
}
