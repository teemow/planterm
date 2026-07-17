// Host-compilable self-check for the navigation/scrape engine: a scripted
// fake heat pump (screens and key semantics per the reference device's
// ground truth) driven purely through PlanNav's tick(), over a scrape route
// expressed as pure ScrapeStep data -- the same shape a consumer supplies.
// Asserts the full verified route (anchor -> alarm -> D01 -> fixed-budget
// Down walk across the D pages -> service-menu seek -> D14 -> anchor), the
// emitted page sequence including the scrolled last-page views, the
// running-state D-list variant that sticks early, and the failure path
// (recovery to the anchor + exponential backoff).
//   c++ -std=c++17 test/test_plan_nav.cpp -o /tmp/t && /tmp/t

#include "../src/plan_nav.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace plan;

// --- device data fixtures (the consumer-side tables) -------------------------

static constexpr int MENU_N = 8;
static const char *const MENU_LABELS[MENU_N] = {
    "A.On/Off Unit", "B.Setpoint", "C.Clock/Scheduler", "D.Input/Output",
    "E.Data logger", "F.Information", "G.Service", "H.Manufacturer",
};
static const NavMenu MAIN_MENU{"Main menu", 8, MENU_LABELS, MENU_N};

// The D-walk is a fixed budget of Downs, NOT a walk to a target page ID:
// the reference controller renumbers the D pages with the unit state, and
// within the list the last section scrolls one row per press and wraps.
static constexpr int WALK_DOWNS = 13;
static constexpr int SEEK_SPAN = 7;

// The reference scrape route: status anchor -> Alarm page -> the D page
// walk -> a service-menu seek to the expansion-valve page -> anchor.
static const ScrapeStep ROUTE[] = {
    // emit the status anchor
    {0, 0, false, NEXP_NONE, 0, nullptr, nullptr, true, 0},
    // Alarm -> alarms page, emit
    {KEY_ALARM, 0, false, NEXP_ROW_PREFIX, 0, "Alarms", nullptr, true, 0},
    // Esc back to the anchor
    {KEY_ESC, 0, false, NEXP_ANCHOR, 0, nullptr, nullptr, false, 0},
    // Prg -> main menu, cursor onto D.Input/Output, Enter lands on D01
    {KEY_PRG, 0, false, NEXP_MENU, 0, nullptr, &MAIN_MENU, false, 0},
    {0, 0, false, NEXP_NONE, 0, "D.Input/Output", &MAIN_MENU, false, 0},
    {KEY_ENTER, 0, false, NEXP_PAGE, 0, "D01", nullptr, true, 0},
    // the fixed-budget Down walk, every press verified to stay on a D page
    {KEY_DOWN, 0, false, NEXP_PAGE_GLOB, 0, "D##", nullptr, true, WALK_DOWNS},
    // Esc x2 back to the anchor (D page -> menu -> anchor)
    {KEY_ESC, 0, false, NEXP_MENU, 0, nullptr, &MAIN_MENU, false, 0},
    {KEY_ESC, 0, false, NEXP_ANCHOR, 0, nullptr, nullptr, false, 0},
    // Prg -> main menu, cursor onto G.Service, Enter into the sub-menu
    {KEY_PRG, 0, false, NEXP_MENU, 0, nullptr, &MAIN_MENU, false, 0},
    {0, 0, false, NEXP_NONE, 0, "G.Service", &MAIN_MENU, false, 0},
    {KEY_ENTER, 0, false, NEXP_NONE, 0, nullptr, nullptr, false, 0},
    // seek the selection band onto the valve entry, Enter -> D14, emit
    {0, SEEK_SPAN, true, NEXP_NONE, 0, "Expansion valve", nullptr, false, 0},
    {KEY_ENTER, 0, false, NEXP_PAGE, 0, "D14", nullptr, true, 0},
};
static constexpr size_t ROUTE_N = sizeof(ROUTE) / sizeof(ROUTE[0]);

// A PIN-gated route: into the Service menu, band-seek onto the PW1-gated
// entry, Enter (pin = 1: the gate may show -- type PW1 -- or pass through
// when the session already passed it), verify + emit the target page.
static const ScrapeStep PIN_ROUTE[] = {
    {KEY_PRG, 0, false, NEXP_MENU, 0, nullptr, &MAIN_MENU, false, 0},
    {0, 0, false, NEXP_NONE, 0, "G.Service", &MAIN_MENU, false, 0},
    {KEY_ENTER, 0, false, NEXP_NONE, 0, nullptr, nullptr, false, 0},
    {0, SEEK_SPAN, true, NEXP_NONE, 0, "Service settings", nullptr, false, 0},
    {KEY_ENTER, 0, false, NEXP_PAGE, 0, "Gfc11", nullptr, true, 0, 1},
};
static constexpr size_t PIN_ROUTE_N = sizeof(PIN_ROUTE) / sizeof(PIN_ROUTE[0]);

// --- fake device --------------------------------------------------------------

// PlanNav only READS a PlanScreen; the fake paints it directly through the
// protected state instead of synthesizing wire frames.
struct TestScreen : PlanScreen {
  void put_row(uint8_t addr, int r, const char *text, uint32_t now) {
    int t = term_index(addr);
    std::snprintf(rows_[t][r], SCR_COLS + 1, "%-22.22s", text);
    row_set_[t][r] = true;
    painted_[t] = now;
  }
  void clear_(uint8_t addr, uint32_t now) {
    int t = term_index(addr);
    for (int r = 0; r < static_cast<int>(SCR_ROWS); r++)
      put_row(addr, r, "", now);
    for (int y = 0; y <= SCR_BAND_MAX_Y; y++)
      bands_[t][y].set = false;
  }
  void select_row(uint8_t addr, int r) {
    int t = term_index(addr);
    Band b;
    b.h = 8;
    b.inverse = true;
    b.set = true;
    bands_[t][r * 8] = b;
  }
};

// The fake device: current page + cursor state, repainted instantly on a key
// press (the 600 ms settle wait still runs -- PlanNav can't tell the
// difference from a fast repaint).
struct FakePump {
  TestScreen scr;
  enum P { STATUS, ALARM, MENU, DPAGE, SERVICE, D14P, PW_GATE, GFC11P } page = STATUS;
  int menu_cur = 5;  // last-used main-menu cursor (1-based), like the real device
  int dnum = 1;      // D01..d_max
  int d_max = 10;    // last D page ID (10 idle; 6 with the unit running)
  int d10_scroll = 0;  // rows scrolled within the last D page (ID unchanged)
  int svc = 5;       // service sub-menu entry index (0-based)
  bool alarm_broken = false;  // failure-path test: Alarm key does nothing

  // PW1 gate in front of e.Service settings (same live-recorded model as
  // test_plan_edit.cpp's fake: digit stages typed by Ups, Enters advance)
  bool pw1_passed = false;
  int gate_val = 0, gate_enters = 0, gate_stage = 0;
  std::vector<uint8_t> gate_keys;  // every key the gate saw

  // The last D page's scrolling output list (ground truth: 06 sits below
  // the landing view's fold; there is no 05 row).
  static constexpr int D10_N = 5;
  const char *d10_rows[D10_N] = {
      "01=Compres.1:      Off", "02=Heatsourcepump: Off", "03=Heating pump  : Off",
      "04=DHW pump      : Off", "06=Emergencyheater:Off",
  };

  static constexpr int SVC_N = 7;
  const char *svc_labels[SVC_N] = {
      "a.Working hours",   "b.Probe adjustment", "c.Expansion valve", "d.Thermal protect.",
      "e.Service settings", "f.Manual managem.",  "g.Var.log",
  };

  void gate_key(uint8_t k) {
    gate_keys.push_back(k);
    if (k == KEY_ESC) { page = SERVICE; return; }
    if (k == KEY_UP && gate_enters > 0) {
      static const int W[3] = {100, 10, 1};
      gate_val += W[gate_stage];
      return;
    }
    if (k == KEY_ENTER) {
      gate_enters++;
      if (gate_enters == 1) { gate_stage = 0; return; }
      if (gate_stage < 2) { gate_stage++; return; }
      // the Enter leaving the units digit confirms (0815 recording)
      if (gate_val == 815) { pw1_passed = true; page = GFC11P; }
      else { gate_val = 0; gate_enters = 0; gate_stage = 0; }
    }
  }

  void press(uint8_t k, uint32_t now) {
    if (page == PW_GATE) { gate_key(k); paint(now); return; }
    switch (k) {
      case KEY_ESC:
        if (page == ALARM || page == MENU)
          page = STATUS;
        else if (page == DPAGE)
          page = MENU, menu_cur = 4;
        else if (page == SERVICE)
          page = MENU, menu_cur = 7;
        else if (page == D14P || page == GFC11P)
          page = SERVICE;
        break;
      case KEY_ALARM:
        if (page == STATUS && !alarm_broken)
          page = ALARM;
        break;
      case KEY_PRG:
        if (page == STATUS)
          page = MENU;  // lands at the LAST-USED cursor position
        break;
      case KEY_DOWN:
        if (page == MENU && menu_cur < MENU_N)
          menu_cur++;
        else if (page == DPAGE && dnum < d_max)
          dnum++;
        else if (page == DPAGE && d10_scroll < D10_N - 4)
          d10_scroll++;  // the last D page scrolls; the ID never advances
        else if (page == SERVICE && svc < SVC_N - 1)
          svc++;
        break;
      case KEY_UP:
        if (page == MENU && menu_cur > 1)
          menu_cur--;
        else if (page == SERVICE && svc > 0)
          svc--;
        break;
      case KEY_ENTER:
        if (page == MENU && menu_cur == 4)
          page = DPAGE, dnum = 1, d10_scroll = 0;  // Enter lands on D01
        else if (page == MENU && menu_cur == 7)
          page = SERVICE;
        else if (page == SERVICE && std::strstr(svc_labels[svc], "Expansion valve"))
          page = D14P;
        else if (page == SERVICE && std::strstr(svc_labels[svc], "Service settings")) {
          if (pw1_passed) { page = GFC11P; }
          else { page = PW_GATE; gate_val = 0; gate_enters = 0; gate_stage = 0; }
        }
        break;
    }
    paint(now);
  }

  void paint(uint32_t now) {
    scr.clear_(SCR_TERM_ESP, now);
    char buf[32];
    switch (page) {
      case STATUS:
        scr.put_row(SCR_TERM_ESP, 0, "02:18 03/07/26 Ekobee1", now);
        break;
      case ALARM:
        scr.put_row(SCR_TERM_ESP, 0, "Alarms", now);
        break;
      case MENU:
        std::snprintf(buf, sizeof buf, "Main menu          %d/8", menu_cur);
        scr.put_row(SCR_TERM_ESP, 0, buf, now);
        scr.put_row(SCR_TERM_ESP, 3, MENU_LABELS[menu_cur - 1], now);
        scr.select_row(SCR_TERM_ESP, 3);
        break;
      case DPAGE:
        std::snprintf(buf, sizeof buf, " Input/Output      D%02d", dnum);
        scr.put_row(SCR_TERM_ESP, 0, buf, now);
        if (dnum == d_max) {  // a 4-row window over the scrolling output list
          scr.put_row(SCR_TERM_ESP, 1, "Digital outputs", now);
          const int rows[4] = {3, 4, 6, 7};
          for (int i = 0; i < 4 && d10_scroll + i < D10_N; i++)
            scr.put_row(SCR_TERM_ESP, rows[i], d10_rows[d10_scroll + i], now);
        }
        break;
      case SERVICE:
        scr.put_row(SCR_TERM_ESP, 0, " Service", now);
        scr.put_row(SCR_TERM_ESP, 2, svc_labels[svc], now);
        scr.select_row(SCR_TERM_ESP, 2);
        break;
      case D14P:
        scr.put_row(SCR_TERM_ESP, 0, " Valve             D14", now);
        break;
      case PW_GATE:
        scr.put_row(SCR_TERM_ESP, 0, "Service Password", now);
        std::snprintf(buf, sizeof buf, "password (PW1):   %04d", gate_val);
        scr.put_row(SCR_TERM_ESP, 5, buf, now);
        break;
      case GFC11P:
        scr.put_row(SCR_TERM_ESP, 0, " Termoreg.       Gfc11", now);
        break;
    }
  }
};

// Step simulated time in 50 ms ticks until the predicate holds (bounded).
template<typename Pred>
static void run_until(PlanNav &nav, uint32_t &now, Pred done) {
  for (int i = 0; i < 100000 && !done(); i++) {
    now += 50;
    nav.tick(now, true);
  }
  assert(done());
}

int main() {
  // nav_menu_cursor_in (planscope TestMenuCursor)
  assert(nav_menu_cursor_in("Main menu          1/8", MAIN_MENU) == 1);
  assert(nav_menu_cursor_in("Main menu          5/8", MAIN_MENU) == 5);
  assert(nav_menu_cursor_in("02:18 03/07/26 Ekobee1", MAIN_MENU) == 0);
  assert(nav_menu_cursor_in(" On/Off Unit       A01", MAIN_MENU) == 0);
  assert(nav_menu_cursor_in("", MAIN_MENU) == 0);

  // nav_page_glob (the D-walk expectation)
  assert(nav_page_glob("D01", "D##"));
  assert(nav_page_glob("D14", "D##"));
  assert(!nav_page_glob("D1", "D##"));
  assert(!nav_page_glob("Gd01", "D##"));
  assert(!nav_page_glob("D014", "D##"));
  assert(!nav_page_glob("", "D##"));

  {  // full happy-path cycle
    uint32_t now = 1000;
    FakePump pump;
    pump.paint(now);
    PlanNav nav(pump.scr, ROUTE, ROUTE_N);
    std::vector<std::string> emitted;
    bool saw_emergency = false;  // the below-the-fold last-page row must be emitted
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    nav.set_emit([&] {
      const char *rows[FIELDS_ROWS];
      for (size_t r = 0; r < FIELDS_ROWS; r++) {
        rows[r] = pump.scr.row(SCR_TERM_ESP, r);
        if (std::strstr(rows[r], "Emergencyheater") != nullptr)
          saw_emergency = true;
      }
      char p[FIELDS_PAGE_MAX];
      page_of(rows, p);
      emitted.push_back(p);
    });
    nav.set_interval_ms(60000);
    nav.enable(now);
    uint32_t t_start = now;
    run_until(nav, now, [&] { return nav.cycles() == 1 && nav.idle(); });

    assert(nav.fails() == 0);
    assert(saw_emergency);
    // A keyless emit settles on an already-quiet screen instead of
    // re-serving the full NAV_QUIET_MS wait: measured on this fake, the
    // cycle runs 32.4 s single-settle vs 41.3 s with the old double-settle
    // (17 emits x ~520 ms). The bound sits between the two, so a regression
    // re-adding the second wait fails here.
    assert(now - t_start < 37000);
    std::vector<std::string> want = {"status", "alarm"};
    for (int d = 1; d <= 10; d++) {
      char b[8];
      std::snprintf(b, sizeof b, "D%02d", d);
      want.push_back(b);
    }
    // the budget's remaining Downs scroll within D10; the ID stays put
    for (int i = 0; i < WALK_DOWNS - 9; i++)
      want.push_back("D10");
    want.push_back("D14");
    if (emitted != want) {
      std::printf("FAIL route: emitted");
      for (auto &e : emitted)
        std::printf(" %s", e.c_str());
      std::printf("\n");
      return 1;
    }
    assert(pump.page == FakePump::STATUS);            // ended back on the anchor
    assert(nav.next_run_ms() == now + 60000);         // no backoff after success

    // second cycle runs at the scheduled time and emits the same route
    run_until(nav, now, [&] { return nav.cycles() == 2 && nav.idle(); });
    assert(emitted.size() == 2 * want.size());
  }

  {  // running-state D list: the controller renumbers the pages and the
     // list already sticks at D06. The fixed budget must complete the
     // cycle instead of failing on a target ID.
    uint32_t now = 1000;
    FakePump pump;
    pump.d_max = 6;
    pump.paint(now);
    PlanNav nav(pump.scr, ROUTE, ROUTE_N);
    int emits = 0;
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    nav.set_emit([&] { emits++; });
    nav.set_interval_ms(60000);
    nav.enable(now);
    run_until(nav, now, [&] { return nav.cycles() == 1 && nav.idle(); });
    assert(nav.fails() == 0);
    // status + alarm + D landing + walk budget + D14
    assert(emits == 4 + WALK_DOWNS);  // full route despite the short list
    assert(pump.page == FakePump::STATUS);
  }

  {  // failure path: Alarm key dead -> recover to anchor, back off, then heal
    uint32_t now = 1000;
    FakePump pump;
    pump.alarm_broken = true;
    pump.paint(now);
    PlanNav nav(pump.scr, ROUTE, ROUTE_N);
    int emits = 0;
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    nav.set_emit([&] { emits++; });
    nav.set_interval_ms(60000);
    nav.enable(now);
    run_until(nav, now, [&] { return nav.cycles() == 1 && nav.idle(); });
    assert(nav.fails() == 1);
    assert(emits == 1);                                // only the status page
    assert(pump.page == FakePump::STATUS);             // recovery reached the anchor
    assert(nav.next_run_ms() == now + 2 * 60000);      // backoff doubled

    run_until(nav, now, [&] { return nav.cycles() == 2 && nav.idle(); });
    assert(nav.fails() == 2);
    assert(nav.next_run_ms() == now + 4 * 60000);      // and doubled again

    pump.alarm_broken = false;                          // device heals
    run_until(nav, now, [&] { return nav.cycles() == 3 && nav.idle(); });
    assert(nav.fails() == 0);                           // streak reset
    // full route on the good cycle: status, alarm, the D01 landing + the
    // walk budget's D views, D14
    assert(emits == 1 + 1 + (4 + WALK_DOWNS));
  }

  {  // PIN-gated step: the gate gets exactly the live-recorded 0815 key
     // sequence, the cycle verifies + emits the gated page, and the next
     // cycle passes through the remembered gate without a re-prompt
    uint32_t now = 1000;
    FakePump pump;
    pump.paint(now);
    PlanNav nav(pump.scr, PIN_ROUTE, PIN_ROUTE_N);
    std::vector<std::string> emitted;
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    nav.set_emit([&] {
      const char *rows[FIELDS_ROWS];
      for (size_t r = 0; r < FIELDS_ROWS; r++)
        rows[r] = pump.scr.row(SCR_TERM_ESP, r);
      char p[FIELDS_PAGE_MAX];
      page_of(rows, p);
      emitted.push_back(p);
    });
    nav.set_pin(1, 815);
    nav.set_interval_ms(60000);
    nav.enable(now);
    run_until(nav, now, [&] { return nav.cycles() == 1 && nav.idle(); });
    assert(nav.fails() == 0);
    assert(pump.pw1_passed);
    assert(emitted == std::vector<std::string>{"Gfc11"});
    assert(pump.page == FakePump::STATUS);  // ended back on the anchor
    std::vector<uint8_t> want = {KEY_ENTER};
    for (int i = 0; i < 8; i++) want.push_back(KEY_UP);
    want.push_back(KEY_ENTER);
    want.push_back(KEY_UP);
    want.push_back(KEY_ENTER);
    for (int i = 0; i < 5; i++) want.push_back(KEY_UP);
    want.push_back(KEY_ENTER);
    if (pump.gate_keys != want) {
      std::printf("FAIL PW1 keys:");
      for (uint8_t k : pump.gate_keys)
        std::printf(" %02X", k);
      std::printf("\n");
      return 1;
    }
    // second cycle: the device remembers the gate -> pass-through, the
    // gate sees no further keys, the page still emits
    run_until(nav, now, [&] { return nav.cycles() == 2 && nav.idle(); });
    assert(nav.fails() == 0);
    assert(pump.gate_keys == want);
    assert(emitted == (std::vector<std::string>{"Gfc11", "Gfc11"}));
  }

  {  // PIN-gated step with no gate at all (already passed before the route
     // ran): pass-through after the 1 s window, cycle green
    uint32_t now = 1000;
    FakePump pump;
    pump.pw1_passed = true;
    pump.paint(now);
    PlanNav nav(pump.scr, PIN_ROUTE, PIN_ROUTE_N);
    int emits = 0;
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    nav.set_emit([&] { emits++; });
    nav.set_pin(1, 815);
    nav.set_interval_ms(60000);
    nav.enable(now);
    run_until(nav, now, [&] { return nav.cycles() == 1 && nav.idle(); });
    assert(nav.fails() == 0);
    assert(emits == 1);
    assert(pump.gate_keys.empty());  // the gate never showed, nothing typed
    assert(pump.page == FakePump::STATUS);
  }

  {  // visible gate with an unconfigured pin: FAIL the cycle -- never type
     // a wrong PIN -- recover to the anchor and back off
    uint32_t now = 1000;
    FakePump pump;
    pump.paint(now);
    PlanNav nav(pump.scr, PIN_ROUTE, PIN_ROUTE_N);
    int emits = 0;
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    nav.set_emit([&] { emits++; });
    // no set_pin: PW1 unconfigured
    nav.set_interval_ms(60000);
    nav.enable(now);
    run_until(nav, now, [&] { return nav.cycles() == 1 && nav.idle(); });
    assert(nav.fails() == 1);
    assert(emits == 0);
    assert(!pump.pw1_passed);
    for (uint8_t k : pump.gate_keys)  // recovery Esc only; no digit typed
      assert(k == KEY_ESC);
    assert(pump.page == FakePump::STATUS);        // recovery reached the anchor
    assert(nav.next_run_ms() == now + 2 * 60000); // backoff doubled
  }

  {  // no cycle starts while not enrolled
    uint32_t now = 1000;
    FakePump pump;
    pump.paint(now);
    PlanNav nav(pump.scr, ROUTE, ROUTE_N);
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    nav.enable(now);
    for (int i = 0; i < 100; i++) {
      now += 50;
      nav.tick(now, false);
    }
    assert(nav.idle() && nav.cycles() == 0);
  }

  std::printf("ok\n");
  return 0;
}
