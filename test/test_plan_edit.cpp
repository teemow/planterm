// Host-compilable self-check for the edit engine (plan_edit.h) and the
// arbiter: a scripted fake heat pump implementing the live-verified edit
// model (Enter walks the page's editable fields, each hop commits the field
// it leaves unchanged, Up/Down step the focused value, Esc cancel-restores,
// the PW1 gate types digit by digit) driven purely through PlanEdit's
// tick(). The macro registry, spec table, menus, and routes here are
// consumer-style fixtures -- the same shape a real consumer supplies.
//
// Asserts the invariants ported from planscope macro_test.go: happy-path
// edit, focus hops, clamp rejection before any edit focus, off-grid
// overshoot abort, device-limit stall abort, enum full-cycle abort,
// bounded-enum reversal, torn multi-cell repaints (the settled change wait:
// the A01 ring's "ENERGY S." case, and a torn same-value repaint on a
// stalled press), PW1 entry key sequence, and gate pass-through.
// Plus the arbiter: a running scrape cycle finishes before a queued edit
// starts, FIFO drain with the scheduler held, scraping resumes after, and
// synchronous refusals (read-only set, full queue). Plus multi-op page
// visits (set_ops as ONE transactional visit -- selector + slot sub-fields,
// the copy pair only ever committed unchanged), their honest-partial abort
// semantics, and the non-committing selector sweep (read_sweep) against
// both cycling and bounded selectors.
//   c++ -std=c++17 test/test_plan_edit.cpp -o /tmp/t && /tmp/t

#include "../src/plan_edit.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

using namespace plan;

// --- device data fixtures (the consumer-side tables) -------------------------

static constexpr int MENU_N = 8;
static const char *const MENU_LABELS[MENU_N] = {
    "A.On/Off Unit", "B.Setpoint", "C.Clock/Scheduler", "D.Input/Output",
    "E.Data logger", "F.Information", "G.Service", "H.Manufacturer",
};
static const NavMenu MAIN_MENU{"Main menu", 8, MENU_LABELS, MENU_N};

static const char *const SVC_LABELS[7] = {
    "a.Change Language", "b.Information", "c.Expansion valve", "d.Working hours",
    "e.Comfort Settings", "f.Service settings", "g.Manual management",
};
static const NavMenu SVC_MENU{"Service menu", 7, SVC_LABELS, 7};

// The extractor spec table (what plan_fields.h calls device data).
static const FieldSpec SPECS[] = {
    // whole trimmed row, not FX_WORD: mode values carry inner spaces
    // ("ENERGY S.", live 2026-07-16) -- a word matcher goes blind on them
    {"A01", 4, FX_TEXT, false, "", "", "mode", ""},
    {"B01", 4, FX_LABEL_NUM, true, "Heating:", "\xDF", "heating_setpoint", "°C"},
    {"B01", 7, FX_LABEL_NUM, true, "Heating:", "\xDF", "heating_eco_setpoint", "°C"},
    {"B02", 4, FX_LABEL_NUM, true, "Domestic:", "\xDF", "dhw_setpoint", "°C"},
    {"B02", 7, FX_LABEL_NUM, true, "Domestic:", "\xDF", "dhw_eco_setpoint", "°C"},
    {"C01", 5, FX_LABEL_TOK, true, "Date:", "", "clock_date", ""},
    {"C02", 2, FX_LABEL_TOK, true, "Day ", "", "timer_day", ""},
    {"Gfc11", 4, FX_LABEL_TOK, true, "Setting:", "", "comfort_setting", ""},
    {"Gfc11", 6, FX_LABEL_NUM, true, "Set T.ext comp:", "\xDF", "comfort_ext_temp", "°C"},
    {"Gfc11", 7, FX_LABEL_NUM, true, "Compensation:", "%", "compensation", "%"},
    {"Gg01", 3, FX_LABEL_SEP_WORD, true, "NO2 Heatsourcepump", "", "manual_no2", ""},
    {"Gg01", 4, FX_LABEL_SEP_WORD, true, "NO3 Heating pump", "", "manual_no3", ""},
};
static constexpr size_t SPECS_N = sizeof(SPECS) / sizeof(SPECS[0]);

// Macro routes (every step verified against the screen).
static const MacroStep ROUTE_B01[] = {
    {KEY_PRG, 2, true, false, NEXP_MENU, 0, nullptr, &MAIN_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "B.Setpoint", &MAIN_MENU},
    {KEY_ENTER, 0, false, false, NEXP_PAGE_PREFIX, 0, "B0", nullptr},
    {KEY_DOWN, 3, true, false, NEXP_ROW_PREFIX, 4, "Heating:", nullptr},
};
static const MacroStep ROUTE_B02[] = {
    {KEY_PRG, 2, true, false, NEXP_MENU, 0, nullptr, &MAIN_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "B.Setpoint", &MAIN_MENU},
    {KEY_ENTER, 0, false, false, NEXP_PAGE_PREFIX, 0, "B0", nullptr},
    {KEY_DOWN, 3, true, false, NEXP_ROW_PREFIX, 4, "Domestic:", nullptr},
};
static const MacroStep ROUTE_A01[] = {
    {KEY_PRG, 2, true, false, NEXP_MENU, 0, nullptr, &MAIN_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "A.On/Off Unit", &MAIN_MENU},
    {KEY_ENTER, 0, false, false, NEXP_PAGE, 0, "A01", nullptr},
};
static const MacroStep ROUTE_COMFORT[] = {
    {KEY_PRG, 2, true, false, NEXP_MENU, 0, nullptr, &MAIN_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "G.Service", &MAIN_MENU},
    {KEY_ENTER, 0, false, false, NEXP_MENU, 0, nullptr, &SVC_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "e.Comfort Settings", &SVC_MENU},
    {KEY_ENTER, 0, false, true, NEXP_PAGE, 0, "Gfc11", nullptr},
};
static const MacroStep ROUTE_MANUAL[] = {
    {KEY_PRG, 2, true, false, NEXP_MENU, 0, nullptr, &MAIN_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "G.Service", &MAIN_MENU},
    {KEY_ENTER, 0, false, false, NEXP_MENU, 0, nullptr, &SVC_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "g.Manual management", &SVC_MENU},
    {KEY_ENTER, 0, false, false, NEXP_PAGE, 0, "Gg01", nullptr},
};
static const MacroStep ROUTE_CLOCK[] = {
    {KEY_PRG, 2, true, false, NEXP_MENU, 0, nullptr, &MAIN_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "C.Clock/Scheduler", &MAIN_MENU},
    {KEY_ENTER, 0, false, false, NEXP_PAGE_PREFIX, 0, "C0", nullptr},
    {KEY_DOWN, 3, true, false, NEXP_PAGE, 0, "C01", nullptr},
};
static const MacroStep ROUTE_WEEK_TIMER[] = {
    {KEY_PRG, 2, true, false, NEXP_MENU, 0, nullptr, &MAIN_MENU},
    {0, 0, false, false, NEXP_NONE, 0, "C.Clock/Scheduler", &MAIN_MENU},
    {KEY_ENTER, 0, false, false, NEXP_PAGE_PREFIX, 0, "C0", nullptr},
    {KEY_DOWN, 3, true, false, NEXP_PAGE, 0, "C02", nullptr},
};

#define ROUTE(r) r, static_cast<uint8_t>(sizeof(r) / sizeof((r)[0]))
static const MacroDef MACROS[] = {
    {"heating-setpoint", ROUTE(ROUTE_B01), "heating_setpoint", 0, 5, 45, false},
    {"dhw-setpoint", ROUTE(ROUTE_B02), "dhw_setpoint", 0, 10, 60, false},
    {"dhw-eco-setpoint", ROUTE(ROUTE_B02), "dhw_eco_setpoint", 1, 10, 60, false},
    {"mode", ROUTE(ROUTE_A01), "mode", 0, 0, 0, false},
    {"comfort-ext-temp", ROUTE(ROUTE_COMFORT), "comfort_ext_temp", 1, 5, 35, false},
    {"manual-no2", ROUTE(ROUTE_MANUAL), "manual_no2", 0, 0, 0, false},
    {"manual-no3", ROUTE(ROUTE_MANUAL), "manual_no3", 1, 0, 0, false},
    {"clock-date", ROUTE(ROUTE_CLOCK), "clock_date", 0, 0, 0, true},
    {"timer-day", ROUTE(ROUTE_WEEK_TIMER), "timer_day", 0, 0, 0, false},
};
#undef ROUTE

static const MacroDef *mfind(const char *name) {
  for (const MacroDef &m : MACROS)
    if (std::strcmp(m.name, name) == 0)
      return &m;
  return nullptr;
}

// Week-scheduler (C02) fixtures: the day/state vocabularies as displayed,
// and the slot sub-field read-back specs. The Enter walk covers 15 fields:
// 1 = Day (selects which program shows), 2 = Copy-in day, 3 = Copy-confirm
// NO/YES, then F1..F4 as hh -> mi -> state triplets (fields 4..15).
static const char *const SCHED_DAYS[7] = {
    "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY", "SUNDAY",
};
static const char *const SCHED_STATES[3] = {"OFF", "ON", "ENERGY SAVE"};

static const FieldSpec SCHED_SLOTS[4][3] = {
    {{"C02", 4, FX_TIMER_HH, true, "F1 ", "", "timer_f1_hh", ""},
     {"C02", 4, FX_TIMER_MI, true, "F1 ", "", "timer_f1_mi", ""},
     {"C02", 4, FX_TIMER_STATE, true, "F1 ", "", "timer_f1_state", ""}},
    {{"C02", 5, FX_TIMER_HH, true, "F2 ", "", "timer_f2_hh", ""},
     {"C02", 5, FX_TIMER_MI, true, "F2 ", "", "timer_f2_mi", ""},
     {"C02", 5, FX_TIMER_STATE, true, "F2 ", "", "timer_f2_state", ""}},
    {{"C02", 6, FX_TIMER_HH, true, "F3 ", "", "timer_f3_hh", ""},
     {"C02", 6, FX_TIMER_MI, true, "F3 ", "", "timer_f3_mi", ""},
     {"C02", 6, FX_TIMER_STATE, true, "F3 ", "", "timer_f3_state", ""}},
    {{"C02", 7, FX_TIMER_HH, true, "F4 ", "", "timer_f4_hh", ""},
     {"C02", 7, FX_TIMER_MI, true, "F4 ", "", "timer_f4_mi", ""},
     {"C02", 7, FX_TIMER_STATE, true, "F4 ", "", "timer_f4_state", ""}},
};

// The consumer-side op builder for a schedule write: ONE C02 page visit --
// select the day program (field 1), hop the copy pair unchanged, then edit
// the slot's hh -> mi -> state sub-fields (fields 4 + 3*(slot-1) ..).
static void sched_ops(const char *day, int slot, int hh, int mi, const char *state,
                      PlanEdit::EditOp ops[4]) {
  ops[0].spec = find_spec(SPECS, SPECS_N, "timer_day");
  ops[0].field = 1;
  std::snprintf(ops[0].target, sizeof ops[0].target, "%s", day);
  uint8_t f = static_cast<uint8_t>(4 + 3 * (slot - 1));
  for (int c = 0; c < 3; c++) {
    ops[1 + c].spec = &SCHED_SLOTS[slot - 1][c];
    ops[1 + c].field = static_cast<uint8_t>(f + c);
  }
  std::snprintf(ops[1].target, sizeof ops[1].target, "%02d", hh);
  std::snprintf(ops[2].target, sizeof ops[2].target, "%02d", mi);
  std::snprintf(ops[3].target, sizeof ops[3].target, "%s", state);
}

// --- fake device --------------------------------------------------------------

// PlanEdit only READS a PlanScreen; the fake paints it directly through the
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

// The fake device: pages + the live-verified edit model, repainted
// instantly on a key press (the 600 ms settle wait still runs).
struct FakePump {
  TestScreen scr;
  enum P { STATUS, MENU, A01P, B_PAGE, C_PAGE, SVC_MENU_P, PW_GATE, GFC11, GG01 } page = STATUS;
  int menu_cur = 5;  // last-used main-menu cursor (1-based), like the real device
  int svc_cur = 3;   // Service-menu cursor (1-based)
  int b_sub = 0;     // 0 = B01 Heating, 1 = B02 Domestic (last-used, wraps)
  int c_sub = 0;     // 0 = C01 clock, 1 = C02 week timer, 2 = C03 (wraps)

  // C02 week timer (the live-verified 15-field Enter walk: day selector,
  // copy-in day, copy-confirm, then F1..F4 as hh/mi/state triplets)
  struct Slot {
    double hh, mi;
    int st;  // index into SCHED_STATES (OFF, ON, ENERGY SAVE)
  };
  Slot week[7][4];
  int day_idx = 0;          // displayed program (the field-1 selector)
  int copy_idx = 0;         // copy-in day
  int confirm_idx = 0;      // 0 = NO, 1 = YES
  bool day_bounded = false; // day selector: bounded list vs cycling
  int copies = 0;           // executed copy operations (must stay 0!)
  double mi_step = 1;       // minute step (live: 1); tests set 10 for off-grid
  const char *yn[2] = {"NO", "YES"};

  // numeric fields: B pages [sub][nominal, eco] step 0.5; Gfc11 ext temp
  // (field 2, step 0.5) + compensation (field 3, step 1)
  double bvals[2][2] = {{10.0, 10.0}, {39.0, 20.0}};
  double dev_hi = 1000;  // device limit: Up sticks here (stall)
  double comfort_ext = 19.0;
  double compensation = 50;
  // enum fields. A01 defaults to the historical bounded 3-option fixture;
  // the ring tests switch it to the live-verified 4-option ring
  // (AUTO -> OFF -> ON -> ENERGY S. -> AUTO, 2026-07-16).
  static constexpr int A01_MAX = 4;
  const char *a01_opts[A01_MAX] = {"AUTO", "OFF", "ON", "ENERGY S."};
  int a01_n = 3;
  bool a01_bounded = true;  // 3-option fixture sticks at the ends
  int a01_idx = 0;
  static constexpr int GG_N = 2;
  const char *gg_opts[GG_N] = {"AUT", "MAN"};  // cycling
  int gg_idx[4] = {0, 0, 0, 0};
  const char *comfort_opts[2] = {"DYNAMIC", "FIXED"};  // cycling
  int comfort_idx = 0;

  // torn repaint model (0 = atomic paints, the historical fixture): an A01
  // value change paints cell-by-cell like the live device -- the row first
  // shows the new value's first two cells over the OLD text's tail, and
  // completes tear_ms later (tick() applies it). tear_flash additionally
  // tears a same-value repaint on a stalled press (the settle-back-to-cur_
  // case the engine must re-arm on, not treat as a change).
  int tear_ms = 0;
  bool tear_flash = false;
  bool tear_pending = false;
  uint32_t tear_due = 0;
  char a01_torn[SCR_COLS + 1] = "";
  bool a01_torn_active = false;

  // edit focus: 0 = navigation, 1..N = field focused (per-page field order)
  int focus = 0;
  double edit_num_start = 0;
  int edit_idx_start = 0;
  bool committed = false;  // any field left with a changed value

  // PW1 gate
  bool pw1_passed = false;
  int gate_val = 0, gate_enters = 0, gate_stage = 0;
  P gate_target = GFC11;
  std::vector<uint8_t> gate_keys;  // every key the gate saw

  FakePump() {
    for (int d = 0; d < 7; d++) {  // per-day distinct F1 (the sweep asserts it)
      week[d][0] = {6 + (double) d, 10, 1};
      week[d][1] = {8, 30, 0};
      week[d][2] = {16, 0, 0};
      week[d][3] = {22, 0, 0};
    }
  }

  bool on_c02() const { return page == C_PAGE && c_sub == 1; }

  int fields_on_page() const {
    switch (page) {
      case A01P: return 1;
      case B_PAGE: return 2;
      case C_PAGE: return c_sub == 1 ? 15 : 0;
      case GFC11: return 3;
      case GG01: return 4;
      default: return 0;
    }
  }
  double *num_field(int f) {
    if (page == B_PAGE) return &bvals[b_sub][f - 1];
    if (page == GFC11 && f == 2) return &comfort_ext;
    if (page == GFC11 && f == 3) return &compensation;
    if (on_c02() && f >= 4 && (f - 4) % 3 != 2) {
      Slot &s = week[day_idx][(f - 4) / 3];
      return (f - 4) % 3 == 0 ? &s.hh : &s.mi;
    }
    return nullptr;
  }
  int *enum_field(int f, int *n, const char *const **opts, bool *bounded) {
    if (page == A01P) { *n = a01_n; *opts = a01_opts; *bounded = a01_bounded; return &a01_idx; }
    if (page == GFC11 && f == 1) { *n = 2; *opts = comfort_opts; *bounded = false; return &comfort_idx; }
    if (page == GG01) { *n = GG_N; *opts = gg_opts; *bounded = false; return &gg_idx[f - 1]; }
    if (on_c02()) {
      if (f == 1) { *n = 7; *opts = SCHED_DAYS; *bounded = day_bounded; return &day_idx; }
      if (f == 2) { *n = 7; *opts = SCHED_DAYS; *bounded = false; return &copy_idx; }
      if (f == 3) { *n = 2; *opts = yn; *bounded = false; return &confirm_idx; }
      if ((f - 4) % 3 == 2) { *n = 3; *opts = SCHED_STATES; *bounded = false;
                              return &week[day_idx][(f - 4) / 3].st; }
    }
    return nullptr;
  }

  void snap_focus() {
    if (double *v = num_field(focus)) edit_num_start = *v;
    int n; const char *const *o; bool b;
    if (int *i = enum_field(focus, &n, &o, &b)) edit_idx_start = *i;
  }
  void leave_focus(bool commit) {
    if (double *v = num_field(focus)) {
      if (commit) committed = committed || *v != edit_num_start;
      else *v = edit_num_start;
    }
    int n; const char *const *o; bool b;
    if (int *i = enum_field(focus, &n, &o, &b)) {
      if (commit) committed = committed || *i != edit_idx_start;
      else *i = edit_idx_start;
    }
  }
  void step_field(int dir) {
    if (double *v = num_field(focus)) {
      if (on_c02()) {  // hh/mi: step 1 (mi_step for minutes), bounded
        bool mi = (focus - 4) % 3 == 1;
        double next = *v + dir * (mi ? mi_step : 1);
        if (next >= 0 && next <= (mi ? 59 : 23)) *v = next;
        return;
      }
      double step = (page == GFC11 && focus == 3) ? 1.0 : 0.5;
      double next = *v + dir * step;
      if (next <= dev_hi) *v = next;
      return;
    }
    int n; const char *const *o; bool bounded;
    if (int *i = enum_field(focus, &n, &o, &bounded)) {
      if (bounded) {
        int next = *i + dir;
        if (next >= 0 && next < n) *i = next;
      } else {
        *i = (*i + n + dir) % n;
      }
    }
  }

  void gate_key(uint8_t k) {
    gate_keys.push_back(k);
    if (k == KEY_ESC) { page = SVC_MENU_P; return; }
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
      if (gate_val == 815) { pw1_passed = true; page = gate_target; }
      else { gate_val = 0; gate_enters = 0; gate_stage = 0; }
    }
  }

  // Complete a pending torn repaint once its cell updates "arrived".
  void tick(uint32_t now) {
    if (tear_pending && now >= tear_due) {
      tear_pending = false;
      a01_torn_active = false;
      paint(now);
    }
  }

  void press(uint8_t k, uint32_t now) {
    if (page == PW_GATE) { gate_key(k); paint(now); return; }
    const char *a01_before = a01_opts[a01_idx];
    bool a01_edit = page == A01P && focus == 1 && (k == KEY_UP || k == KEY_DOWN);
    switch (k) {
      case KEY_ESC:
        if (focus > 0) { leave_focus(false); focus = 0; }
        else if (page == MENU) page = STATUS;
        else if (page == A01P || page == B_PAGE || page == C_PAGE) page = MENU;
        else if (page == SVC_MENU_P) page = MENU;
        else if (page == GFC11 || page == GG01) page = SVC_MENU_P;
        break;
      case KEY_PRG:
        if (page == STATUS) page = MENU;  // lands at the LAST-USED cursor
        break;
      case KEY_DOWN:
        if (focus > 0) step_field(-1);
        else if (page == MENU && menu_cur < MENU_N) menu_cur++;
        else if (page == SVC_MENU_P && svc_cur < 7) svc_cur++;
        else if (page == B_PAGE) b_sub = 1 - b_sub;  // the 2-page set wraps
        else if (page == C_PAGE) c_sub = (c_sub + 1) % 3;  // the 3-page set wraps
        break;
      case KEY_UP:
        if (focus > 0) step_field(1);
        else if (page == MENU && menu_cur > 1) menu_cur--;
        else if (page == SVC_MENU_P && svc_cur > 1) svc_cur--;
        else if (page == B_PAGE) b_sub = 1 - b_sub;
        else if (page == C_PAGE) c_sub = (c_sub + 2) % 3;
        break;
      case KEY_ENTER:
        if (focus > 0) {  // commit + advance (the hop rule); exits after the last
          leave_focus(true);
          // a committed copy-confirm YES executes the day copy (direction
          // is irrelevant to the tests: copies must stay 0)
          if (on_c02() && focus == 3 && confirm_idx == 1) {
            for (int s = 0; s < 4; s++) week[day_idx][s] = week[copy_idx][s];
            copies++;
            confirm_idx = 0;
          }
          focus = focus < fields_on_page() ? focus + 1 : 0;
          if (focus > 0) snap_focus();
        } else if (page == MENU && menu_cur == 1) {
          page = A01P; focus = 0;
        } else if (page == MENU && menu_cur == 2) {
          page = B_PAGE;  // lands on the LAST-USED sub-page
        } else if (page == MENU && menu_cur == 3) {
          page = C_PAGE;  // lands on the LAST-USED C sub-page
        } else if (page == MENU && menu_cur == 7) {
          page = SVC_MENU_P;
        } else if (page == SVC_MENU_P && svc_cur == 5) {  // e.Comfort Settings (PW1)
          if (pw1_passed) { page = GFC11; }
          else { page = PW_GATE; gate_target = GFC11; gate_val = 0; gate_enters = 0; }
        } else if (page == SVC_MENU_P && svc_cur == 7) {  // g.Manual management
          page = GG01;
        } else if (page == A01P || page == B_PAGE || page == GFC11 || page == GG01 || on_c02()) {
          focus = 1;  // first Enter focuses the first editable field
          snap_focus();
        }
        break;
    }
    if (tear_ms > 0 && a01_edit) {
      const char *a01_after = a01_opts[a01_idx];
      bool changed = std::strcmp(a01_after, a01_before) != 0;
      if (changed) {
        // first two cells of the new text land over the old text's tail;
        // the completion (tick) paints the final row tear_ms later
        char oldp[SCR_COLS + 1], newp[SCR_COLS + 1];
        std::snprintf(oldp, sizeof oldp, "%-22.22s", a01_before);
        std::snprintf(newp, sizeof newp, "%-22.22s", a01_after);
        std::memcpy(oldp, newp, 2);
        std::snprintf(a01_torn, sizeof a01_torn, "%s", oldp);
        a01_torn_active = true;
        tear_pending = true;
        tear_due = now + static_cast<uint32_t>(tear_ms);
      } else if (tear_flash) {
        // stalled press, same-value repaint torn as a blank-tail refresh:
        // the row briefly shows only the first two cells ("EN" out of
        // "ENERGY S."), then completes back to the unchanged value -- the
        // settle-back-to-cur_ glimpse the engine must NOT take for a change
        std::snprintf(a01_torn, sizeof a01_torn, "%.2s", a01_after);
        a01_torn_active = true;
        tear_pending = true;
        tear_due = now + static_cast<uint32_t>(tear_ms);
      }
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
      case MENU:
        std::snprintf(buf, sizeof buf, "Main menu          %d/8", menu_cur);
        scr.put_row(SCR_TERM_ESP, 0, buf, now);
        scr.put_row(SCR_TERM_ESP, 3, MENU_LABELS[menu_cur - 1], now);
        scr.select_row(SCR_TERM_ESP, 3);
        break;
      case A01P:
        scr.put_row(SCR_TERM_ESP, 0, " On/Off Unit       A01", now);
        scr.put_row(SCR_TERM_ESP, 4, a01_torn_active ? a01_torn : a01_opts[a01_idx], now);
        break;
      case B_PAGE:
        std::snprintf(buf, sizeof buf, " Thermoreg. Unit   B%02d", b_sub + 1);
        scr.put_row(SCR_TERM_ESP, 0, buf, now);
        for (int f = 0; f < 2; f++) {
          std::snprintf(buf, sizeof buf, "%-9s%11.1f\xDF" "C",
                        b_sub == 0 ? "Heating:" : "Domestic:", bvals[b_sub][f]);
          scr.put_row(SCR_TERM_ESP, f == 0 ? 4 : 7, buf, now);
        }
        break;
      case C_PAGE:
        if (c_sub != 1) {  // C01/C03: only the page ID matters (route seek)
          std::snprintf(buf, sizeof buf, " Clock             C%02d", c_sub == 0 ? 1 : 3);
          scr.put_row(SCR_TERM_ESP, 0, buf, now);
          break;
        }
        scr.put_row(SCR_TERM_ESP, 0, " Week Timer        C02", now);
        scr.put_row(SCR_TERM_ESP, 1, " Heatprogram", now);
        std::snprintf(buf, sizeof buf, "Day     %s", SCHED_DAYS[day_idx]);
        scr.put_row(SCR_TERM_ESP, 2, buf, now);
        std::snprintf(buf, sizeof buf, "Copy in %s      %s", SCHED_DAYS[copy_idx],
                      yn[confirm_idx]);
        scr.put_row(SCR_TERM_ESP, 3, buf, now);
        for (int s = 0; s < 4; s++) {
          const Slot &sl = week[day_idx][s];
          std::snprintf(buf, sizeof buf, "F%d %02.0f:%02.0f   %s", s + 1, sl.hh, sl.mi,
                        SCHED_STATES[sl.st]);
          scr.put_row(SCR_TERM_ESP, 4 + s, buf, now);
        }
        break;
      case SVC_MENU_P:
        std::snprintf(buf, sizeof buf, "Service menu       %d/7", svc_cur);
        scr.put_row(SCR_TERM_ESP, 0, buf, now);
        scr.put_row(SCR_TERM_ESP, 2, SVC_LABELS[svc_cur - 1], now);
        scr.select_row(SCR_TERM_ESP, 2);
        break;
      case PW_GATE:
        scr.put_row(SCR_TERM_ESP, 0, "Service Password", now);
        std::snprintf(buf, sizeof buf, "password (PW1):   %04d", gate_val);
        scr.put_row(SCR_TERM_ESP, 5, buf, now);
        break;
      case GFC11:
        scr.put_row(SCR_TERM_ESP, 0, " Termoreg.       Gfc11", now);
        std::snprintf(buf, sizeof buf, "Setting:       %s", comfort_opts[comfort_idx]);
        scr.put_row(SCR_TERM_ESP, 4, buf, now);
        std::snprintf(buf, sizeof buf, "Set T.ext comp: %.1f\xDF" "C", comfort_ext);
        scr.put_row(SCR_TERM_ESP, 6, buf, now);
        std::snprintf(buf, sizeof buf, "Compensation:     %.0f%%", compensation);
        scr.put_row(SCR_TERM_ESP, 7, buf, now);
        break;
      case GG01: {
        scr.put_row(SCR_TERM_ESP, 0, " Manual mng.      Gg01", now);
        const char *labels[4] = {"NO2 Heatsourcepump", "NO3 Heating pump  ",
                                 "NO4 DHW circ.pump ", "NO8 Heatpump 2    "};
        const int rows[4] = {3, 4, 6, 7};
        for (int f = 0; f < 4; f++) {
          std::snprintf(buf, sizeof buf, "%s:%s", labels[f], gg_opts[gg_idx[f]]);
          scr.put_row(SCR_TERM_ESP, rows[f], buf, now);
        }
        break;
      }
    }
  }
};

// One captured selector value's program, composed from the F rows exactly
// like a consumer would in its sweep_emit hook (space-squeezed and joined).
static std::string compose_program(const TestScreen &scr) {
  std::string prog;
  for (int r = 4; r <= 7; r++) {
    const char *row = scr.row(SCR_TERM_ESP, r);
    if (!(row[0] == 'F' && row[1] >= '0' && row[1] <= '9'))
      continue;
    if (!prog.empty())
      prog += " | ";
    bool gap = false;
    for (const char *p = row; *p != '\0'; p++) {
      if (*p == ' ') {
        gap = true;
        continue;
      }
      if (gap && !prog.empty() && prog.back() != ' ')
        prog += ' ';
      prog += *p;
      gap = false;
    }
  }
  while (!prog.empty() && prog.back() == ' ')
    prog.pop_back();
  return prog;
}

// One PlanEdit request against a fresh-ish pump, run to completion.
struct Result {
  bool called = false, ok = false;
  std::string value, msg;
};

static Result run(FakePump &pump, uint32_t &now, const char *macro, const char *target) {
  PlanEdit edit(pump.scr);
  edit.set_specs(SPECS, SPECS_N);
  edit.set_pin(815);
  edit.set_press([&](uint8_t k) { pump.press(k, now); });
  Result r;
  edit.set_done([&](bool ok, const char *value, const char *msg) {
    r.called = true;
    r.ok = ok;
    r.value = value;
    r.msg = msg;
  });
  const MacroDef *def = mfind(macro);
  assert(def != nullptr);
  bool started = target != nullptr ? edit.set(def, target) : edit.get(def);
  assert(started);
  for (int i = 0; i < 100000 && !r.called; i++) {
    now += 50;
    pump.tick(now);
    edit.tick(now);
  }
  assert(r.called);
  assert(edit.idle());
  return r;
}

// One multi-op schedule write, run to completion.
static Result run_sched(FakePump &pump, uint32_t &now, const char *day, int slot, int hh, int mi,
                        const char *state) {
  PlanEdit edit(pump.scr);
  edit.set_specs(SPECS, SPECS_N);
  edit.set_press([&](uint8_t k) { pump.press(k, now); });
  Result r;
  edit.set_done([&](bool ok, const char *value, const char *msg) {
    r.called = true;
    r.ok = ok;
    r.value = value;
    r.msg = msg;
  });
  PlanEdit::EditOp ops[4];
  sched_ops(day, slot, hh, mi, state, ops);
  assert(edit.set_ops(mfind("timer-day"), ops, 4, "schedule"));
  for (int i = 0; i < 100000 && !r.called; i++) {
    now += 50;
    pump.tick(now);
    edit.tick(now);
  }
  assert(r.called && edit.idle());
  return r;
}

// One read_sweep day walk; returns the emitted (day, program) pairs.
static std::vector<std::pair<std::string, std::string>> run_sweep(FakePump &pump, uint32_t &now,
                                                                  bool *ok_out) {
  PlanEdit edit(pump.scr);
  edit.set_specs(SPECS, SPECS_N);
  edit.set_press([&](uint8_t k) { pump.press(k, now); });
  std::vector<std::pair<std::string, std::string>> days;
  edit.set_sweep_emit([&](const char *day) {
    // the screen shows this day's page state right now -- compose here,
    // exactly what a consumer's hook does
    days.emplace_back(day, compose_program(pump.scr));
  });
  bool called = false;
  edit.set_done([&](bool ok, const char *, const char *) {
    called = true;
    *ok_out = ok;
  });
  assert(edit.read_sweep(mfind("timer-day"), 7, "schedule"));
  for (int i = 0; i < 100000 && !called; i++) {
    now += 50;
    pump.tick(now);
    edit.tick(now);
  }
  assert(called && edit.idle());
  return days;
}

// A minimal scrape route for the arbiter tests: the fake has no alarms
// page, so a started cycle fails through its verify timeout and recovers --
// a running cycle either way.
static const ScrapeStep SCRAPE_ROUTE[] = {
    {0, 0, false, NEXP_NONE, 0, nullptr, nullptr, true, 0},
    {KEY_ALARM, 0, false, NEXP_ROW_PREFIX, 0, "Alarms", nullptr, true, 0},
};
static constexpr size_t SCRAPE_ROUTE_N = sizeof(SCRAPE_ROUTE) / sizeof(SCRAPE_ROUTE[0]);

int main() {
  {  // helpers
    assert(edit_eq("40", "40.0") && edit_eq(" AUTO ", "AUTO") && !edit_eq("39.0", "39.5"));
    assert(edit_pw_gate("Service Password") && edit_pw_gate("Manufacturer password"));
    assert(!edit_pw_gate("02:18 03/07/26"));
    assert(edit_pw_shows("password (PW1):   0800", 800));
    assert(!edit_pw_shows("password (PW1):   0800", 810));
  }

  {  // happy-path numeric edit: navigate (with B-sub-page seek), edit, commit
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "dhw-setpoint", "40.0");
    assert(r.ok && r.value == "40.0");  // done reports the commit read-back
    assert(pump.committed && pump.bvals[1][0] == 40.0);
    assert(pump.page == FakePump::STATUS);  // teardown Esc'd back to the anchor
  }

  {  // focus hop: field 2 edited, field 1 committed UNCHANGED on the way in
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "dhw-eco-setpoint", "21.0");
    assert(r.ok && r.value == "21.0");
    assert(pump.bvals[1][0] == 39.0 && pump.bvals[1][1] == 21.0);
  }

  {  // get: read-only navigation + read, no edit focus ever opened
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "dhw-setpoint", nullptr);
    assert(r.ok && r.value == "39.0");
    assert(!pump.committed && pump.page == FakePump::STATUS);
  }

  {  // clamp: rejected after the read, BEFORE any edit focus opens
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "dhw-setpoint", "80");
    assert(!r.ok && r.msg.find("clamp") != std::string::npos);
    assert(!pump.committed && pump.bvals[1][0] == 39.0);
    // numeric/enum mismatch refused the same way
    r = run(pump, now, "dhw-setpoint", "AUTO");
    assert(!r.ok && r.msg.find("mismatch") != std::string::npos);
    assert(!pump.committed);
  }

  {  // off-grid target: stepped past -> Esc abort, value restored, no commit
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "dhw-setpoint", "39.3");
    assert(!r.ok && r.msg.find("stepped past") != std::string::npos);
    assert(!pump.committed && pump.bvals[1][0] == 39.0);
    assert(pump.page == FakePump::STATUS);
  }

  {  // device-limit stall: value sticks below the target -> Esc abort
    FakePump pump;
    pump.dev_hi = 41.0;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "dhw-setpoint", "45");
    assert(!r.ok && r.msg.find("stuck") != std::string::npos);
    assert(!pump.committed && pump.bvals[1][0] == 39.0);
  }

  {  // cycling enum without the target: full-cycle detection -> Esc abort
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "manual-no2", "OFF");
    assert(!r.ok && r.msg.find("cycled") != std::string::npos);
    assert(!pump.committed && pump.gg_idx[0] == 0);
  }

  {  // cycling enum happy path on a hopped field (manual-no3 = field 2)
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "manual-no3", "MAN");
    assert(r.ok && r.value == "MAN");
    assert(pump.committed && pump.gg_idx[1] == 1 && pump.gg_idx[0] == 0);
  }

  {  // bounded enum: stalls at the top, reverses ONCE, finds the target below
    FakePump pump;
    pump.a01_idx = 1;  // OFF; target AUTO sits below the start
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "mode", "AUTO");
    assert(r.ok && r.value == "AUTO");
    assert(pump.committed && pump.a01_idx == 0);
  }

  {  // the live A01 ring with torn repaints (2026-07-23 "set mode from ON"
     // failure): ON -> AUTO crosses the multi-word "ENERGY S.", whose
     // cell-by-cell repaint first shows torn text ("EN" over the old row) --
     // the settled wait must ignore the tear and land EXACTLY on the target
    FakePump pump;
    pump.a01_n = 4;
    pump.a01_bounded = false;  // AUTO -> OFF -> ON -> ENERGY S. -> AUTO
    pump.a01_idx = 2;          // ON
    pump.tear_ms = 200;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "mode", "AUTO");
    assert(r.ok && r.value == "AUTO");
    assert(pump.committed && pump.a01_idx == 0);
    assert(pump.page == FakePump::STATUS);
  }

  {  // torn same-value repaint on a stalled press (bounded list top): the
     // glimpse settles BACK to the current value -- no change, so the enum
     // reversal must still fire and find the target below, never treating
     // the tear as a step
    FakePump pump;
    pump.a01_n = 4;
    pump.a01_bounded = true;
    pump.a01_idx = 3;  // ENERGY S., the bounded top
    pump.tear_ms = 200;
    pump.tear_flash = true;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "mode", "ON");
    assert(r.ok && r.value == "ON");
    assert(pump.committed && pump.a01_idx == 2);
  }

  {  // PW1: the gate gets exactly the live-recorded 0815 key sequence, then
     // the same session passes through without a re-prompt
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "comfort-ext-temp", "20.0");
    assert(r.ok && r.value == "20.0");
    assert(pump.comfort_ext == 20.0 && pump.comfort_idx == 0);  // Setting hop untouched
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
    assert(pump.pw1_passed);
    // second edit: gate remembered, no re-prompt (gate_keys unchanged)
    r = run(pump, now, "comfort-ext-temp", "19.0");
    assert(r.ok && pump.comfort_ext == 19.0);
    assert(pump.gate_keys == want);
  }

  {  // read-only macros refuse set
    FakePump pump;
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    assert(!edit.set(mfind("clock-date"), "01/01/26"));
    // but get is allowed (starts; not run here)
  }

  {  // already at the target: success without opening an edit focus
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run(pump, now, "dhw-setpoint", "39.0");
    assert(r.ok && r.value == "39.0");
    assert(!pump.committed);
  }

  {  // set_ops boundary validation: no route, zero ops, too many ops, a
     // nullptr spec -- all refused synchronously
    FakePump pump;
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    PlanEdit::EditOp ops[5];
    sched_ops("MONDAY", 1, 6, 0, "ON", ops);
    assert(!edit.set_ops(nullptr, ops, 4, "schedule"));
    assert(!edit.set_ops(mfind("timer-day"), ops, 0, "schedule"));
    assert(!edit.set_ops(mfind("timer-day"), ops, 5, "schedule"));
    ops[2].spec = nullptr;
    assert(!edit.set_ops(mfind("timer-day"), ops, 4, "schedule"));
    assert(edit.idle());
  }

  {  // multi-op happy path: ONE C02 visit -- select WEDNESDAY, hop the
     // copy pair + F1 unchanged, edit F2 hh/mi/state; nothing else moves
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    FakePump::Slot before[7][4];
    std::memcpy(before, pump.week, sizeof before);
    Result r = run_sched(pump, now, "WEDNESDAY", 2, 7, 45, "ENERGY SAVE");
    assert(r.ok && r.value == "ENERGY SAVE");  // the last commit read-back
    assert(pump.day_idx == 2);                 // day selector committed
    assert(pump.week[2][1].hh == 7 && pump.week[2][1].mi == 45 && pump.week[2][1].st == 2);
    for (int d = 0; d < 7; d++)  // every other slot of every day untouched
      for (int s = 0; s < 4; s++) {
        if (d == 2 && s == 1)
          continue;
        assert(pump.week[d][s].hh == before[d][s].hh && pump.week[d][s].mi == before[d][s].mi &&
               pump.week[d][s].st == before[d][s].st);
      }
    assert(pump.copies == 0 && pump.confirm_idx == 0);  // NO confirm never copied
    assert(pump.page == FakePump::STATUS);
  }

  {  // multi-op on the displayed day, last slot: the day op commits
     // unchanged, the walk crosses F1-F3 unchanged, mi is already equal
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run_sched(pump, now, "MONDAY", 4, 23, 0, "ON");
    assert(r.ok);
    assert(pump.day_idx == 0);
    assert(pump.week[0][3].hh == 23 && pump.week[0][3].mi == 0 && pump.week[0][3].st == 1);
    assert(pump.copies == 0 && pump.page == FakePump::STATUS);
  }

  {  // multi-op abort mid-transaction: minutes off the device's step grid
     // -> Esc abort restores the FOCUSED field, but the sub-fields
     // committed before it stay (honest partial; the failure says so)
    FakePump pump;
    pump.mi_step = 10;
    uint32_t now = 1000;
    pump.paint(now);
    Result r = run_sched(pump, now, "TUESDAY", 1, 5, 35, "ON");
    assert(!r.ok && r.msg.find("stepped past") != std::string::npos);
    assert(pump.day_idx == 1);           // day committed before the abort
    assert(pump.week[1][0].hh == 5);     // hh committed before the abort
    assert(pump.week[1][0].mi == 10);    // the aborted mi was Esc-restored
    assert(pump.week[1][0].st == 1);     // state never reached
    assert(pump.copies == 0 && pump.page == FakePump::STATUS);
  }

  {  // read_sweep, cycling day selector: all 7 programs emitted in Up
     // order, NOTHING committed, the selector Esc-restored
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    bool ok = false;
    auto days = run_sweep(pump, now, &ok);
    assert(ok && days.size() == 7);
    for (int d = 0; d < 7; d++) {
      char want[100];
      std::snprintf(want, sizeof want,
                    "F1 %02d:10 ON | F2 08:30 OFF | F3 16:00 OFF | F4 22:00 OFF", 6 + d);
      assert(days[d].first == SCHED_DAYS[d]);
      assert(days[d].second == want);
    }
    assert(!pump.committed && pump.copies == 0);
    assert(pump.day_idx == 0);  // Esc cancel-restored the selector
    assert(pump.page == FakePump::STATUS);
  }

  {  // read_sweep, BOUNDED day selector, entered mid-list: Up to the
     // top, turn around ONCE, Down collects the rest -- still all 7
    FakePump pump;
    pump.day_bounded = true;
    pump.day_idx = 3;  // THURSDAY
    uint32_t now = 1000;
    pump.paint(now);
    bool ok = false;
    auto days = run_sweep(pump, now, &ok);
    assert(ok && days.size() == 7);
    std::vector<std::string> seen;
    for (auto &d : days)
      seen.push_back(d.first);
    std::vector<std::string> want = {"THURSDAY", "FRIDAY",  "SATURDAY", "SUNDAY",
                                     "WEDNESDAY", "TUESDAY", "MONDAY"};
    assert(seen == want);
    assert(!pump.committed && pump.day_idx == 3);
    assert(pump.page == FakePump::STATUS);
  }

  {  // page-visit emit: a request fires the page-settled emit exactly once,
     // on the target page, before any edit focus opens -- the hook an owner
     // uses to force-publish the whole page per visit
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    edit.set_press([&](uint8_t k) { pump.press(k, now); });
    int emits = 0;
    std::string page_at_emit;
    edit.set_emit([&] {
      emits++;
      const char *rows[FIELDS_ROWS];
      for (size_t r = 0; r < FIELDS_ROWS; r++)
        rows[r] = pump.scr.row(SCR_TERM_ESP, r);
      char p[FIELDS_PAGE_MAX];
      page_of(rows, p);
      page_at_emit = p;
    });
    bool called = false;
    edit.set_done([&](bool ok, const char *, const char *) {
      called = true;
      assert(ok);
    });
    assert(edit.get(mfind("dhw-setpoint")));
    for (int i = 0; i < 100000 && !called; i++) {
      now += 50;
      edit.tick(now);
    }
    assert(called && emits == 1 && page_at_emit == "B02");
    assert(!pump.committed);  // a visit never opens an edit focus
  }

  {  // arbiter: sets jump ahead of queued gets (scheduled page reads must
     // not delay an entity write); gets keep FIFO among themselves
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    PlanNav nav(pump.scr, SCRAPE_ROUTE, SCRAPE_ROUTE_N);
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    EditArbiter arb(edit, nav);
    edit.set_press([&](uint8_t k) { pump.press(k, now); });
    std::vector<std::string> done;
    edit.set_done([&](bool ok, const char *value, const char *) {
      done.push_back(std::string(arb.running()->name) + "=" + (ok ? value : "FAIL"));
    });
    assert(arb.enqueue(mfind("dhw-eco-setpoint"), nullptr));
    assert(arb.enqueue(mfind("heating-setpoint"), nullptr));
    assert(arb.enqueue(mfind("dhw-setpoint"), "40.0"));  // enqueued last
    for (int i = 0; i < 100000 && done.size() < 3; i++) {
      now += 50;
      arb.tick(now, true);
    }
    assert(done.size() == 3);
    assert(done[0] == "dhw-setpoint=40.0");       // the set ran first
    assert(done[1] == "dhw-eco-setpoint=20.0");   // then the gets, in order
    assert(done[2] == "heating-setpoint=10.0");
  }

  {  // arbiter: a multi-op request is a set (jumps queued gets), the
     // sweep a get behind them; running_kind() tells them apart
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    PlanNav nav(pump.scr, SCRAPE_ROUTE, SCRAPE_ROUTE_N);
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    EditArbiter arb(edit, nav);
    edit.set_press([&](uint8_t k) { pump.press(k, now); });
    edit.set_sweep_emit([](const char *) {});
    std::vector<std::string> done;
    edit.set_done([&](bool ok, const char *, const char *) {
      assert(ok);
      switch (arb.running_kind()) {
        case EditArbiter::Kind::OPS:
          done.push_back("sched-set");
          break;
        case EditArbiter::Kind::SWEEP:
          done.push_back("sched-read");
          break;
        default:
          done.push_back(arb.running()->name);
          break;
      }
    });
    assert(arb.enqueue(mfind("dhw-setpoint"), nullptr));           // get
    assert(arb.enqueue_sweep(mfind("timer-day"), 7, "schedule"));  // get, behind it
    PlanEdit::EditOp ops[4];
    sched_ops("FRIDAY", 3, 18, 30, "ON", ops);
    assert(arb.enqueue_ops(mfind("timer-day"), ops, 4, "schedule"));  // set: jumps both
    for (int i = 0; i < 200000 && done.size() < 3; i++) {
      now += 50;
      arb.tick(now, true);
    }
    assert(done.size() == 3);
    assert(done[0] == "sched-set");
    assert(done[1] == "dhw-setpoint");
    assert(done[2] == "sched-read");
    assert(pump.week[4][2].hh == 18 && pump.week[4][2].mi == 30 && pump.week[4][2].st == 1);
  }

  {  // arbiter: a running scrape cycle finishes first (edits never
     // interleave), queued requests then run in FIFO order while the
     // scheduler is held, and scraping resumes once the queue drains.
     // The fake has no alarms page, so the scrape cycle fails through its
     // verify timeouts and recovers -- a running cycle either way.
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    PlanNav nav(pump.scr, SCRAPE_ROUTE, SCRAPE_ROUTE_N);
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    EditArbiter arb(edit, nav);
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    edit.set_press([&](uint8_t k) { pump.press(k, now); });
    edit.set_pin(815);
    std::vector<std::string> done;
    edit.set_done([&](bool ok, const char *value, const char *) {
      done.push_back(std::string(arb.running()->name) + "=" + (ok ? value : "FAIL"));
    });
    nav.set_interval_ms(1000);  // eager scheduler: held only by the arbiter
    nav.enable(now);
    for (int i = 0; i < 100 && nav.idle(); i++) {
      now += 50;
      arb.tick(now, true);
    }
    assert(!nav.idle());  // a scrape cycle is running
    assert(arb.enqueue(mfind("dhw-setpoint"), "40.0"));
    assert(arb.enqueue(mfind("dhw-eco-setpoint"), nullptr));  // get, second
    while (!nav.idle()) {  // the cycle finishes first; the edit never starts
      assert(edit.idle());
      now += 50;
      arb.tick(now, true);
    }
    assert(nav.cycles() == 1);
    for (int i = 0; i < 100000 && done.size() < 2; i++) {
      now += 50;
      arb.tick(now, true);
      assert(nav.cycles() == 1);  // scheduler held while requests pend/run
    }
    assert(done.size() == 2);  // FIFO order, both against the live values
    assert(done[0] == "dhw-setpoint=40.0");
    assert(done[1] == "dhw-eco-setpoint=20.0");
    assert(pump.bvals[1][0] == 40.0);
    for (int i = 0; i < 100000 && nav.cycles() < 2; i++) {  // queue empty: free again
      now += 50;
      arb.tick(now, true);
    }
    assert(nav.cycles() == 2);
  }

  {  // arbiter hold (external client owns the session): a running scrape
     // cycle finishes first, then NOTHING starts -- no new cycle, not even
     // queued edits; release drains the queue and scraping resumes.
    FakePump pump;
    uint32_t now = 1000;
    pump.paint(now);
    PlanNav nav(pump.scr, SCRAPE_ROUTE, SCRAPE_ROUTE_N);
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    EditArbiter arb(edit, nav);
    nav.set_press([&](uint8_t k) { pump.press(k, now); });
    edit.set_press([&](uint8_t k) { pump.press(k, now); });
    std::vector<std::string> done;
    edit.set_done([&](bool ok, const char *value, const char *) {
      done.push_back(std::string(arb.running()->name) + "=" + (ok ? value : "FAIL"));
    });
    nav.set_interval_ms(1000);
    nav.enable(now);
    for (int i = 0; i < 100 && nav.idle(); i++) {
      now += 50;
      arb.tick(now, true);
    }
    assert(!nav.idle());  // a scrape cycle is running
    assert(arb.enqueue(mfind("dhw-setpoint"), "40.0"));
    while (!nav.idle()) {  // held: the running cycle still finishes first
      assert(edit.idle());
      now += 50;
      arb.tick(now, true, true);
    }
    assert(nav.cycles() == 1);
    for (int i = 0; i < 1000; i++) {  // held + idle: nothing starts (50 s)
      now += 50;
      arb.tick(now, true, true);
      assert(nav.idle() && edit.idle());
    }
    assert(arb.pending());  // the edit waited the hold out
    for (int i = 0; i < 100000 && done.size() < 1; i++) {  // release
      now += 50;
      arb.tick(now, true);
    }
    assert(done.size() == 1 && done[0] == "dhw-setpoint=40.0");
    for (int i = 0; i < 100000 && nav.cycles() < 2; i++) {  // scraping resumes
      now += 50;
      arb.tick(now, true);
    }
    assert(nav.cycles() == 2);
  }

  {  // arbiter refusals are synchronous (the caller snaps its entity back):
     // read-only set, unknown macro, full queue
    FakePump pump;
    PlanNav nav(pump.scr, SCRAPE_ROUTE, SCRAPE_ROUTE_N);
    PlanEdit edit(pump.scr);
    edit.set_specs(SPECS, SPECS_N);
    EditArbiter arb(edit, nav);
    assert(!arb.enqueue(mfind("clock-date"), "01/01/26"));
    assert(!arb.enqueue(nullptr, "1"));
    assert(!arb.pending());
    assert(arb.enqueue(mfind("clock-date"), nullptr));  // get on read-only is fine
    for (size_t i = 1; i < EDIT_QUEUE_N; i++)
      assert(arb.enqueue(mfind("mode"), nullptr));
    assert(!arb.enqueue(mfind("mode"), nullptr));  // full
    assert(arb.pending());
  }

  std::printf("ok\n");
  return 0;
}
