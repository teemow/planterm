#pragma once

// Non-blocking navigation engine for an enrolled pLAN terminal session: the
// C++ port of planscope's macro.go primitives (escAnchor, menuSelect,
// seekSelected, expectPage, settle waits) and the obsScrape scrape loop.
// Driven from the owner's loop() with millis()-based waits -- no task, no
// blocking; key presses go out asynchronously through an injected callback.
//
// Engine vs data: this header ships the verified primitives, the step
// expectation kinds, and the machines. WHAT to press -- menu headers and
// labels, routes, walk budgets, seek spans -- is device application data
// (it encodes ground truth about one controller application's menu tree)
// that the consumer supplies as NavMenu / MacroStep / ScrapeStep tables.
//
// Two machines share the primitives through NavEngine:
//   PlanNav (here)          -- a periodic scrape route, pure ScrapeStep data
//   PlanEdit (plan_edit.h)  -- transactional value edits (macro.go editValue)
//
// PlanNav scheduler: one cycle per interval (owner-configured); every step
// is verified against the screen before the next key (never a blind Enter),
// emit() fires once per visited view after it settled. On failure the cycle
// aborts, recovery Esc's back to the anchor, and the next run backs off
// exponentially (capped 16x) -- planscope runObserve's loop verbatim.
//
// Pure and platform-free like plan_screen.h/plan_fields.h: reads a
// PlanScreen, presses keys and logs through injected callbacks. Host-tested
// against a scripted fake heat pump in test/test_plan_nav.cpp.

#include "plan_fields.h"
#include "plan_frame.h"
#include "plan_screen.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>

namespace plan {

// Timing constants, all measured/derived in planscope:
static constexpr uint32_t NAV_QUIET_MS = 600;        // settleQuiet: repaint done
static constexpr uint32_t NAV_SETTLE_CAP_MS = 6000;  // waitSettle's 10x-quiet deadline
static constexpr uint32_t NAV_VERIFY_MS = 2000;      // navTimeout per step verification
static constexpr uint32_t NAV_SEEK_CHECK_MS = 500;   // seekSelected per-position check
// escAnchor press budget: the deepest pages observed sit 4 Escs down.
static constexpr int NAV_ESC_MAX = 6;
static constexpr int NAV_BACKOFF_CAP = 4;  // interval << min(fails, 4)
// The password gate gets 1 s to show after a PIN-flagged step (macro.go);
// a session that already passed the PIN lands straight on the target page.
static constexpr uint32_t EDIT_PIN_GATE_MS = 1000;

// pwGateRe: `(?i)^(service|manufacturer) password` on row 0.
inline bool edit_pw_gate(const char *row0) {
  auto ci_prefix = [](const char *s, const char *pfx) {
    for (; *pfx != '\0'; s++, pfx++) {
      char c = (*s >= 'A' && *s <= 'Z') ? static_cast<char>(*s + 32) : *s;
      if (c != *pfx)
        return false;
    }
    return true;
  };
  return ci_prefix(row0, "service password") || ci_prefix(row0, "manufacturer password");
}

// The gate's value display: `password\s*(PW\d):\s*NNNN` on row 5 -- port as
// "the 4 digits right after the row's colon" (the only colon the gate
// paints on that row).
inline bool edit_pw_shows(const char *row5, int shown) {
  const char *c = std::strchr(row5, ':');
  if (c == nullptr)
    return false;
  const char *p = c + 1;
  while (*p == ' ')
    p++;
  char want[8];
  std::snprintf(want, sizeof want, "%04d", shown);
  return std::strncmp(p, want, 4) == 0;
}

// One cursor-driven menu screen (planscope menuDef): its row-0 header prefix
// with the cursor position ("Main menu N/8", "Service menu N/7"), the /total
// that tells same-titled menus apart, and its entries in cursor order.
// Menu contents are device application data.
struct NavMenu {
  const char *header;
  int total;
  const char *const *labels;
  int n;
  // The cursor wraps at the ends (Down from n lands on 1, Up from 1 on n);
  // menu_select_in_ then takes the shortest modular path. Trailing member
  // with a default: existing aggregate tables stay valid (no wrap).
  bool wraps = false;
};

// Cursor position from the menu header (planscope menuCursor); 0 = not this
// menu's header. The header is fresh on page ENTRY only -- cursor MOVES
// repaint as graphics, so they are verified via the inverse band.
inline int nav_menu_cursor_in(const char *row0, const NavMenu &m) {
  size_t hl = std::strlen(m.header);
  if (std::strncmp(row0, m.header, hl) != 0)
    return 0;
  const char *p = fx_skip_sp(row0 + hl);
  if (!fx_digit(*p))
    return 0;
  int n = 0;
  while (fx_digit(*p))
    n = n * 10 + (*p++ - '0');
  if (*p++ != '/')
    return 0;
  int total = 0;
  while (fx_digit(*p))
    total = total * 10 + (*p++ - '0');
  return total == m.total ? n : 0;
}

// Step expectation kinds: what must hold on the screen after a move. The
// Go step.expect strings made data.
enum NavExp : uint8_t {
  NEXP_NONE,         // no verification ("" in Go)
  NEXP_ANCHOR,       // the status anchor (clock row 0)
  NEXP_MENU,         // the step's menu header is present ("menu"/row0 regexes)
  NEXP_PAGE,         // page_of == arg (a page ID: "A01", "Gfc10", ...)
  NEXP_PAGE_PREFIX,  // page_of starts with arg ("B0", "C0", "Gfc1")
  NEXP_PAGE_GLOB,    // page_of matches arg, '#' = one digit ("D##" = ^D\d\d$)
  NEXP_ROW_PREFIX,   // screen row `row` starts with arg at column 0
};

// NEXP_PAGE_GLOB matcher: '#' matches one digit, everything else literally;
// full match.
inline bool nav_page_glob(const char *page, const char *pat) {
  for (; *pat != '\0'; pat++, page++) {
    if (*pat == '#' ? !fx_digit(*page) : *page != *pat)
      return false;
  }
  return *page == '\0';
}

// One verifiable navigation move of a macro route (macro.go step), executed
// by PlanEdit's route runner. A step with key == 0 and a menu pins that
// menu's cursor onto arg (band-verified select); a step with a key AND a
// menu presses the key and expects the menu's header (NEXP_MENU).
struct MacroStep {
  uint8_t key;         // 0 on a menu-select step
  uint8_t times;       // presses (0 = 1); with seek: the press budget
  bool seek;           // press key up to times UNTIL exp holds (0 presses ok)
  bool pin;            // type the service PIN if the gate shows, else pass
  NavExp exp;
  uint8_t row;         // NEXP_ROW_PREFIX row
  const char *arg;     // page ID / glob / prefix / row prefix / menu label
  const NavMenu *menu; // key 0: select target; else the NEXP_MENU menu
};

// One verifiable move of a scrape route (pure data, executed by PlanNav).
// Exactly one move shape applies:
//   menu != nullptr && key == 0  -> band-verified menu select (arg = label)
//   band_seek                    -> seek_selected_(arg, span)
//   otherwise                    -> press key (0 = none), settle, verify exp
// emit publishes the settled page after the move verifies; walk > 0 turns
// the step into a budgeted walk -- (press + verify + emit) repeated up to
// walk times (e.g. a Down walk across pages whose numbering shifts with
// the device state, so no page ID is a reliable endpoint). An emitting
// walk also wrap-stops: when the settled body repeats a view already seen
// this walk (or the landing view), the walk ends early WITHOUT emitting,
// so every distinct view is published exactly once per cycle; the budget
// stays the safety cap (live-updating rows defeat the hash -- then the
// walk just runs to budget as before).
// pin != 0 marks a press step whose landing may be a password gate
// (1 = PW1/service, 2 = PW2/manufacturer): after the press the gate gets
// 1 s to show; an absent gate passes through, a visible one is typed with
// the owner-configured PIN (NavEngine::set_pin) before the verify. The
// field trails the struct so existing tables zero-fill it (no gate).
struct ScrapeStep {
  uint8_t key;
  uint8_t span;         // band_seek: seek span (span Downs, then 2x span Ups)
  bool band_seek;       // seek the selection band onto arg instead of a press
  NavExp exp;
  uint8_t row;          // NEXP_ROW_PREFIX row
  const char *arg;
  const NavMenu *menu;  // key 0: select target; else the NEXP_MENU menu
  bool emit;
  uint8_t walk;
  uint8_t pin{0};       // 0 = no gate expected; 1 = PW1; 2 = PW2
};

// NavEngine: the verified navigation primitives shared by the scrape machine
// (PlanNav) and the edit machine (PlanEdit, plan_edit.h) -- the settle-then-
// verify step_, esc_anchor_, the band-verified menu_select_in_ and
// seek_selected_, page identity, and the expectation check. Pure reads of a
// PlanScreen plus injected press/log callbacks; exactly one primitive runs
// at a time per derived machine (they share the aphase_/at0_ sub-state).
class NavEngine {
 public:
  explicit NavEngine(const PlanScreen &scr) : scr_(scr) {}

  void set_press(std::function<void(uint8_t)> f) { press_ = std::move(f); }
  void set_log(std::function<void(bool err, const char *msg)> f) { log_ = std::move(f); }

  // PW pins from the owner's config (which: 1 = PW1/service, 2 = PW2/
  // manufacturer); 0 = not configured -- a gated step FAILs at a visible
  // gate instead of typing a wrong PIN.
  void set_pin(uint8_t which, int pin) {
    if (which >= 1 && which <= 2)
      pins_[which - 1] = pin;
  }
  int pin_of(uint8_t which) const { return which >= 1 && which <= 2 ? pins_[which - 1] : 0; }

 protected:
  enum class Act : uint8_t { RUN, OK, FAIL };

  const char *row_(int r) const { return scr_.row(SCR_TERM_ESP, r); }
  const char *row0_() const { return row_(0); }

  void page_(char *out) const {
    const char *rows[FIELDS_ROWS];
    for (size_t r = 0; r < FIELDS_ROWS; r++)
      rows[r] = scr_.row(SCR_TERM_ESP, r);
    page_of(rows, out);
  }
  bool page_is_(const char *id) const {
    char p[FIELDS_PAGE_MAX];
    page_(p);
    return std::strcmp(p, id) == 0;
  }

  // FNV-1a over the body rows 1..7 (row 0 excluded: page headers lag the
  // body repaint and renumber with the device state) -- the wrap-stop
  // identity of a settled view. A separator byte per row keeps shifted row
  // contents from hashing alike.
  //
  // Digits are normalized to one fixed byte before hashing: page identity on
  // this device is the LABEL structure. Live numeric values (temperatures,
  // flows, counters) drift between walk passes -- hashing them verbatim made
  // a wrapped page never repeat, so the walk ran its full budget and emitted
  // every page twice (seen live: 25.0 -> 25.1 between D-list passes). Labels
  // repeat exactly on wrap; everything non-digit ('.', '-', degree signs,
  // spacing) stays literal so layout still matters.
  uint32_t body_hash_() const {
    uint32_t h = 2166136261u;
    for (int r = 1; r < static_cast<int>(SCR_ROWS); r++) {
      for (const char *p = row_(r); *p != '\0'; p++) {
        uint8_t c = static_cast<uint8_t>(*p);
        if (c >= '0' && c <= '9')
          c = '#';  // any digit hashes alike (volatile-value normalization)
        h ^= c;
        h *= 16777619u;
      }
      h ^= 0xFFu;  // row separator
      h *= 16777619u;
    }
    return h;
  }

  // The step expectation check (shared by both machines' route runners).
  bool expect_(NavExp exp, uint8_t row, const char *arg, const NavMenu *menu) const {
    switch (exp) {
      case NEXP_NONE:
        return true;
      case NEXP_ANCHOR:
        return fx_clock(row0_());
      case NEXP_MENU:
        return menu != nullptr && nav_menu_cursor_in(row0_(), *menu) > 0;
      case NEXP_PAGE:
        return page_is_(arg);
      case NEXP_PAGE_PREFIX: {
        char p[FIELDS_PAGE_MAX];
        page_(p);
        return std::strncmp(p, arg, std::strlen(arg)) == 0;
      }
      case NEXP_PAGE_GLOB: {
        char p[FIELDS_PAGE_MAX];
        page_(p);
        return nav_page_glob(p, arg);
      }
      case NEXP_ROW_PREFIX:
        return std::strncmp(row_(row), arg, std::strlen(arg)) == 0;
    }
    return false;
  }

  // waitSelected: the inverse-video band covers a row carrying the label
  // (menu moves are graphics-only; the band IS the cursor).
  bool selected_(const char *label) const {
    for (int r = 1; r < static_cast<int>(SCR_ROWS); r++) {
      const char *row = scr_.row(SCR_TERM_ESP, r);
      if (row[0] != '\0' && std::strstr(row, label) != nullptr &&
          scr_.row_inverse(SCR_TERM_ESP, r))
        return true;
    }
    return false;
  }

  // One verifiable move (macro.go runStep): optionally press, wait for the
  // repaint to settle (quiet >= 600 ms since press AND since the last frame,
  // capped at 6 s like waitSettle), then poll pred until verify_ms expires.
  // A keyless settle (key == 0, e.g. the emit after a verified move) has no
  // press to outwait, so it counts quiet from the last frame alone: a screen
  // that has already been silent >= 600 ms settles immediately instead of
  // re-serving the full wait (~600 ms x every emit of a scrape cycle).
  // Uses the shared aphase_/at0_ sub-state; exactly one step_ runs at a time.
  template<typename Pred>
  Act step_(uint8_t key, Pred pred, uint32_t verify_ms, uint32_t now, bool settle = true) {
    if (aphase_ == 0) {
      if (key != 0 && press_)
        press_(key);
      at0_ = now;
      aphase_ = settle ? 1 : 2;
      return Act::RUN;
    }
    if (aphase_ == 1) {
      uint32_t p = scr_.painted_ms(SCR_TERM_ESP);
      bool quiet = (key == 0 || now - at0_ >= NAV_QUIET_MS) &&
                   (p == 0 || now - p >= NAV_QUIET_MS);
      if (!quiet && now - at0_ < NAV_SETTLE_CAP_MS)
        return Act::RUN;
      at0_ = now;
      aphase_ = 2;  // fall through to the verify check this same tick
    }
    if (pred()) {
      aphase_ = 0;
      return Act::OK;
    }
    if (now - at0_ >= verify_ms) {
      aphase_ = 0;
      return Act::FAIL;
    }
    return Act::RUN;
  }

  // escAnchor: press Esc until the clock row shows; NAV_ESC_MAX press budget.
  // Per press: settle, one anchor check, next Esc (Go's escAnchor loop --
  // no per-press verify window, the settle already absorbed the repaint).
  Act esc_anchor_(uint32_t now) {
    if (aphase_ == 0) {
      if (fx_clock(row0_())) {
        esc_i_ = 0;
        return Act::OK;
      }
      if (esc_i_ >= NAV_ESC_MAX) {
        esc_i_ = 0;
        return Act::FAIL;
      }
      esc_i_++;
    }
    Act a = step_(KEY_ESC, [this] { return fx_clock(row0_()); }, 0, now);
    if (a == Act::OK) {
      esc_i_ = 0;
      return Act::OK;
    }
    return Act::RUN;  // FAIL = anchor not yet reached -> loop for another Esc
  }

  // menuSelect: pin a cursor menu onto label. The header gives the start
  // position; every move is verified by the band covering the expected
  // label -- the text truth that makes the following Enter safe (never
  // blindly into the wrong entry).
  Act menu_select_in_(const NavMenu &menu, const char *label, uint32_t now) {
    switch (mphase_) {
      case 0: {  // header must be present (fresh page entry)
        Act a = step_(0, [this, &menu] { return nav_menu_cursor_in(row0_(), menu) > 0; },
                      NAV_VERIFY_MS, now, false);
        if (a != Act::OK)
          return a == Act::FAIL ? Act::FAIL : Act::RUN;
        menu_pos_ = nav_menu_cursor_in(row0_(), menu);
        menu_target_ = 0;
        for (int i = 0; i < menu.n; i++)
          if (std::strcmp(menu.labels[i], label) == 0)
            menu_target_ = i + 1;
        if (menu_target_ == 0)
          return Act::FAIL;
        mphase_ = 1;
        return Act::RUN;
      }
      case 1: {  // walk the cursor, one verified move at a time
        if (menu_pos_ == menu_target_) {
          mphase_ = 2;
          return Act::RUN;
        }
        int dir = menu_pos_ < menu_target_ ? 1 : -1;
        if (menu.wraps) {  // shortest modular path (e.g. 8 -> 1 = one Down)
          int fwd = (menu_target_ - menu_pos_ + menu.n) % menu.n;
          dir = 2 * fwd <= menu.n ? 1 : -1;
        }
        int next_pos = menu_pos_ + dir;
        if (next_pos > menu.n)
          next_pos = 1;
        else if (next_pos < 1)
          next_pos = menu.n;
        const char *next = menu.labels[next_pos - 1];
        Act a = step_(dir > 0 ? KEY_DOWN : KEY_UP,
                      [this, next] { return selected_(next); }, NAV_VERIFY_MS, now, false);
        if (a == Act::OK)
          menu_pos_ = next_pos;
        else if (a == Act::FAIL) {
          mphase_ = 0;
          return Act::FAIL;
        }
        return Act::RUN;
      }
      default: {  // final band check on the target label
        Act a = step_(0, [this, label] { return selected_(label); }, NAV_VERIFY_MS, now, false);
        if (a != Act::RUN)
          mphase_ = 0;
        return a;
      }
    }
  }

  // seekSelected: walk a sub-menu cursor onto label. The landing position
  // after Enter is unknown (last-used), so seek Down across the span first,
  // then back Up across twice the span; each position gets a 500 ms band
  // check before the next press.
  Act seek_selected_(const char *label, int span, uint32_t now) {
    if (sphase_ == 2) {  // final check after both sweeps
      Act a = step_(0, [this, label] { return selected_(label); }, NAV_SEEK_CHECK_MS, now, false);
      if (a != Act::RUN) {
        sphase_ = 0;
        scheck_ = true;
        seek_i_ = 0;
      }
      return a;
    }
    int budget = sphase_ == 0 ? span : 2 * span;
    if (scheck_) {
      Act a = step_(0, [this, label] { return selected_(label); }, NAV_SEEK_CHECK_MS, now, false);
      if (a == Act::OK) {
        sphase_ = 0;
        scheck_ = true;
        seek_i_ = 0;
        return Act::OK;
      }
      if (a == Act::FAIL) {  // not here: press on (or turn around / give up)
        if (seek_i_ >= budget) {
          sphase_++;
          seek_i_ = 0;
        } else {
          scheck_ = false;
        }
      }
      return Act::RUN;
    }
    Act a = step_(sphase_ == 0 ? KEY_DOWN : KEY_UP, [] { return true; }, 0, now);
    if (a == Act::OK) {
      seek_i_++;
      scheck_ = true;
    }
    return Act::RUN;
  }

  // --- PIN entry (macro.go enterPIN, live-recorded for PW1) ------------------

  // Stage order cycles hundreds -> tens -> units -> thousands; the
  // thousands stage is skipped when its digit is 0 (the units Enter already
  // confirmed). Every Up is paced by the settle wait -- an Up fired into
  // the gate's repaint is accepted but not counted -- and read back against
  // the gate's own value display. The caller owns which pin to type
  // (pins_[which - 1]) and resets pphase_ before the first tick.
  Act pin_tick_(int pin, uint32_t now) {
    // positional divisors in stage order; also the per-Up display weights
    static const int WEIGHTS[4] = {100, 10, 1, 1000};
    switch (pphase_) {
      case 0:  // Enter focuses the hundreds digit
        if (step_(KEY_ENTER, [] { return true; }, 0, now) == Act::OK) {
          pin_stage_ = 0;
          pin_shown_ = 0;
          pin_left_ = (pin / 100) % 10;
          pphase_ = 1;
        }
        return Act::RUN;
      case 1: {  // the stage's Ups, each read back against the display
        if (pin_left_ == 0) {
          pphase_ = 2;
          return Act::RUN;
        }
        int want = pin_shown_ + WEIGHTS[pin_stage_];
        Act a = step_(KEY_UP, [this, want] { return edit_pw_shows(row_(5), want); },
                      NAV_VERIFY_MS, now);
        if (a == Act::FAIL)
          return Act::FAIL;  // gate never showed the digit
        if (a == Act::OK) {
          pin_shown_ = want;
          pin_left_--;
        }
        return Act::RUN;
      }
      default: {  // Enter advances to the next stage / confirms
        if (step_(KEY_ENTER, [] { return true; }, 0, now) != Act::OK)
          return Act::RUN;
        pin_stage_++;
        // the thousands stage is skipped when its digit is 0: the units
        // Enter already confirmed (the live-recorded model)
        if (pin_stage_ >= 4 || (pin_stage_ == 3 && (pin / 1000) % 10 == 0))
          return Act::OK;
        pin_left_ = (pin / WEIGHTS[pin_stage_]) % 10;
        pphase_ = 1;
        return Act::RUN;
      }
    }
  }

  void reset_nav_() {
    aphase_ = 0;
    esc_i_ = 0;
    mphase_ = 0;
    sphase_ = 0;
    scheck_ = true;
    seek_i_ = 0;
    pphase_ = 0;
  }

  const PlanScreen &scr_;
  std::function<void(uint8_t)> press_;
  std::function<void(bool, const char *)> log_;

  // step_ sub-state (shared: exactly one step_ runs at a time)
  uint8_t aphase_{0};
  uint32_t at0_{0};
  // sub-machine state
  int esc_i_{0};
  uint8_t mphase_{0};
  int menu_pos_{0}, menu_target_{0};
  uint8_t sphase_{0};
  bool scheck_{true};
  int seek_i_{0};

  char buf_[96];

  // PIN entry (pin_tick_) sub-state + the owner-configured pins
  // (pins_[0] = PW1, pins_[1] = PW2; 0 = not configured)
  uint8_t pphase_{0};
  int pin_stage_{0}, pin_shown_{0}, pin_left_{0};
  int pins_[2]{0, 0};
};

// PlanNav runs a consumer-supplied scrape route on a schedule: Esc to the
// status anchor, the ScrapeStep list in order, Esc back to the anchor.
class PlanNav : public NavEngine {
 public:
  PlanNav(const PlanScreen &scr, const ScrapeStep *route, size_t route_n)
      : NavEngine(scr), route_(route), route_n_(route_n) {}

  // "This page settled; publish a full snapshot" -- the owner extracts.
  void set_emit(std::function<void()> f) { emit_ = std::move(f); }
  void set_interval_ms(uint32_t ms) { interval_ms_ = ms; }

  // Swap the route table. Only while no cycle runs (mid-cycle the step index
  // and the old table's lifetime are live); returns false when refused.
  bool set_route(const ScrapeStep *route, size_t route_n) {
    if (st_ != St::IDLE)
      return false;
    route_ = route;
    route_n_ = route_n;
    return true;
  }

  // Start the scheduler; the first cycle runs at first_run_ms.
  void enable(uint32_t first_run_ms) {
    enabled_ = true;
    next_run_ms_ = first_run_ms;
  }
  bool enabled() const { return enabled_; }
  bool idle() const { return st_ == St::IDLE; }
  uint32_t cycles() const { return cycles_; }
  uint32_t fails() const { return fails_; }
  uint32_t next_run_ms() const { return next_run_ms_; }

  // Advance the machine; call every loop(). While not enrolled no new cycle
  // starts (a cycle already running just fails and backs off -- the presses
  // have no poll slot to ride in, so the screen never verifies).
  void tick(uint32_t now, bool enrolled) {
    if (st_ == St::IDLE) {
      if (!enabled_ || !enrolled || static_cast<int32_t>(now - next_run_ms_) < 0)
        return;
      reset_sub_();
      st_ = St::ANCHOR;
    }
    switch (st_) {
      case St::IDLE:
        break;
      case St::ANCHOR: {
        Act a = esc_anchor_(now);
        if (a == Act::OK)
          st_ = route_n_ > 0 ? St::ROUTE : St::ANCHOR_END;
        else if (a == Act::FAIL)
          fail_("anchor");
        break;
      }
      case St::ROUTE:
        route_tick_(now);
        break;
      case St::ANCHOR_END: {
        Act a = esc_anchor_(now);
        if (a == Act::OK)
          finish_(now, true);
        else if (a == Act::FAIL)
          fail_("return to anchor");
        break;
      }
      case St::RECOVER: {
        // Best-effort Esc back to the anchor; either way the cycle is over
        // and the scheduler backs off (planscope runObserve's recovery).
        if (esc_anchor_(now) != Act::RUN)
          finish_(now, false);
        break;
      }
    }
  }

 protected:
  enum class St : uint8_t { IDLE, ANCHOR, ROUTE, ANCHOR_END, RECOVER };

  // One route step per pass: the move (ephase_ 0), then the settled-page
  // emit (ephase_ 1); a walk step repeats the pair `walk` times.
  void route_tick_(uint32_t now) {
    const ScrapeStep &s = route_[step_i_];
    if (ephase_ == 0) {
      Act a;
      if (s.key == 0 && s.menu != nullptr) {
        a = menu_select_in_(*s.menu, s.arg, now);
      } else if (s.band_seek) {
        a = seek_selected_(s.arg, s.span, now);
      } else if (s.key == 0 && s.exp == NEXP_NONE) {
        a = Act::OK;  // emit-only step: nothing to press or verify
      } else if (s.pin != 0) {
        a = gate_step_(s, now);
      } else {
        a = step_(s.key, [this, &s] { return expect_(s.exp, s.row, s.arg, s.menu); },
                  s.exp == NEXP_NONE ? 0 : NAV_VERIFY_MS, now);
      }
      if (a == Act::FAIL) {
        std::snprintf(buf_, sizeof buf_, "step %d/%d", static_cast<int>(step_i_) + 1,
                      static_cast<int>(route_n_));
        fail_(buf_);
        return;
      }
      if (a != Act::OK)
        return;
      ephase_ = 1;
    }
    // emitScrapePage: wait for the page to settle, then publish one snapshot.
    // An emitting walk dedupes on the settled body: the seen-list seeds with
    // the landing view's hash (the previous emit) and a repeated body ends
    // the walk WITHOUT emitting -- every view published exactly once.
    bool wrap_stop = false;
    if (s.emit) {
      if (step_(0, [] { return true; }, 0, now) != Act::OK)
        return;
      uint32_t h = body_hash_();
      if (s.walk > 1) {
        if (walk_i_ == 0) {  // seed with the landing view
          walk_seen_n_ = 0;
          walk_seen_[walk_seen_n_++] = last_emit_hash_;
        }
        wrap_stop = walk_seen_has_(h);
        if (!wrap_stop && walk_seen_n_ < WALK_SEEN_MAX)
          walk_seen_[walk_seen_n_++] = h;
      }
      if (!wrap_stop) {
        last_emit_hash_ = h;
        if (emit_)
          emit_();
      }
    }
    ephase_ = 0;
    walk_i_++;
    if (wrap_stop || walk_i_ >= (s.walk == 0 ? 1 : static_cast<int>(s.walk))) {
      walk_i_ = 0;
      step_i_++;
      if (step_i_ >= route_n_)
        st_ = St::ANCHOR_END;
    }
  }

  bool walk_seen_has_(uint32_t h) const {
    for (uint8_t i = 0; i < walk_seen_n_; i++)
      if (walk_seen_[i] == h)
        return true;
    return false;
  }

  // A PIN-gated press step (pin != 0), mirroring PlanEdit's rphase_ 1/2/3:
  // press + settle, give the password gate 1 s to show; an absent gate is a
  // pass-through (the session already passed this PIN); a visible gate is
  // typed with the configured PIN -- unconfigured (0) FAILs the cycle
  // instead of typing a wrong one.
  Act gate_step_(const ScrapeStep &s, uint32_t now) {
    switch (gphase_) {
      case 0: {  // the press, settle-paced, no verify yet
        if (step_(s.key, [] { return true; }, 0, now) == Act::OK)
          gphase_ = 1;
        return Act::RUN;
      }
      case 1: {  // gate check (a missing gate is not an error)
        Act a = step_(0, [this] { return edit_pw_gate(row0_()); }, EDIT_PIN_GATE_MS, now, false);
        if (a == Act::OK) {
          if (pin_of(s.pin) == 0)
            return Act::FAIL;  // gated step without a configured PIN
          pphase_ = 0;
          gphase_ = 2;
        } else if (a == Act::FAIL) {
          gphase_ = 3;  // no gate showed: pass through
        }
        return Act::RUN;
      }
      case 2: {  // type the PIN
        Act a = pin_tick_(pin_of(s.pin), now);
        if (a == Act::FAIL)
          return Act::FAIL;
        if (a == Act::OK)
          gphase_ = 3;
        return Act::RUN;
      }
      default: {  // verify the step's expectation (navTimeout)
        Act a = step_(0, [this, &s] { return expect_(s.exp, s.row, s.arg, s.menu); },
                      s.exp == NEXP_NONE ? 0 : NAV_VERIFY_MS, now, false);
        if (a != Act::RUN)
          gphase_ = 0;
        return a;
      }
    }
  }

  void reset_sub_() {
    reset_nav_();
    step_i_ = 0;
    ephase_ = 0;
    walk_i_ = 0;
    gphase_ = 0;
    walk_seen_n_ = 0;
    last_emit_hash_ = 0;  // no stale cross-cycle landing seed
  }

  void fail_(const char *what) {
    if (log_) {
      char m[96];
      std::snprintf(m, sizeof m, "scrape failed at %.24s (row0 %.22s)", what, row0_());
      log_(true, m);
    }
    reset_sub_();
    st_ = St::RECOVER;
  }

  void finish_(uint32_t now, bool ok) {
    cycles_++;
    fails_ = ok ? 0 : fails_ + 1;
    uint32_t shift = fails_ < NAV_BACKOFF_CAP ? fails_ : NAV_BACKOFF_CAP;
    uint32_t wait = interval_ms_ << shift;
    next_run_ms_ = now + wait;
    reset_sub_();
    st_ = St::IDLE;
    if (log_) {
      if (ok)
        std::snprintf(buf_, sizeof buf_, "scrape ok (cycle %u); next in %us",
                      static_cast<unsigned>(cycles_), static_cast<unsigned>(wait / 1000));
      else
        std::snprintf(buf_, sizeof buf_, "recovered; retry in %us (fail #%u)",
                      static_cast<unsigned>(wait / 1000), static_cast<unsigned>(fails_));
      log_(false, buf_);
    }
  }

  const ScrapeStep *route_;
  size_t route_n_;
  std::function<void()> emit_;
  uint32_t interval_ms_{60000};

  bool enabled_{false};
  uint32_t next_run_ms_{0};
  St st_{St::IDLE};
  uint32_t cycles_{0};
  uint32_t fails_{0};
  size_t step_i_{0};
  uint8_t ephase_{0};  // 0 = the move, 1 = the emit
  int walk_i_{0};      // iterations done within a walk step
  uint8_t gphase_{0};  // gate_step_ sub-phase (pin != 0 steps)

  // Wrap-stop dedupe state (new members at class END -- see W3 offset rule).
  static constexpr uint8_t WALK_SEEN_MAX = 16;
  uint32_t walk_seen_[WALK_SEEN_MAX]{};  // settled-body hashes this walk
  uint8_t walk_seen_n_{0};
  uint32_t last_emit_hash_{0};  // the last published view (a walk's landing)
};

}  // namespace plan
