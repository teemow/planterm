#pragma once

// Non-blocking transactional edit engine: the C++ port of planscope's
// macro.go value machinery -- navigate (whole-route retry), readValue,
// focusField, editValue/editLoop/waitChange, and enterPIN -- driven from
// the owner's loop() like PlanNav, sharing its verified primitives through
// NavEngine (plan_nav.h).
//
// Engine vs data: the machinery lives here; the macro registry it executes
// (MacroDef tables: routes, extractor field names, focus hops, safety
// clamps) and the extractor spec table are device application data supplied
// by the consumer.
//
// Edit-loop invariants, preserved exactly (macro.go):
//   - the target is validated BEFORE the edit focus opens: numeric/enum
//     mismatch and the safety clamp reject without touching the device;
//   - the first Enter focuses the page's first editable field; each focus
//     hop presses Enter again, committing the field it leaves UNCHANGED --
//     the target row is read back after every hop, and a hop that alters
//     it aborts before the edit loop starts;
//   - the edit loop reads the screen back after EVERY press (no step-size
//     table -- the device calibrates itself) and aborts on stall, stepping
//     past the target (off the device's step grid), moving away (wrapped),
//     or the EDIT_MAX_PRESS budget;
//   - enums walk Up first; a bounded list (stall) reverses ONCE and walks
//     Down; a wrapping list that comes back to the start without showing
//     the target errors out (full-cycle detection);
//   - any divergence presses Esc (cancels the edit focus) WITHOUT
//     committing; commit is Enter + read-back verify.
//
// Failure contract: an edit either commits a verified value or aborts back
// to the status anchor. Teardown is just Esc to the anchor -- an always-on
// session stays enrolled (the one teardown difference from planscope's
// macroTeardown).
//
// PW1 (enterPIN): the Service PIN comes from the owner (set_pin), not the
// table. Digits are typed hundreds -> tens -> units -> thousands (thousands
// skipped when 0 -- the live-recorded model), each Up paced by the settle
// wait and read back against the gate's own value display.
//
// Multi-op page visits: a request is generally an ordered list of sub-edits
// executed within ONE page visit's Enter walk. A macro get/set is the 1-op
// case; set_ops runs a consumer-built list (e.g. a week-scheduler write:
// day selector, then a slot's hh -> mi -> state -- selecting the value and
// editing dependent fields under the SAME focus walk when a selector's
// persistence across page exits is unverified). read_sweep pages a focused
// selector field through all its values WITHOUT committing (Esc
// cancel-restores) and emits each value for the owner to snapshot.
//
// Pure and platform-free; host-tested against a scripted fake heat pump in
// test/test_plan_edit.cpp.

#include "plan_fields.h"
#include "plan_nav.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>

namespace plan {

// editMaxPress: bounds one numeric edit -- the widest observed clamp range
// (50 K) at the finest plausible step (0.1 K) plus margin. Enum edits stop
// at the cycle detection well before this.
static constexpr int EDIT_MAX_PRESS = 600;
// navigate retries the whole route on a transient failure (the controller's
// net rebuild can bounce the session back to the anchor mid-route).
static constexpr int EDIT_NAV_ATTEMPTS = 3;
// The password gate gets 1 s to show after a PIN-flagged step (macro.go);
// a session that already passed PW1 lands straight on the target page.
static constexpr uint32_t EDIT_PIN_GATE_MS = 1000;

// One named configuration value (macro.go macroDef): the verified route
// from the status anchor to its page (every step verified against the
// screen, never a blind key sequence), the extractor field (resolved in the
// consumer's FieldSpec table -- the same tested matcher the observe layer
// uses), and, on multi-field pages, how many focus hops reach the field
// (Enter walks the page's editable fields in order; every hop commits the
// field it leaves UNCHANGED, so hopping is value-neutral).
struct MacroDef {
  const char *name;        // macro name ("heating-setpoint")
  const MacroStep *route;
  uint8_t route_n;
  const char *field;       // extractor field name in the consumer's specs
  uint8_t hops;            // Enter presses AFTER the first (0 = first field)
  float min, max;          // numeric safety clamp; 0,0 = enum (Up-cycle)
  bool read_only;          // set refused: the page's edit focus does not map
                           // onto the extracted row (e.g. sub-field walks)
};

// parseNum (Go): the whole trimmed string must parse as a number.
inline bool edit_num(const char *s, float *out) {
  while (*s == ' ')
    s++;
  char *end = nullptr;
  float f = std::strtof(s, &end);
  if (end == s)
    return false;
  while (*end == ' ')
    end++;
  if (*end != '\0')
    return false;
  *out = f;
  return true;
}

// valueEq (Go): numerically when both parse (a target "40" matches the
// displayed "40.0"), trimmed-verbatim otherwise.
inline bool edit_eq(const char *a, const char *b) {
  float fa, fb;
  if (edit_num(a, &fa) && edit_num(b, &fb))
    return std::fabs(fa - fb) < 0.05f;
  while (*a == ' ')
    a++;
  while (*b == ' ')
    b++;
  size_t la = std::strlen(a), lb = std::strlen(b);
  while (la > 0 && a[la - 1] == ' ')
    la--;
  while (lb > 0 && b[lb - 1] == ' ')
    lb--;
  return la == lb && std::strncmp(a, b, la) == 0;
}

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

// PlanEdit executes one request at a time. The owner (typically through the
// EditArbiter below) starts a request while the scrape is idle and ticks
// the machine; the result arrives through the done callback.
class PlanEdit : public NavEngine {
 public:
  // A request is at most this many sub-edits in one page visit.
  static constexpr size_t OPS_MAX = 4;
  // A selector sweep captures at most this many distinct values.
  static constexpr size_t SWEEP_MAX = 8;

  // One sub-edit of a request: the Enter-walk field index it sits at
  // (1-based), the extractor that reads it back, and the value to commit
  // ("" = read only).
  struct EditOp {
    const FieldSpec *spec;
    uint8_t field;
    char target[FIELDS_VAL_MAX];
  };

  explicit PlanEdit(const PlanScreen &scr) : NavEngine(scr) {}

  // The extractor table (device application data) that get/set/read_sweep
  // resolve MacroDef::field against.
  void set_specs(const FieldSpec *specs, size_t n) {
    specs_ = specs;
    nspecs_ = n;
  }
  // PW1 from the owner's config; 0 = no PIN available (a gated route fails
  // at the gate instead of typing a wrong one).
  void set_pin(int pin) { pin_ = pin; }
  int pin() const { return pin_; }
  // "The target page settled and its value row verified" -- fired right
  // after READ succeeds, before any edit focus opens. The owner hooks a
  // full-page force extraction here, which turns a queued get into a page
  // visit: one request refreshes EVERY field on the page.
  void set_emit(std::function<void()> f) { emit_ = std::move(f); }
  // done(ok, value, msg): on success value = the final read-back (the
  // committed value for a set, the read value for a get); on failure the
  // pre-edit value ("was ..."). msg = failure reason ("" on success).
  void set_done(std::function<void(bool, const char *, const char *)> f) {
    done_ = std::move(f);
  }
  // One distinct selector value captured by read_sweep. The screen shows
  // that value's page state when this fires, so the owner can snapshot
  // dependent rows (e.g. compose a scheduler day's program).
  void set_sweep_emit(std::function<void(const char *)> f) { sweep_emit_ = std::move(f); }

  bool idle() const { return st_ == St::IDLE; }

  // Start a read: navigate to the macro's page and read its value.
  bool get(const MacroDef *def) { return start_(def, nullptr); }

  // Start an edit: navigate, edit with read-back verification, commit.
  // Refused (false) while busy, on a read-only macro, or when the macro's
  // extractor is missing from the specs table (a programming error).
  bool set(const MacroDef *def, const char *target) {
    if (target == nullptr || def->read_only)
      return false;
    return start_(def, target);
  }

  // Multi-op write: ONE transactional page visit over `route`'s page --
  // the Enter walk crosses every field up to each op's index (committed
  // unchanged: hopping is value-neutral), each op then runs the full
  // read-back edit loop. Ops are consumer-built and consumer-validated at
  // the system boundary; per-op shape checks (numeric/enum) still run at
  // each op's arrival. An op committed before a later one aborts STAYS
  // committed -- the failure is reported, the screen is the only truth.
  // name labels the request in logs and the done context.
  bool set_ops(const MacroDef *route, const EditOp *ops, size_t n, const char *name) {
    if (st_ != St::IDLE || route == nullptr || n == 0 || n > OPS_MAX)
      return false;
    for (size_t i = 0; i < n; i++)
      if (ops[i].spec == nullptr)
        return false;
    def_ = route;
    for (size_t i = 0; i < n; i++)
      ops_[i] = ops[i];
    nops_ = static_cast<uint8_t>(n);
    set_req_ = true;
    sweep_ = false;
    name_ = name;
    begin_();
    return true;
  }

  // Selector-sweep read: focus the macro's field (a selector whose
  // dependent rows repaint per value) and page it through every one of its
  // `expected` values WITHOUT committing anything (teardown Esc
  // cancel-restores the selector), emitting each through sweep_emit.
  bool read_sweep(const MacroDef *route, int expected, const char *name) {
    if (st_ != St::IDLE || route == nullptr || expected < 1 ||
        expected > static_cast<int>(SWEEP_MAX))
      return false;
    const FieldSpec *sp = find_spec(specs_, nspecs_, route->field);
    if (sp == nullptr)
      return false;
    def_ = route;
    ops_[0].spec = sp;
    ops_[0].field = static_cast<uint8_t>(route->hops + 1);
    ops_[0].target[0] = '\0';
    nops_ = 1;
    set_req_ = false;
    sweep_ = true;
    sweep_expected_ = expected;
    name_ = name;
    begin_();
    return true;
  }

  // Advance the machine; call every loop() while not idle. While not
  // enrolled the presses have no poll slot to ride in, so the current
  // request just fails through its verify timeouts.
  void tick(uint32_t now) {
    switch (st_) {
      case St::IDLE:
        break;
      case St::ANCHOR: {
        Act a = esc_anchor_(now);
        if (a == Act::OK) {
          step_i_ = 0;
          rphase_ = 0;
          st_ = St::ROUTE;
        } else if (a == Act::FAIL) {
          retry_("could not reach the status anchor");
        }
        break;
      }
      case St::ROUTE: {
        Act a = route_tick_(now);
        if (a == Act::OK)
          st_ = St::READ;
        else if (a == Act::FAIL)
          retry_(buf_);
        break;
      }
      case St::READ: {
        // readValue: the extractor row must match within the verify window.
        Act a = step_(0, [this] { return read_(cur_); }, NAV_VERIFY_MS, now, false);
        if (a == Act::FAIL) {
          fail_("value row never matched");
          break;
        }
        if (a != Act::OK)
          break;
        std::strcpy(old_, cur_);
        if (emit_)
          emit_();  // page settled + verified: full-page force snapshot
        if (sweep_) {  // read_sweep: page the selector, commit nothing
          swphase_ = 0;
          st_ = St::SWEEP;
          break;
        }
        if (nops_ == 1) {
          const char *target = ops_[0].target;
          if (target[0] == '\0') {  // get: done
            finish_(true, "");
            break;
          }
          if (edit_eq(old_, target)) {  // already there; no edit focus opened
            finish_(true, "");
            break;
          }
          // validate the target BEFORE touching the device
          float tf, cf;
          t_is_num_ = edit_num(target, &tf);
          if (t_is_num_ != edit_num(old_, &cf)) {
            fail_("numeric/enum mismatch");
            break;
          }
          if (t_is_num_ && (def_->min != 0 || def_->max != 0) &&
              (tf < def_->min || tf > def_->max)) {
            fail_("target outside the safety clamp");
            break;
          }
        }
        // multi-op targets (set_ops) were validated upfront by the
        // consumer; per-op shape checks run at each op's arrival.
        op_i_ = 0;
        field_ = 0;
        st_ = St::FOCUS;
        break;
      }
      case St::FOCUS: {
        // focusField: Enter (the first focuses field 1; each further hop
        // commits the field it leaves UNCHANGED), read the current op's
        // row back -- a hop that alters it aborts before any edit.
        Act a = step_(KEY_ENTER, [this] { return read_(cur_); }, NAV_VERIFY_MS, now);
        if (a == Act::FAIL) {
          abort_("value row lost during focus");
          break;
        }
        if (a != Act::OK)
          break;
        if (!edit_eq(cur_, old_)) {
          abort_("focus hop altered the value");
          break;
        }
        field_++;
        arrive_();
        break;
      }
      case St::EDIT: {
        // editLoop: one press per pass, read back after EVERY press.
        const char *target = ops_[op_i_].target;
        if (press_i_ >= EDIT_MAX_PRESS) {
          abort_("target not reached within the press budget");
          break;
        }
        float tf = 0, cf = 0;
        if (t_is_num_) {
          edit_num(target, &tf);
          edit_num(cur_, &cf);
        }
        uint8_t key = reversed_ ? KEY_DOWN : KEY_UP;
        if (t_is_num_ && tf < cf)
          key = KEY_DOWN;
        // waitChange: the value row must show something DIFFERENT from cur_
        // (step_ presses only on its first tick, so key stays constant here)
        Act a = step_(key, [this] { return read_(next_) && std::strcmp(next_, cur_) != 0; },
                      NAV_VERIFY_MS, now, false);
        if (a == Act::RUN)
          break;
        press_i_++;
        if (a == Act::FAIL) {
          // stall: no edit focus, or the value sits at a device limit
          if (!t_is_num_ && !reversed_) {
            reversed_ = true;  // bounded enum: turn around once
            break;
          }
          abort_("value stuck (no edit focus, or device limit)");
          break;
        }
        if (edit_eq(next_, target)) {
          st_ = St::COMMIT;
          break;
        }
        if (t_is_num_) {
          float nf;
          if (!edit_num(next_, &nf)) {
            abort_("value went non-numeric");
            break;
          }
          if ((cf < tf) != (nf < tf)) {
            abort_("stepped past the target (not on the device's step grid)");
            break;
          }
          if (std::fabs(nf - tf) > std::fabs(cf - tf)) {
            abort_("value moved away from the target");
            break;
          }
        } else if (!reversed_ && edit_eq(next_, estart_)) {
          abort_("cycled every option without seeing the target");
          break;
        }
        std::strcpy(cur_, next_);
        break;
      }
      case St::COMMIT: {
        // Enter commits; read back and verify the device kept the value.
        Act a = step_(KEY_ENTER, [this] { return read_(cur_); }, NAV_VERIFY_MS, now);
        if (a == Act::FAIL) {
          fail_("commit read-back: value row never matched");
          break;
        }
        if (a != Act::OK)
          break;
        if (!edit_eq(cur_, ops_[op_i_].target)) {
          fail_("commit read-back mismatch");
          break;
        }
        field_++;  // the commit Enter advanced the focus to the next field
        if (op_i_ + 1 >= nops_) {
          finish_(true, "");
          break;
        }
        // Next op: its baseline comes off the SAME settled screen -- a
        // selector commit repaints the dependent rows, so their baselines
        // must be read only after it (never in READ, which shows the OLD
        // selection).
        op_i_++;
        if (!read_(old_)) {
          abort_("next value row lost after commit");
          break;
        }
        std::strcpy(cur_, old_);
        arrive_();
        break;
      }
      case St::SWEEP: {
        // read_sweep: with the selector FOCUSED, Up pages through its
        // values live (the dependent rows repaint per value); capture each
        // unseen one. A bounded list turns around ONCE (Down re-passes the
        // seen values, dedupe skips them); teardown's first Esc
        // cancel-restores the selector, so nothing ever commits.
        switch (swphase_) {
          case 0: {  // Enter focuses the selector; the row must hold
            Act a = step_(KEY_ENTER, [this] { return read_(cur_); }, NAV_VERIFY_MS, now);
            if (a == Act::FAIL) {
              abort_("selector row lost on focus");
              break;
            }
            if (a != Act::OK)
              break;
            if (!edit_eq(cur_, old_)) {
              abort_("focus altered the selector");
              break;
            }
            seen_n_ = 0;
            sweep_down_ = false;
            press_i_ = 0;
            swphase_ = 1;
            break;
          }
          case 1: {  // capture the shown value once
            bool unseen = true;
            for (int i = 0; i < seen_n_; i++)
              unseen = unseen && std::strcmp(seen_[i], cur_) != 0;
            if (unseen && seen_n_ < sweep_expected_) {
              std::snprintf(seen_[seen_n_++], FIELDS_VAL_MAX, "%s", cur_);
              if (sweep_emit_)
                sweep_emit_(cur_);
            }
            if (seen_n_ >= sweep_expected_) {  // every value seen (a cycle dedupes here)
              finish_(true, "");
              break;
            }
            swphase_ = 2;
            break;
          }
          default: {  // step to the next value
            // Budget: Ups to the top of a bounded list plus a full Down
            // re-pass; a cycling list captures everything well before this.
            if (press_i_ >= 2 * sweep_expected_ + 2) {  // can't happen on a
              finish_(true, "");                        // real list; belt anyway
              break;
            }
            Act a = step_(sweep_down_ ? KEY_DOWN : KEY_UP,
                          [this] { return read_(next_) && std::strcmp(next_, cur_) != 0; },
                          NAV_VERIFY_MS, now, false);
            if (a == Act::RUN)
              break;
            press_i_++;
            if (a == Act::FAIL) {  // stall: bounded end (or no focus at all)
              if (!sweep_down_) {
                sweep_down_ = true;  // turn around once
                break;
              }
              finish_(true, "");  // both ends hit: publish what we saw
              break;
            }
            std::strcpy(cur_, next_);
            swphase_ = 1;
            break;
          }
        }
        break;
      }
      case St::ABORT_ESC: {
        // Cancel the edit focus -- never commit a bad state.
        if (step_(KEY_ESC, [] { return true; }, 0, now) == Act::OK)
          finish_(false, buf_);
        break;
      }
      case St::TEARDOWN: {
        // Best-effort Esc back to the anchor; the session stays enrolled.
        if (esc_anchor_(now) != Act::RUN) {
          st_ = St::IDLE;
          // success: the final read-back (committed value; == old_ for get);
          // failure: the pre-edit value ("was ...").
          if (done_)
            done_(done_ok_, done_ok_ ? cur_ : old_, done_ok_ ? "" : buf_);
        }
        break;
      }
    }
  }

 protected:
  enum class St : uint8_t {
    IDLE,
    ANCHOR,
    ROUTE,
    READ,
    FOCUS,
    EDIT,
    COMMIT,
    SWEEP,
    ABORT_ESC,
    TEARDOWN
  };

  bool start_(const MacroDef *def, const char *target) {
    if (st_ != St::IDLE || def == nullptr)
      return false;
    const FieldSpec *sp = find_spec(specs_, nspecs_, def->field);
    if (sp == nullptr)
      return false;
    def_ = def;
    ops_[0].spec = sp;
    ops_[0].field = static_cast<uint8_t>(def->hops + 1);
    if (target != nullptr)
      std::snprintf(ops_[0].target, sizeof ops_[0].target, "%s", target);
    else
      ops_[0].target[0] = '\0';
    nops_ = 1;
    set_req_ = target != nullptr;
    sweep_ = false;
    name_ = def->name;
    begin_();
    return true;
  }

  void begin_() {
    old_[0] = cur_[0] = '\0';
    attempt_ = 0;
    reset_edit_();
    st_ = St::ANCHOR;
  }

  // The focus walk reached field_: hop on, or open the current op's edit.
  // An op whose row already shows the target goes straight to COMMIT --
  // the Enter there commits it unchanged and advances the walk.
  void arrive_() {
    const EditOp &op = ops_[op_i_];
    if (field_ != op.field) {
      st_ = St::FOCUS;
      return;
    }
    if (edit_eq(cur_, op.target)) {
      st_ = St::COMMIT;
      return;
    }
    float tf, cf;
    t_is_num_ = edit_num(op.target, &tf);
    if (t_is_num_ != edit_num(cur_, &cf)) {
      abort_("numeric/enum mismatch");
      return;
    }
    press_i_ = 0;
    reversed_ = false;
    std::strcpy(estart_, cur_);
    st_ = St::EDIT;
  }

  // readValue off the CURRENT page: the current op's extractor row.
  bool read_(char *out) const {
    const FieldSpec *sp = ops_[op_i_].spec;
    return fx_match(*sp, row_(sp->row), out, FIELDS_VAL_MAX);
  }

  // --- route runner (macro.go runRoute/runStep, non-blocking) ---------------

  Act route_tick_(uint32_t now) {
    const MacroStep &s = def_->route[step_i_];
    Act a = step_run_(s, now);
    if (a == Act::FAIL) {
      std::snprintf(buf_, sizeof buf_, "route step %d failed (row0 %.22s)", step_i_ + 1, row0_());
      return Act::FAIL;
    }
    if (a != Act::OK)
      return Act::RUN;
    step_i_++;
    rphase_ = 0;
    press_i_ = 0;
    return step_i_ >= def_->route_n ? Act::OK : Act::RUN;
  }

  // One MacroStep. rphase_: 0 = pressing (or menu/seek), 1 = PIN gate check,
  // 2 = PIN entry, 3 = final expect verify.
  Act step_run_(const MacroStep &s, uint32_t now) {
    if (s.key == 0 && s.menu != nullptr)
      return menu_select_in_(*s.menu, s.arg, now);
    if (s.seek) {
      // press key up to `times` UNTIL the expectation holds (0 presses ok);
      // the screen is settled, so the check needs no long timeout.
      if (rphase_ == 0) {
        Act a = step_(0, [this, &s] { return expect_(s.exp, s.row, s.arg, s.menu); },
                      NAV_SEEK_CHECK_MS, now, false);
        if (a == Act::OK)
          return Act::OK;
        if (a == Act::FAIL) {
          if (press_i_ >= (s.times == 0 ? 1 : s.times))
            return Act::FAIL;
          rphase_ = 1;
        }
        return Act::RUN;
      }
      if (step_(s.key, [] { return true; }, 0, now) == Act::OK) {
        press_i_++;
        rphase_ = 0;
      }
      return Act::RUN;
    }
    switch (rphase_) {
      case 0: {  // the step's presses, each settle-paced
        int times = s.times == 0 ? 1 : s.times;
        if (press_i_ >= times) {
          rphase_ = s.pin ? 1 : 3;
          return Act::RUN;
        }
        if (step_(s.key, [] { return true; }, 0, now) == Act::OK)
          press_i_++;
        return Act::RUN;
      }
      case 1: {  // PIN gate: a session that already passed PW1 lands
        // straight on the target page, so a missing gate is not an error.
        Act a = step_(0, [this] { return edit_pw_gate(row0_()); }, EDIT_PIN_GATE_MS, now, false);
        if (a == Act::OK) {
          if (pin_ == 0)
            return Act::FAIL;  // gated route without a configured PIN
          pphase_ = 0;
          rphase_ = 2;
        } else if (a == Act::FAIL) {
          rphase_ = 3;  // no gate showed: pass through
        }
        return Act::RUN;
      }
      case 2: {  // type the PIN
        Act a = pin_tick_(now);
        if (a == Act::FAIL)
          return Act::FAIL;
        if (a == Act::OK)
          rphase_ = 3;
        return Act::RUN;
      }
      default:  // verify the step's expectation (navTimeout)
        return step_(0, [this, &s] { return expect_(s.exp, s.row, s.arg, s.menu); },
                     NAV_VERIFY_MS, now, false);
    }
  }

  // --- PIN entry (macro.go enterPIN, live-recorded for PW1) ------------------

  // Stage order cycles hundreds -> tens -> units -> thousands; the
  // thousands stage is skipped when its digit is 0 (the units Enter already
  // confirmed). Every Up is paced by the settle wait -- an Up fired into
  // the gate's repaint is accepted but not counted -- and read back against
  // the gate's own value display.
  Act pin_tick_(uint32_t now) {
    // positional divisors in stage order; also the per-Up display weights
    static const int WEIGHTS[4] = {100, 10, 1, 1000};
    switch (pphase_) {
      case 0:  // Enter focuses the hundreds digit
        if (step_(KEY_ENTER, [] { return true; }, 0, now) == Act::OK) {
          pin_stage_ = 0;
          pin_shown_ = 0;
          pin_left_ = (pin_ / 100) % 10;
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
        if (pin_stage_ >= 4 || (pin_stage_ == 3 && (pin_ / 1000) % 10 == 0))
          return Act::OK;
        pin_left_ = (pin_ / WEIGHTS[pin_stage_]) % 10;
        pphase_ = 1;
        return Act::RUN;
      }
    }
  }

  // --- outcome paths ----------------------------------------------------------

  void reset_edit_() {
    reset_nav_();
    step_i_ = 0;
    rphase_ = 0;
    press_i_ = 0;
    pphase_ = 0;
    op_i_ = 0;
    field_ = 0;
    reversed_ = false;
    swphase_ = 0;
    sweep_down_ = false;
    seen_n_ = 0;
  }

  // Route failures retry the whole route (Go navigate): Esc'ing back and
  // rerunning commits nothing.
  void retry_(const char *why) {
    if (++attempt_ >= EDIT_NAV_ATTEMPTS) {
      fail_(why);
      return;
    }
    if (log_) {
      char m[96];
      std::snprintf(m, sizeof m, "route attempt %d failed (%.60s); retrying", attempt_, why);
      log_(false, m);
    }
    reset_edit_();
    st_ = St::ANCHOR;
  }

  // abort_: cancel the open edit focus via Esc, then tear down (the Go
  // editValue error path -- never commit a bad state).
  void abort_(const char *why) {
    keep_msg_(why);
    reset_edit_();
    st_ = St::ABORT_ESC;
  }

  // fail_: no edit focus is open; straight to teardown.
  void fail_(const char *why) {
    keep_msg_(why);
    finish_(false, buf_);
  }

  void keep_msg_(const char *why) {
    if (why != buf_)
      std::snprintf(buf_, sizeof buf_, "%s", why);
  }

  void finish_(bool ok, const char *msg) {
    keep_msg_(msg);
    done_ok_ = ok;
    if (log_ && !ok) {
      char m[120];
      std::snprintf(m, sizeof m, "%s %.24s: %.80s", set_req_ ? "set" : "get", name_, buf_);
      log_(true, m);
    }
    reset_edit_();
    st_ = St::TEARDOWN;
  }

  const MacroDef *def_{nullptr};
  const char *name_{""};
  const FieldSpec *specs_{nullptr};
  size_t nspecs_{0};
  int pin_{0};
  std::function<void(bool, const char *, const char *)> done_;
  std::function<void()> emit_;
  std::function<void(const char *)> sweep_emit_;

  St st_{St::IDLE};
  int attempt_{0};
  // route runner
  int step_i_{0};
  uint8_t rphase_{0};
  int press_i_{0};
  // PIN entry
  uint8_t pphase_{0};
  int pin_stage_{0}, pin_shown_{0}, pin_left_{0};
  // request: the ordered sub-edits of one page visit (1 for a macro
  // get/set, up to OPS_MAX for set_ops), and read_sweep's selector walk
  EditOp ops_[OPS_MAX];
  uint8_t nops_{0};
  uint8_t op_i_{0};   // current op
  uint8_t field_{0};  // Enter-walk field currently focused (0 = none)
  bool set_req_{false};
  bool sweep_{false};
  int sweep_expected_{0};
  uint8_t swphase_{0};
  bool sweep_down_{false};
  int seen_n_{0};
  char seen_[SWEEP_MAX][FIELDS_VAL_MAX];
  // edit
  bool t_is_num_{false};
  bool reversed_{false};
  bool done_ok_{false};
  char old_[FIELDS_VAL_MAX];
  char cur_[FIELDS_VAL_MAX];
  char next_[FIELDS_VAL_MAX];
  char estart_[FIELDS_VAL_MAX];
};

// --- arbiter --------------------------------------------------------------------
// One nav owner at a time on the single enrolled session. Entity writes
// enqueue an edit request; the queue drains only while the scrape is idle,
// and the scrape scheduler starts no new cycle while requests are pending
// or an edit runs -- a running scrape cycle finishes first, so edits wait
// but never interleave. Owned by the component, ticked once per loop();
// pure like the machines it arbitrates, host-tested in
// test/test_plan_edit.cpp.

// ponytail: fixed FIFO, refuse when full -- sized for a scene touching a
// couple dozen writable entities plus one full scheduled page-read batch; a
// bigger burst just snaps back. Each slot carries inline OPS payload
// (~150 B), so the queue costs a few KB of static RAM; shrink EDIT_QUEUE_N
// on tighter targets.
static constexpr size_t EDIT_QUEUE_N = 32;

class EditArbiter {
 public:
  enum class Kind : uint8_t { GET, SET, OPS, SWEEP };

  EditArbiter(PlanEdit &edit, PlanNav &nav) : edit_(edit), nav_(nav) {}

  void set_log(std::function<void(bool err, const char *msg)> f) { log_ = std::move(f); }

  // Enqueue a get (target nullptr) or a set. Refused (false) on a set to a
  // read-only macro or a full queue -- synchronously, so the caller can
  // snap its entity state back right away.
  bool enqueue(const MacroDef *def, const char *target) {
    if (def == nullptr || n_ >= EDIT_QUEUE_N)
      return false;
    if (target != nullptr && def->read_only)
      return false;
    Req &r = q_[(head_ + n_) % EDIT_QUEUE_N];
    r.def = def;
    r.kind = target != nullptr ? Kind::SET : Kind::GET;
    if (target != nullptr)
      std::snprintf(r.target, sizeof r.target, "%s", target);
    r.name = def->name;
    n_++;
    return true;
  }

  // Enqueue a multi-op page visit (a set: jumps queued gets). Ops are the
  // owner's, already validated at its system boundary; name must outlive
  // the request (a string literal).
  bool enqueue_ops(const MacroDef *route, const PlanEdit::EditOp *ops, size_t nops,
                   const char *name) {
    if (route == nullptr || nops == 0 || nops > PlanEdit::OPS_MAX || n_ >= EDIT_QUEUE_N)
      return false;
    Req &r = q_[(head_ + n_) % EDIT_QUEUE_N];
    r.def = route;
    r.kind = Kind::OPS;
    for (size_t i = 0; i < nops; i++)
      r.ops[i] = ops[i];
    r.nops = static_cast<uint8_t>(nops);
    r.name = name;
    n_++;
    return true;
  }

  // Enqueue a selector-sweep read (a get: entity writes jump it).
  bool enqueue_sweep(const MacroDef *route, int expected, const char *name) {
    if (route == nullptr || n_ >= EDIT_QUEUE_N)
      return false;
    Req &r = q_[(head_ + n_) % EDIT_QUEUE_N];
    r.def = route;
    r.kind = Kind::SWEEP;
    r.expected = static_cast<uint8_t>(expected);
    r.name = name;
    n_++;
    return true;
  }

  bool pending() const { return n_ > 0; }
  // The request the edit machine executed last (context for the owner's
  // done callback: which field to publish, which name to log). For OPS and
  // SWEEP requests def is the route carrier; running_kind() tells them
  // apart from plain macro get/set.
  const MacroDef *running() const { return running_; }
  Kind running_kind() const { return running_kind_; }

  // One pass per loop(); ticks exactly one machine, so the two never
  // interleave key presses on the shared session. hold = an external client
  // owns the session (an armed write path, or an explicit observe pause):
  // the running machine finishes its cycle, then nothing new starts --
  // queued edits stay queued until the hold releases.
  void tick(uint32_t now, bool joined, bool hold = false) {
    if (!edit_.idle()) {
      edit_.tick(now);
      return;
    }
    if (n_ == 0 && !hold) {
      nav_.tick(now, joined);  // normal scraping; the scheduler is free
      return;
    }
    if (!nav_.idle()) {
      // A running cycle finishes first. Ticked only while NOT idle, the
      // scheduler cannot start a new cycle under a pending request or hold.
      nav_.tick(now, joined);
      return;
    }
    if (hold)
      return;  // the session is the external client's until release
    if (!joined)
      return;  // presses have no poll slot to ride in; hold the queue
    // Sets jump ahead of gets: scheduled page reads can queue several gets
    // at once, and an entity write must not wait behind them. Sets stay
    // FIFO among sets, gets FIFO among gets.
    size_t pick = 0;
    for (size_t i = 0; i < n_; i++) {
      Kind k = q_[(head_ + i) % EDIT_QUEUE_N].kind;
      if (k == Kind::SET || k == Kind::OPS) {
        pick = i;
        break;
      }
    }
    Req r = q_[(head_ + pick) % EDIT_QUEUE_N];
    for (size_t i = pick; i > 0; i--)  // close the gap toward the head
      q_[(head_ + i) % EDIT_QUEUE_N] = q_[(head_ + i - 1) % EDIT_QUEUE_N];
    head_ = (head_ + 1) % EDIT_QUEUE_N;
    n_--;
    running_ = r.def;
    running_kind_ = r.kind;
    bool started = false;
    switch (r.kind) {
      case Kind::GET:
        started = edit_.get(r.def);
        break;
      case Kind::SET:
        started = edit_.set(r.def, r.target);
        break;
      case Kind::OPS:
        started = edit_.set_ops(r.def, r.ops, r.nops, r.name);
        break;
      case Kind::SWEEP:
        started = edit_.read_sweep(r.def, r.expected, r.name);
        break;
    }
    if (log_) {
      char m[96];
      if (!started) {  // enqueue filtered read-only; only a bug lands here
        std::snprintf(m, sizeof m, "edit %.24s refused by the engine", r.name);
      } else if (r.kind == Kind::OPS) {
        std::snprintf(m, sizeof m, "edit start: %.24s (%u ops)", r.name,
                      static_cast<unsigned>(r.nops));
      } else if (r.kind == Kind::SWEEP) {
        std::snprintf(m, sizeof m, "edit start: %.24s (sweep)", r.name);
      } else {
        std::snprintf(m, sizeof m, "edit start: %.24s%s%.23s", r.name,
                      r.kind == Kind::SET ? " -> " : " (get)", r.kind == Kind::SET ? r.target : "");
      }
      log_(!started, m);
    }
    if (started)
      edit_.tick(now);
  }

 protected:
  struct Req {
    const MacroDef *def;                    // the macro, or the OPS/SWEEP route carrier
    Kind kind;
    char target[FIELDS_VAL_MAX];            // SET target
    PlanEdit::EditOp ops[PlanEdit::OPS_MAX];  // OPS payload
    uint8_t nops;
    uint8_t expected;                       // SWEEP: distinct selector values
    const char *name;                       // log/context label (string literal)
  };

  PlanEdit &edit_;
  PlanNav &nav_;
  std::function<void(bool, const char *)> log_;
  Req q_[EDIT_QUEUE_N];
  size_t head_{0}, n_{0};
  const MacroDef *running_{nullptr};
  Kind running_kind_{Kind::GET};
};

}  // namespace plan
