#pragma once

// Replay conformance for PlanTerminal (src/plan_terminal.h): every received
// byte of a recorded capture goes into on_byte exactly as the RX ISR feeds it
// (plan_bridge_isr.cpp: byte, recovered 9th bit, time), and the transmit
// actions it returns are compared with
//   vs-recorded   what the bridge actually transmitted in that capture
//   vs-pgd-model  what a pGD-faithful terminal at 0x1F would have sent
//                 (reference.h: wave A6 pGD model + A1 R-RC-12/R-RC-30)
//   walk-detector its FF-walk detector against every walk start on the wire
// and the reference itself is calibrated against the real pGD at 0x20 in the
// same capture (pgd-model-vs-pgd). A fixture may also name a counterfactual
// task state (`# counterfactual: enroll=1 fwd=1`): the same capture replayed
// as if the bridge had been configured that way (counterfactual-vs-pgd-model),
// e.g. a passive capture as if enrolled with today's boot configuration.
//
// Every slot (a received frame some side answers) is classified
//   same | different-bytes | different-timing | missing | extra
// from planterm's point of view (missing = the other side sends, planterm
// does not) and tagged with the spec rule (R-*), the deviation (D*, DV-*,
// V*) or the pGD-model transition (A6:T*) it maps to.

#include "../../src/plan_terminal.h"
#include "frames.h"
#include "reference.h"

#include <algorithm>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace replay {

static constexpr uint8_t BRIDGE = plan::ENROLL_ADDR;
static constexpr uint8_t PGD = 0x20;
static constexpr int64_t PGD_LIVE_US = 15'000'000;  // planterm's liveness window

struct Unit {
  std::vector<uint8_t> bytes;  // empty = silent
  bool empty() const { return bytes.empty(); }
};

inline std::string kind_of(const std::vector<uint8_t> &b) {
  if (b.empty())
    return "";
  if (b.size() < 2)
    return "ack";
  switch (b[1]) {
    case 0x01:
      return b[0] == 0x01 ? "link" : "fwd_poll";
    case 0x02:
      return "rollcall";
    case 0x03:
      return "session_ack";
    case 0x51:
      return "ident";
    case 0x1E:
      return "key";
    case 0x1F:
      return "type1F";
    default:
      return "other";
  }
}

// One recorded transmission of the bridge: the frames of one tx-sent entry
// (a key reply is two frames, report + link reply, with one device stamp).
struct TxUnit {
  size_t pos = 0;  // index of the first rx frame after it in file order
  std::string kind;
  int64_t dev_us = 0;
  std::vector<Frame> frames;
  std::vector<uint8_t> bytes;
};

struct Stream {
  std::vector<Frame> rx;
  std::vector<TxUnit> tx;
  std::vector<int> rec_of;      // rx index -> aligned tx unit, -1
  std::vector<size_t> unaligned;
  std::vector<bool> pgd_alive;  // planterm's liveness predicate at each rx frame
  std::vector<bool> after_join; // the previous frame to 0x1F was a roll-call
};

// A recorded transmission answers the frame (addressed to us) that ended
// right before it; this is the frame kind each tx kind can answer.
inline bool answers(const std::string &kind, const Frame &f) {
  if (!f.bit9 || f.to != BRIDGE)
    return false;
  int t = frame_type(f);
  if (kind == "rollcall")
    return t == 0x02;
  if (kind == "link" || kind == "key" || kind == "fwd_poll")
    return t == 0x01;
  if (kind == "ident")
    return t == plan::IDENT_REQ_TYPE;
  if (kind == "session_ack")
    return t >= 0x03 && t != plan::IDENT_REQ_TYPE;
  return false;
}

inline Stream build_stream(const Fixture &fx) {
  Stream s;
  for (const Frame &f : fx.frames) {
    if (!f.tx) {
      s.rx.push_back(f);
      continue;
    }
    if (f.kind == "9bit")
      continue;  // the task's TX(9bit) log of a queued key; the tx-sent entry is the truth
    TxUnit *last = s.tx.empty() ? nullptr : &s.tx.back();
    if (last && last->pos == s.rx.size() && last->kind == f.kind && f.dev_us != 0 &&
        last->dev_us == f.dev_us) {
      last->frames.push_back(f);
      last->bytes.insert(last->bytes.end(), f.bytes.begin(), f.bytes.end());
      continue;
    }
    TxUnit u;
    u.pos = s.rx.size();
    u.kind = f.kind;
    u.dev_us = f.dev_us;
    u.frames.push_back(f);
    u.bytes = f.bytes;
    s.tx.push_back(u);
  }
  assign_times(s.rx);

  // Align each recorded transmission to its trigger. The firmware logs the
  // tx-sent entry before the trigger frame's capture record (that record is
  // only closed by the NEXT address byte), so look forward first, monotone.
  s.rec_of.assign(s.rx.size(), -1);
  long last = -1;
  for (size_t ui = 0; ui < s.tx.size(); ui++) {
    const TxUnit &u = s.tx[ui];
    const long pos = static_cast<long>(u.pos);
    const long n = static_cast<long>(s.rx.size());
    long found = -1;
    for (long j = std::max(pos, last + 1); j < std::min(n, pos + 9) && found < 0; j++)
      if (s.rec_of[j] < 0 && answers(u.kind, s.rx[j]))
        found = j;
    for (long j = pos - 1; j >= std::max(last + 1, pos - 4) && found < 0; j--)
      if (s.rec_of[j] < 0 && answers(u.kind, s.rx[j]))
        found = j;
    if (found < 0) {
      s.unaligned.push_back(ui);
      continue;
    }
    s.rec_of[found] = static_cast<int>(ui);
    last = found;
    // Our reply goes out one turnaround after the trigger's last byte.
    const Frame &trig = s.rx[found];
    int64_t at = byte_time(trig, trig.bytes.size() - 1) + 250;
    for (Frame &tf : s.tx[ui].frames) {
      tf.start_us = at;
      tf.t_ms = trig.t_ms;
      at += static_cast<int64_t>(tf.bytes.size()) * 192;
    }
  }

  // Context for the tags: planterm's own pGD-liveness predicate (any frame
  // to 0x01 whose third byte is 0x20, 15 s) and "first frame after a join".
  int64_t alive_at = -1;
  bool prev_to_us_rc = false;
  for (const Frame &f : s.rx) {
    s.pgd_alive.push_back(alive_at >= 0 && f.start_us - alive_at < PGD_LIVE_US);
    s.after_join.push_back(prev_to_us_rc);
    if (f.bit9 && f.to == 0x01 && f.bytes.size() >= 4 && f.bytes[2] == PGD)
      alive_at = f.start_us;
    if (f.bit9 && f.to == BRIDGE)
      prev_to_us_rc = is_rollcall(f) && f.bytes[2] == 0x01;
  }
  return s;
}

// The bridge task's side of the ISR handshake, taken from the capture: the
// fixture's `# task:` line (state at the window start) plus `# events:`
// changes `field=value@L<line>`, applied before the first received frame at
// or after that source line (e.g. a set_poll_fwd service call).
struct TaskState {
  int enroll = 0, fwd = 0, tx_mode = 2;
  int64_t turnaround_us = 250;  // PlanBridge::turnaround_us_ default
  struct Event {
    int line;
    std::string field;
    int value;
  };
  std::vector<Event> events;

  void parse_events(const std::string &spec) {
    std::istringstream is(spec);
    std::string tok;
    while (is >> tok) {
      auto eq = tok.find('='), at = tok.find("@L");
      if (eq == std::string::npos || at == std::string::npos || at < eq)
        continue;  // prose around the events
      events.push_back({std::atoi(tok.c_str() + at + 2), tok.substr(0, eq), std::atoi(tok.c_str() + eq + 1)});
    }
    std::stable_sort(events.begin(), events.end(), [](const Event &a, const Event &b) { return a.line < b.line; });
  }
  // Apply every event due at `line`; returns the next event index.
  template <typename F>
  size_t apply_due(size_t next, int line, F &&set) const {
    for (; next < events.size() && events[next].line <= line; next++)
      set(events[next].field, events[next].value);
    return next;
  }
};

struct PlantermRun {
  std::vector<Unit> out;     // per rx index: what planterm put on the wire
  std::vector<bool> backoff; // per rx index: forward back-off active (a forward failed < 1 s ago)
  std::vector<bool> walk;    // per rx index: the FF-walk detector fired (tel_walks_ counted it)
  std::vector<bool> folded;  // per rx index: an opener folded into its lone announce (R-RC-06)
  uint32_t not_sent = 0;     // actions the ISR's FIFO gate would have dropped (stale match)
};

// The ISR loop of plan_bridge_isr.cpp, minus the hardware: on_byte per
// received byte; an action is sent only if no further byte completes within
// the turnaround (the ISR checks the RX FIFO before and after the delay),
// so a match inside a frame is dropped as stale, exactly like on the wire.
inline PlantermRun run_planterm(const Stream &s, const TaskState &ts) {
  plan::PlanTerminal term;
  term.enroll_ = ts.enroll != 0;
  term.fwd_polls_ = ts.fwd;
  term.tx_mode_ = ts.tx_mode;
  PlantermRun r;
  r.out.resize(s.rx.size());
  r.backoff.resize(s.rx.size());
  r.walk.resize(s.rx.size());
  r.folded.resize(s.rx.size());
  int64_t fail_at = -1;
  size_t ev = 0;
  for (size_t i = 0; i < s.rx.size(); i++) {
    const Frame &f = s.rx[i];
    ev = ts.apply_due(ev, f.line, [&](const std::string &k, int v) {
      if (k == "enroll")
        term.enroll_ = v != 0;
      else if (k == "fwd")
        term.fwd_polls_ = v;
      else if (k == "tx_mode")
        term.tx_mode_ = v;
    });
    const uint32_t fails = term.fwd_fail_, walks = term.tel_walks_, folds = term.tel_walks_folded_;
    // Task input: the key the recorded firmware had queued for this poll.
    const int ri = s.rec_of[i];
    if (ri >= 0 && s.tx[ri].kind == "key" && s.tx[ri].bytes.size() >= 11 && !term.tx_pending_) {
      const std::vector<uint8_t> &k = s.tx[ri].bytes;
      uint8_t rep[plan::REPLY9_LEN] = {k[0], k[1], k[2], PGD, k[4], k[5], 0, 0x01, 0x01, PGD, 0xDD};
      rep[6] = static_cast<uint8_t>(0xFE - (rep[1] + rep[2] + rep[3] + rep[4] + rep[5]));
      for (size_t j = 0; j < plan::REPLY9_LEN; j++)
        term.tx_frame_[j] = rep[j];
      term.tx_pending_ = true;
    }
    for (size_t j = 0; j < f.bytes.size(); j++) {
      const int64_t t = byte_time(f, j);
      plan::TxAction act = term.on_byte(f.bytes[j], (j == 0 && f.bit9) ? 1 : 0, t);
      if (act.kind == plan::TxAction::NONE)
        continue;
      int64_t next = INT64_MAX;
      if (j + 1 < f.bytes.size())
        next = byte_time(f, j + 1);
      else if (i + 1 < s.rx.size())
        next = byte_time(s.rx[i + 1], 0);
      if (next - t > ts.turnaround_us) {
        term.tx_sent(act, t + ts.turnaround_us + act.len * 192 + 16);
        r.out[i].bytes.assign(act.frame, act.frame + act.len);
      } else {
        term.tx_not_sent(act);
        r.not_sent++;
      }
    }
    if (term.fwd_fail_ != fails)
      fail_at = f.start_us;
    r.backoff[i] = fail_at >= 0 && f.start_us - fail_at < 1'000'000;
    r.walk[i] = term.tel_walks_ != walks;
    r.folded[i] = term.tel_walks_folded_ != folds;
  }
  return r;
}

inline std::vector<Unit> recorded_units(const Stream &s) {
  std::vector<Unit> out(s.rx.size());
  for (size_t i = 0; i < s.rx.size(); i++)
    if (s.rec_of[i] >= 0)
      out[i].bytes = s.tx[s.rec_of[i]].bytes;
  return out;
}

inline std::vector<Unit> units_of(const std::vector<RefOut> &ref) {
  std::vector<Unit> out(ref.size());
  for (size_t i = 0; i < ref.size(); i++)
    out[i].bytes = ref[i].bytes;
  return out;
}

inline std::vector<RefOut> run_reference(const Stream &s, const TaskState &ts, bool pgd_present) {
  RefTerminal ref(BRIDGE, true);
  ref.enrolled = ts.enroll != 0;
  ref.pgd_may_exist = pgd_present;
  std::vector<RefOut> out(s.rx.size());
  size_t ev = 0;
  for (size_t i = 0; i < s.rx.size(); i++) {
    ev = ts.apply_due(ev, s.rx[i].line, [&](const std::string &k, int v) {
      if (k == "enroll")
        ref.enrolled = v != 0;
    });
    const int ri = s.rec_of[i];
    if (ri >= 0 && s.tx[ri].kind == "key" && s.tx[ri].bytes.size() >= 7)
      ref.key.assign(s.tx[ri].bytes.begin(), s.tx[ri].bytes.begin() + 7);
    out[i] = ref.on_frame(s.rx[i]);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Divergences

struct Div {
  std::string cmp, cls, tag;
  int line = 0;          // source line of the slot frame (the trigger)
  int64_t t_ms = -1;
  int64_t start_us = 0;  // the slot frame's modelled wire start
  std::string pk;        // frame kind of p
  std::string slot;      // the slot frame, hex
  std::string p, o;      // planterm's / the other side's frame, hex
  std::string rule;      // the reference's rule for this slot
};

inline bool is_bare_ack(const Frame &f) {
  return f.bit9 && f.bytes.size() == 1 && f.bytes[0] == 0x01;
}

inline std::string tag_for(const std::string &cmp, const Stream &s, size_t i, const Unit &p,
                           const Unit &o, const RefOut *ref, bool backoff) {
  const std::string rule = ref ? ref->rule : std::string();
  const Frame &f = s.rx[i];
  if (!f.bit9 || f.to != BRIDGE)
    return "ISR:false-match";
  const int t = frame_type(f), snd = frame_sender(f);
  const std::string kp = kind_of(p.bytes), ko = kind_of(o.bytes);
  if (t == 0x02) {
    if (snd != 0x01)
      return "D7 (A6:T13; A1 R-RC-31 keeps it)";
    if (rule == "R-RC-30" && o.empty()) {
      if (kp == "rollcall" && ref->alt.size() > 3 && ((p.bytes[3] ^ ref->alt[3]) & 0x80))
        return "R-RC-30/V3 + D2/DV-2 honest skip";
      return "R-RC-30/V3";
    }
    if ((kp == "rollcall" && ko == "link") || (kp == "link" && ko == "rollcall"))
      return "D1/DV-1 (R-RC-13)";
    // The recording's confirm of a reply only the OLD firmware sent (planterm
    // stayed silent on the FF probe before it, R-RC-30): the slot exists only
    // because the bus was recorded with the old answer (B13 limits).
    if (p.empty() && f.bytes.size() > 3 && (f.bytes[3] & 0x40))
      return "counterfactual confirm (R-RC-30 silent before)";
    if (kp == "rollcall" && ko == "rollcall" && ((p.bytes[3] ^ o.bytes[3]) & 0x80))
      return "D2/DV-2 (R-RC-32)";
    return "R-LL-14";
  }
  if (t == 0x01) {
    if (kp == "fwd_poll" || ko == "fwd_poll") {
      if (!s.pgd_alive[i] || s.after_join[i])
        return "D3/DV-4 (R-LL-17)";
      if (cmp != "vs-recorded")
        return "R-LL-19 dual-terminal forward (A6:T19, no pGD rule)";
      return backoff ? "R-LL-19 fwd back-off after an unanswered forward" : "R-LL-19 alternation phase";
    }
    if (kp == "key" || ko == "key")
      return "A5:key-slot";
    if (snd != 0x01 && !p.empty() && !o.empty())
      return "A6:T12";
    return "R-LL-04";
  }
  if (snd != 0x01)
    return "D4/DV-3";
  if (!frame_valid(f))
    return "DV-7";
  if (t == 0x05)
    return "R-LL-20";
  return "R-SE-02";
}

inline Div make_div(const std::string &cmp, const std::string &cls, const std::string &tag,
                    const Frame &slot, const Unit &p, const Unit &o, const std::string &rule) {
  Div d;
  d.cmp = cmp;
  d.cls = cls;
  d.tag = tag;
  d.line = slot.line;
  d.t_ms = slot.t_ms;
  d.start_us = slot.start_us;
  d.slot = hex(slot.bytes);
  d.pk = kind_of(p.bytes);
  d.p = hex(p.bytes);
  d.o = hex(o.bytes);
  d.rule = rule;
  return d;
}

// Fold a missing + extra pair with identical bytes, or a pair of
// different-bytes slots whose frames are swapped, within a few slots into
// one different-timing divergence: the same frame sent in another slot.
inline void merge_timing(std::vector<Div> &divs, const std::vector<size_t> &slot_of) {
  const size_t WINDOW = 8;
  std::vector<bool> gone(divs.size(), false);
  for (size_t a = 0; a < divs.size(); a++) {
    if (gone[a] || divs[a].cls == "same")
      continue;
    for (size_t b = a + 1; b < divs.size() && slot_of[b] <= slot_of[a] + WINDOW; b++) {
      if (gone[b] || divs[b].cls == "same")
        continue;
      Div &x = divs[a], &y = divs[b];
      bool pair = (x.cls == "missing" && y.cls == "extra" && x.o == y.p) ||
                  (x.cls == "extra" && y.cls == "missing" && x.p == y.o);
      bool swap = x.cls == "different-bytes" && y.cls == "different-bytes" && x.p == y.o && y.p == x.o;
      if (!pair && !swap)
        continue;
      std::string tag = x.tag == y.tag ? x.tag : x.tag + "+" + y.tag;
      if (pair) {
        x.cls = "different-timing";
        x.tag = tag;
        if (x.p.empty())
          x.p = y.p + " (later slot L" + std::to_string(y.line) + ")";
        else
          x.o = y.o + " (later slot L" + std::to_string(y.line) + ")";
        gone[b] = true;
      } else {
        x.cls = "different-timing";
        x.tag = tag;
        y.cls = "same";
        y.tag = y.pk;
      }
      break;
    }
  }
  std::vector<Div> keep;
  for (size_t a = 0; a < divs.size(); a++)
    if (!gone[a])
      keep.push_back(divs[a]);
  divs.swap(keep);
}

inline std::vector<Div> compare(const std::string &cmp, const Stream &s, const PlantermRun &run,
                                const std::vector<Unit> &other, const std::vector<RefOut> *ref,
                                bool check_resend) {
  const std::vector<Unit> &pt = run.out;
  std::vector<Div> divs;
  std::vector<size_t> slot_of;
  for (size_t i = 0; i < s.rx.size(); i++) {
    const Unit &p = pt[i], &o = other[i];
    const std::string rule = ref ? (*ref)[i].rule : std::string();
    if (p.empty() && o.empty()) {
      if (ref && rule == "R-RC-30")
        divs.push_back(make_div(cmp, "same", "silent (R-RC-30)", s.rx[i], p, o, rule));
      else
        continue;
    } else if (p.bytes == o.bytes) {
      divs.push_back(make_div(cmp, "same", kind_of(p.bytes), s.rx[i], p, o, rule));
    } else {
      const char *cls = p.empty() ? "missing" : o.empty() ? "extra" : "different-bytes";
      divs.push_back(make_div(cmp, cls, tag_for(cmp, s, i, p, o, ref ? &(*ref)[i] : nullptr, run.backoff[i]),
                              s.rx[i], p, o, rule));
    }
    slot_of.push_back(i);
    // A6:T8 / R-LL-05: a pGD resends an unacked link reply (2 more copies,
    // ~14 ms apart). planterm never does (D5). Only judged where the
    // recording shows the reply really went out and the next frame on the
    // wire is neither the controller's ack nor a new frame to us.
    const std::string kp = kind_of(p.bytes);
    if (check_resend && (kp == "link" || kp == "key") && s.rec_of[i] >= 0 &&
        s.tx[s.rec_of[i]].bytes == p.bytes && i + 1 < s.rx.size() && !is_bare_ack(s.rx[i + 1]) &&
        !(s.rx[i + 1].bit9 && s.rx[i + 1].to == BRIDGE)) {
      Unit resend;
      resend.bytes.assign(p.bytes.end() - 4, p.bytes.end());
      divs.push_back(make_div(cmp, "missing", "D5 (R-LL-05/A6:T8)", s.rx[i], Unit(), resend, "A6:T8"));
      slot_of.push_back(i);
    }
  }
  merge_timing(divs, slot_of);
  return divs;
}

// The FF-walk detector (link_reset_, t_link_reset_us_, tel_walks_ / the HA
// walks sensor) against every walk start on the wire, cold or warm.
inline std::vector<Div> compare_walks(const Stream &s, const PlantermRun &run) {
  std::vector<Div> divs;
  for (size_t i = 0; i < s.rx.size(); i++) {
    const bool truth = is_ff_walk_start(s.rx[i]);
    if (!truth && !run.walk[i])
      continue;
    const Unit mark{{0x02, 0x02, 0x01}};
    if (truth && run.walk[i])
      divs.push_back(make_div("walk-detector", "same", "ff-walk", s.rx[i], mark, mark, ""));
    else if (truth && run.folded[i])  // the restart of a lone announce probe: one recovery
      divs.push_back(make_div("walk-detector", "same", "ff-walk restart folded (R-RC-06)", s.rx[i], mark,
                              mark, ""));
    else if (truth)
      divs.push_back(make_div("walk-detector", "missing",
                              s.rx[i].bytes[7] || s.rx[i].bytes[8] ? "D8/V1 warm walk (claims not 00 00)"
                                                                    : "D8/V1",
                              s.rx[i], Unit(), mark, ""));
    else
      divs.push_back(make_div("walk-detector", "extra", "D8/V1 false trigger", s.rx[i], mark, Unit(), ""));
  }
  return divs;
}

// Calibration: the reference at 0x20 (no R-RC-30) against the real pGD.
// The wire stream is the receive stream plus the bridge's recorded
// transmissions (a forwarded poll 20' 01 1F BF is a slot for the pGD too).
inline std::vector<Div> calibrate_pgd(const Stream &s) {
  struct W {
    const Frame *f;
  };
  std::vector<W> wire;
  for (size_t i = 0; i < s.rx.size(); i++) {
    wire.push_back({&s.rx[i]});
    if (s.rec_of[i] >= 0)
      for (const Frame &tf : s.tx[s.rec_of[i]].frames)
        wire.push_back({&tf});
  }
  const std::string cmp = "pgd-model-vs-pgd";
  RefTerminal model(PGD, false);
  std::vector<Div> divs;
  const Frame *slot = nullptr;
  RefOut expect;
  bool taken = false;
  std::vector<uint8_t> last_unit;  // the pGD's last frame unit, while no other frame intervened
  int resends = 0;
  bool context = false;            // a window cut mid-exchange starts with an orphan pGD frame
  auto close_slot = [&]() {
    if (slot && !taken && !expect.bytes.empty())
      divs.push_back(make_div(cmp, "missing", expect.rule + " unanswered", *slot, Unit(),
                              Unit{expect.bytes}, expect.rule));
    slot = nullptr;
  };
  for (size_t w = 0; w < wire.size(); w++) {
    const Frame &f = *wire[w].f;
    if (!f.tx && f.bit9 && f.from == PGD) {
      if (!context)
        continue;
      std::vector<uint8_t> unit = f.bytes;
      if (frame_type(f) == 0x1E && w + 1 < wire.size() && wire[w + 1].f->from == PGD &&
          frame_type(*wire[w + 1].f) == 0x01) {
        unit.insert(unit.end(), wire[w + 1].f->bytes.begin(), wire[w + 1].f->bytes.end());
        w++;
      }
      Unit act{unit};
      if (slot && !taken) {
        taken = true;
        std::vector<uint8_t> want = expect.bytes;
        // A key press is a physical input the model cannot know: a keypad
        // report in front of the expected link reply counts as the same answer.
        if (kind_of(unit) == "key" && kind_of(want) == "link" && unit.size() > want.size() &&
            std::equal(want.begin(), want.end(), unit.end() - want.size()))
          want = unit;
        if (want == unit)
          divs.push_back(make_div(cmp, "same", kind_of(unit), *slot, Unit{want}, act, expect.rule));
        else if (want.empty())
          divs.push_back(make_div(cmp, "extra", expect.rule.empty() ? "unclassified" : expect.rule + " answered",
                                  *slot, Unit(), act, expect.rule));
        else if (frame_type(*slot) == 0x01 && kind_of(unit) == "type1F")
          divs.push_back(make_div(cmp, "different-bytes", "R-LL-20", *slot, Unit{want}, act, expect.rule));
        else
          divs.push_back(make_div(cmp, "different-bytes", expect.rule + " (model)", *slot, Unit{want}, act,
                                  expect.rule));
        last_unit = unit;
        resends = 0;
      } else if (!last_unit.empty() && unit == last_unit && kind_of(unit) == "link") {
        resends++;
        divs.push_back(make_div(cmp, resends <= 2 ? "same" : "extra",
                                resends <= 2 ? "resend (A6:T8/R-LL-05)" : "A6:T8 resend beyond 3 attempts", f,
                                Unit{unit}, act, "A6:T8"));
      } else {
        std::string k = kind_of(unit), tag = "unclassified";
        if (k == "rollcall")
          tag = "R-RC-25 (pGD master walk/token)";
        else if (k == "type1F")
          tag = "R-SE-19/R-LL-20";
        else if (k == "fwd_poll")
          tag = "A6:T19";
        divs.push_back(make_div(cmp, "extra", tag, f, Unit(), act, ""));
        last_unit = unit;
        resends = 0;
      }
      continue;
    }
    close_slot();
    context = true;
    last_unit.clear();
    resends = 0;
    RefOut e = model.on_frame(f);
    if (f.bit9 && f.to == PGD) {
      slot = &f;
      expect = e;
      taken = false;
    }
  }
  close_slot();
  // Calibration rows carry the model's answer in `p` and the pGD's in `o`.
  return divs;
}

// ---------------------------------------------------------------------------
// The expectations table: one row per (fixture, comparison, class, tag).

struct Row {
  std::string fixture, cmp, cls, tag;
  int count = 0;
  int first_line = 0;
};

inline std::vector<Row> tabulate(const std::string &fixture, const std::vector<Div> &divs) {
  std::map<std::tuple<std::string, std::string, std::string>, Row> m;
  for (const Div &d : divs) {
    Row &r = m[std::make_tuple(d.cmp, d.cls, d.tag)];
    if (r.count == 0) {
      r.fixture = fixture;
      r.cmp = d.cmp;
      r.cls = d.cls;
      r.tag = d.tag;
      r.first_line = d.line;
    }
    r.count++;
  }
  std::vector<Row> rows;
  for (auto &kv : m)
    rows.push_back(kv.second);
  return rows;
}

inline std::string row_line(const Row &r) {
  return r.fixture + "\t" + r.cmp + "\t" + r.cls + "\t" + r.tag + "\t" + std::to_string(r.count) + "\tL" +
         std::to_string(r.first_line);
}

struct FixtureResult {
  std::vector<Div> divs;
  size_t unaligned = 0;  // recorded transmissions without a trigger (harness invariant: 0)
  std::string harness;   // alignment and ISR stale-gate counts, a row of the table
};

inline FixtureResult run_fixture(const Fixture &fx) {
  FixtureResult res;
  Stream s = build_stream(fx);
  TaskState ts;
  ts.enroll = fx.get_kv("task", "enroll", 0);
  ts.fwd = fx.get_kv("task", "fwd", 0);
  ts.tx_mode = fx.get_kv("task", "tx_mode", 2);
  ts.turnaround_us = fx.get_kv("task", "turnaround_us", 250);
  ts.parse_events(fx.get("events"));
  const bool pgd = fx.get("pgd") == "present";
  // `# warmup_ms: N`: slots in the first N ms only prime the state machine
  // (a window cut out of a running capture starts mid-alternation).
  const int64_t warm_end =
      s.rx.empty() ? 0 : s.rx[0].start_us + std::atoll(fx.get("warmup_ms", "0").c_str()) * 1000;

  PlantermRun pt = run_planterm(s, ts);
  std::vector<Unit> rec = recorded_units(s);
  std::vector<RefOut> ref = run_reference(s, ts, pgd);

  auto keep = [&](const std::vector<Div> &d) {
    for (const Div &x : d)
      if (x.start_us >= warm_end)
        res.divs.push_back(x);
  };
  keep(compare("vs-recorded", s, pt, rec, nullptr, false));
  keep(compare("vs-pgd-model", s, pt, units_of(ref), &ref, true));
  keep(calibrate_pgd(s));
  keep(compare_walks(s, pt));

  const std::string cf = fx.get("counterfactual");
  if (!cf.empty()) {
    TaskState cts = ts;
    cts.enroll = fx.get_kv("counterfactual", "enroll", ts.enroll);
    cts.fwd = fx.get_kv("counterfactual", "fwd", ts.fwd);
    PlantermRun cpt = run_planterm(s, cts);
    std::vector<RefOut> cref = run_reference(s, cts, pgd);
    keep(compare("counterfactual-vs-pgd-model", s, cpt, units_of(cref), &cref, false));
  }
  res.unaligned = s.unaligned.size();
  res.harness = "tx_units=" + std::to_string(s.tx.size()) + " unaligned=" + std::to_string(s.unaligned.size()) +
                " isr_stale_drops=" + std::to_string(pt.not_sent);
  return res;
}

}  // namespace replay
