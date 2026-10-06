// Display settle and value correctness (wave E4; A4 R-DI-22..27, W-01..W-10;
// A3 R-SE-05/06): crafted wire-frame sequences through PlanScreen, the
// NavEngine settle, and the field extraction. One block per W-id.
//
// Every frame is built byte-exact (sum-8 or CRC-16/Modbus trailer, the
// address byte carries bit9) and fed byte by byte, so the tests exercise the
// same path as the firmware's receive stream.

#include "../src/plan_nav.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace plan;

using Bytes = std::vector<uint8_t>;

static uint16_t crc16(const Bytes &d) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : d) {
    crc = static_cast<uint16_t>(crc ^ b);
    for (int k = 0; k < 8; k++)
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
  }
  return crc;
}

// ADDR TYPE LEN 01 <payload> CK, LEN = total length.
static Bytes disp(uint8_t addr, uint8_t type, const Bytes &payload) {
  Bytes f{addr, type, 0, 0x01};
  f.insert(f.end(), payload.begin(), payload.end());
  bool crc = type == 0x64 || type == 0x65 || type == 0x66;
  f[2] = static_cast<uint8_t>(f.size() + (crc ? 2 : 1));
  if (crc) {
    uint16_t c = crc16(f);
    f.push_back(static_cast<uint8_t>(c & 0xFF));
    f.push_back(static_cast<uint8_t>(c >> 8));
  } else {
    f.push_back(static_cast<uint8_t>(0xFF - sum8(f.data(), f.size())));
  }
  return f;
}
static Bytes row_frame(uint8_t addr, uint8_t r, const char *text, size_t cols = SCR_COLS) {
  Bytes p{r};
  size_t n = std::strlen(text);
  for (size_t c = 0; c < cols; c++)
    p.push_back(static_cast<uint8_t>(c < n ? text[c] : ' '));
  return disp(addr, 0x0B, p);
}
static Bytes cell(uint8_t addr, uint8_t r, uint8_t c, char ch) {
  return disp(addr, 0x0C, {r, c, static_cast<uint8_t>(ch)});
}
static Bytes link(const Bytes &head) {  // sum-8 link-layer frame
  Bytes f = head;
  f.push_back(static_cast<uint8_t>(0xFF - sum8(f.data(), f.size())));
  return f;
}
static Bytes poll(uint8_t addr) { return link({addr, 0x01, 0x01}); }
// Controller roll-call: MSB of the first mask byte = address 32.
static Bytes rollcall(uint8_t to, uint32_t map, uint32_t claims) {
  return link({to, 0x02, 0x01, static_cast<uint8_t>(map >> 24), static_cast<uint8_t>(map >> 16),
               static_cast<uint8_t>(map >> 8), static_cast<uint8_t>(map), static_cast<uint8_t>(claims >> 24),
               static_cast<uint8_t>(claims >> 16), static_cast<uint8_t>(claims >> 8),
               static_cast<uint8_t>(claims)});
}
static uint32_t bit(int a) { return 1u << (a - 1); }

static const char *ANCHOR[8] = {"10:00 06/10/26 Ekobee1", "",
                                "    Hotwater:   35.9\xDF" "C", "    Heating:    30.1\xDF" "C",
                                "    Outside:    12.0\xDF" "C", "",
                                "  STATUS:            ", "                  On  "};
static const char *D02[8] = {" Input/Output      D02", "", "", "B4 =Heat source in.",
                             "    temp.:      20.7\xDF" "C", "", "", ""};
static const char *MENU[8] = {"Main menu          3/8", "", "A.Unit On/Off", "B.Setpoint",
                              "C.Clock/Scheduler", "D.Input/Output", "E.Data logger", "F.Technician"};

// The bus as the receiver sees it: frames 15 ms apart, controller polls
// every 24 ms in between unless the controller is silent.
struct Wire {
  PlanScreen scr;
  uint32_t now = 1000;
  bool changed = false;
  void send(const Bytes &f) {
    for (size_t i = 0; i < f.size(); i++)
      changed |= scr.feed(f[i], i == 0 ? 1 : 0, now);
    now += 15;
  }
  void polls(uint32_t ms, uint8_t to = SCR_TERM_ESP) {
    for (uint32_t end = now + ms; now + 24 <= end;) {
      now += 24;
      send(poll(to));
      now -= 15;
    }
  }
  void silence(uint32_t ms) { now += ms; }
  void init(uint8_t a, const char *const rows[8], size_t upto = 8) {  // R-SE-05
    send(disp(a, 0x66, {0x00, 0x01}));
    send(disp(a, 0x65, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}));
    send(disp(a, 0x0D, {0, 0, 0}));
    send(disp(a, 0x0E, {0x80, 0, 0}));
    send(disp(a, 0x0F, {0x00}));
    for (size_t r = 0; r < upto; r++)
      send(row_frame(a, static_cast<uint8_t>(r), rows[r]));
  }
  // First instant (ms after now, polls running) at which the terminal settles.
  int settle_after(uint8_t a, uint32_t limit_ms = 3000) {
    for (uint32_t waited = 0; waited <= limit_ms; waited += 24) {
      if (scr.settled(a, now, NAV_QUIET_MS))
        return static_cast<int>(waited);
      send(poll(SCR_TERM_ESP));
      now += 24 - 15;
    }
    return -1;
  }
};

static std::string emitted_io(const PlanScreen &scr, uint8_t a) {
  const char *rows[FIELDS_ROWS];
  for (size_t r = 0; r < FIELDS_ROWS; r++)
    rows[r] = scr.row(a, static_cast<int>(r));
  std::string out;
  extract_io("D", rows, [&](const char *, const char *name, const char *val, const char *) {
    out += std::string(name) + "=" + val + ";";
  });
  return out;
}

// Healthy bus: the gate adds nothing -- settled exactly 600 ms after the
// last display frame (the owner's rule: the walk never gets slower).
static void test_healthy_settle_unchanged() {
  Wire w;
  w.polls(100);
  w.init(SCR_TERM_ESP, ANCHOR);
  uint32_t last = w.now - 15;  // the last row's stamp
  assert(!w.scr.settled(SCR_TERM_ESP, w.now, NAV_QUIET_MS));
  int t = w.settle_after(SCR_TERM_ESP);
  assert(t >= 0 && w.now - last >= NAV_QUIET_MS && w.now - last < NAV_QUIET_MS + 24);
  assert(std::strcmp(w.scr.row(SCR_TERM_ESP, 2), ANCHOR[2]) == 0);
  // R-SE-06: 0x65 + a delta keeps the other rows (no pending repaint).
  w.send(disp(SCR_TERM_ESP, 0x65, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}));
  w.send(row_frame(SCR_TERM_ESP, 2, "    Hotwater:   36.0\xDF" "C"));
  assert(w.scr.pending(SCR_TERM_ESP) == 0 && std::strcmp(w.scr.row(SCR_TERM_ESP, 3), ANCHOR[3]) == 0);
  assert(w.settle_after(SCR_TERM_ESP) >= 0);
  // A blinking 0x0E (alarm LED, R-SE-07) carries no text and never starves it.
  w.send(row_frame(SCR_TERM_ESP, 2, "    Hotwater:   36.1\xDF" "C"));
  last = w.now - 15;
  for (int i = 0; i < 4; i++) {
    w.polls(250);
    w.send(disp(SCR_TERM_ESP, 0x0E, {0x00, static_cast<uint8_t>(i % 2 ? 0x08 : 0x00), 0x00}));
  }
  assert(w.scr.settled(SCR_TERM_ESP, w.now, NAV_QUIET_MS) && w.now - last >= 1000);
  // A display-only stream (no link layer, e.g. a filtered replay) settles on quiet alone.
  PlanScreen bare;
  Bytes f = row_frame(SCR_TERM_ESP, 0, ANCHOR[0]);
  for (size_t i = 0; i < f.size(); i++)
    bare.feed(f[i], i == 0, 5000);
  assert(bare.settled(SCR_TERM_ESP, 5600, NAV_QUIET_MS) && !bare.settled(SCR_TERM_ESP, 5599, NAV_QUIET_MS));
  std::printf("ok   healthy settle: +%d ms after the quiet window, 0x0E blink ignored\n", t);
}

// W-01 (R-DI-22, R-DI-26a): a page turn cut by a link fault. The controller
// goes silent (link fault, ~2 s) -- that silence is not a settle -- and the
// FF-walk that follows drops every claim: the rows wait for the re-init.
static void test_w01_link_fault_cut_page() {
  Wire w;
  w.polls(100);
  w.init(SCR_TERM_ESP, D02);
  assert(w.settle_after(SCR_TERM_ESP) >= 0);
  w.send(disp(SCR_TERM_ESP, 0x65, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}));
  w.send(cell(SCR_TERM_ESP, 0, 21, '3'));
  w.send(row_frame(SCR_TERM_ESP, 3, "B6 =Outside temp.:"));
  for (int i = 0; i < 20; i++) {  // 2 s of controller silence
    w.silence(100);
    assert(!w.scr.settled(SCR_TERM_ESP, w.now, NAV_QUIET_MS));
  }
  // Today's read would have been b6 = 20.7 (B4's value under B6's label).
  w.send(rollcall(0x02, 0xFFFFFFFF, 0));  // FF-walk: no claims
  assert(w.scr.pending(SCR_TERM_ESP) == 0xFF && w.scr.resessions(SCR_TERM_ESP) == 2);
  assert(emitted_io(w.scr, SCR_TERM_ESP).empty());
  for (int r = 0; r < 8; r++)
    assert(w.scr.row(SCR_TERM_ESP, r)[0] == '\0');
  w.polls(3000);  // polls resume (re-joined) but no repaint yet: still not readable
  assert(!w.scr.settled(SCR_TERM_ESP, w.now, NAV_QUIET_MS));
  static const char *D03[8] = {" Input/Output      D03", "", "", "B6 =Outside temp.:",
                               "    temp.:      12.0\xDF" "C", "", "", ""};
  w.init(SCR_TERM_ESP, D03);
  assert(w.settle_after(SCR_TERM_ESP) >= 0);
  assert(emitted_io(w.scr, SCR_TERM_ESP) == "b6=12.0;");
  std::printf("ok   W-01 page turn cut by a link fault: no b6 from B4's row, re-init reads 12.0\n");
}

// W-01 (R-DI-15, A3 DV-6): a session init cut after row 5 (the link
// survives, the controller moves on): the old page's rows 6-7 must not be
// read under the new page's header, however long the quiet.
static void test_w01_cut_session_init() {
  Wire w;
  w.polls(100);
  w.init(SCR_TERM_ESP, MENU);
  assert(w.settle_after(SCR_TERM_ESP) >= 0);
  w.init(SCR_TERM_ESP, ANCHOR, 6);  // 66 65 0D 0E 0F rows 0..5, then nothing
  w.polls(5000);
  assert(!w.scr.settled(SCR_TERM_ESP, w.now, NAV_QUIET_MS));
  assert(w.scr.pending(SCR_TERM_ESP) == 0xC0);
  assert(w.scr.row(SCR_TERM_ESP, 7)[0] == '\0' && std::strcmp(w.scr.raw_row(SCR_TERM_ESP, 7), "F.Technician          ") == 0);
  w.send(row_frame(SCR_TERM_ESP, 6, ANCHOR[6]));
  w.send(row_frame(SCR_TERM_ESP, 7, ANCHOR[7]));
  assert(w.settle_after(SCR_TERM_ESP) >= 0 && std::strcmp(w.scr.row(SCR_TERM_ESP, 7), ANCHOR[7]) == 0);
  std::printf("ok   W-01 cut session init: not settled until rows 6-7 repainted\n");
}

// W-02 (R-DI-23): a row frame lost at our receiver, then the controller's
// one-cell delta against ITS shadow: 35.9 + lost 36.0 + '1' would read 35.1.
static void test_w02_lost_row_then_cell() {
  Wire w;
  w.polls(100, SCR_TERM_PGD);
  w.init(SCR_TERM_PGD, ANCHOR);
  Bytes bad = row_frame(SCR_TERM_PGD, 2, "    Hotwater:   36.0\xDF" "C");
  bad[22] ^= 0x01;  // a flipped bit inside the value: checksum fails
  w.send(bad);
  assert(w.scr.lost() == 1 && w.scr.row(SCR_TERM_PGD, 2)[0] == '\0');
  w.send(cell(SCR_TERM_PGD, 2, 19, '1'));
  assert(w.scr.row(SCR_TERM_PGD, 2)[0] == '\0');
  assert(std::strstr(w.scr.raw_row(SCR_TERM_PGD, 2), "35.1") != nullptr);  // the forged value
  // the other rows stay readable: only the hit row waits for its repaint
  assert(std::strcmp(w.scr.row(SCR_TERM_PGD, 3), ANCHOR[3]) == 0);
  w.polls(700, SCR_TERM_PGD);
  assert(w.scr.settled(SCR_TERM_PGD, w.now, NAV_QUIET_MS));
  w.send(row_frame(SCR_TERM_PGD, 2, "    Hotwater:   36.2\xDF" "C"));
  assert(std::strcmp(w.scr.row(SCR_TERM_PGD, 2), "    Hotwater:   36.2\xDF" "C") == 0);

  // A lost bit9 mark merges two row frames into one run: the intact head
  // is applied, the swallowed frame's row is distrusted.
  Bytes merged = row_frame(SCR_TERM_PGD, 3, "    Heating:    30.2\xDF" "C");
  Bytes next = row_frame(SCR_TERM_PGD, 4, "    Outside:    12.1\xDF" "C");
  merged.insert(merged.end(), next.begin(), next.end());
  for (size_t i = 0; i < merged.size(); i++)
    w.scr.feed(merged[i], i == 0, w.now);
  w.send(poll(SCR_TERM_PGD));
  assert(std::strstr(w.scr.row(SCR_TERM_PGD, 3), "30.2") != nullptr);
  assert(w.scr.row(SCR_TERM_PGD, 4)[0] == '\0');

  // A transport drop (ring overflow) may have hidden any repaint.
  w.scr.lost_bytes();
  for (int r = 0; r < 8; r++)
    assert(w.scr.row(SCR_TERM_PGD, r)[0] == '\0');
  std::printf("ok   W-02 lost row + later cell: row unknown until repainted (raw would read 35.1)\n");
}

// W-04 (R-DI-07): a 20-column row (LEN 0x1A) never reads as full text.
static void test_w04_clipped_rows() {
  Wire w;
  w.polls(100);
  w.init(SCR_TERM_ESP, ANCHOR);
  w.send(row_frame(SCR_TERM_ESP, 7, "             Alarm-On", 20));
  assert(w.scr.row(SCR_TERM_ESP, 7)[0] == '\0');
  assert(std::strcmp(w.scr.raw_row(SCR_TERM_ESP, 7), "             Alarm-O  ") == 0);
  assert(w.scr.pending(SCR_TERM_ESP) == 0);  // the repaint debt is paid, the text is not trusted
  w.send(row_frame(SCR_TERM_ESP, 7, "             Alarm-On"));
  assert(std::strcmp(w.scr.row(SCR_TERM_ESP, 7), "             Alarm-On ") == 0);
  std::printf("ok   W-04 20-column row is not published as 'Alarm-O'\n");
}

// W-05 (R-DI-27): a screen that never goes quiet. The keyless settle (the
// emit's) fails at the 6 s cap instead of reading; a keyed step reports it.
struct StepProbe : NavEngine {
  explicit StepProbe(const PlanScreen &s) : NavEngine(s) {}
  Act emit_settle(uint32_t now) { return step_(0, [] { return true; }, 0, now); }
  Act keyed(uint32_t now) { return step_(KEY_DOWN, [] { return true; }, 0, now); }
  using NavEngine::Act;
};
static void test_w05_settle_cap_fails() {
  Wire w;
  w.polls(100);
  w.init(SCR_TERM_ESP, D02);
  StepProbe nav(w.scr);
  int errors = 0;
  nav.set_log([&](bool err, const char *) { errors += err; });
  StepProbe::Act a = nav.emit_settle(w.now);
  uint32_t t0 = w.now;
  for (int i = 0; a == StepProbe::Act::RUN && i < 100; i++) {  // a frame every 400 ms
    w.send(row_frame(SCR_TERM_ESP, 4, i % 2 ? "    temp.:      20.7\xDF" "C" : "    temp.:      51.7\xDF" "C"));
    w.polls(385);
    a = nav.emit_settle(w.now);
  }
  assert(a == StepProbe::Act::FAIL && w.now - t0 >= NAV_SETTLE_CAP_MS && w.now - t0 < NAV_SETTLE_CAP_MS + 400);
  assert(nav.settle_caps() == 1 && errors == 1);
  a = nav.keyed(w.now);  // press
  for (int i = 0; a == StepProbe::Act::RUN && i < 100; i++) {
    w.send(row_frame(SCR_TERM_ESP, 4, "    temp.:      20.7\xDF" "C"));
    w.polls(385);
    a = nav.keyed(w.now);
  }
  assert(a == StepProbe::Act::OK && nav.settle_caps() == 2);  // no re-press loop, but reported
  std::printf("ok   W-05 settle cap: emit settle FAILs at 6 s, nothing read\n");
}

// W-05 end to end: a scrape cycle whose emit never settles publishes nothing.
static void test_w05_scrape_emits_nothing() {
  Wire w;
  w.polls(100);
  w.init(SCR_TERM_ESP, ANCHOR);
  static const ScrapeStep ROUTE[] = {{0, 0, false, NEXP_NONE, 0, nullptr, nullptr, true, 0}};
  PlanNav nav(w.scr, ROUTE, 1);
  int emits = 0, presses = 0;
  nav.set_emit([&] { emits++; });
  nav.set_press([&](uint8_t) { presses++; });
  nav.set_interval_ms(600000);
  nav.enable(w.now);
  for (int i = 0; i < 40 && nav.fails() == 0; i++) {
    w.send(cell(SCR_TERM_ESP, 2, 19, static_cast<char>('0' + i % 10)));
    w.polls(385);
    nav.tick(w.now, true);
  }
  assert(nav.fails() == 1 && emits == 0);
  std::printf("ok   W-05 scrape: a never-settled emit fails the cycle, 0 emits (%d recovery presses)\n", presses);
}

// W-06 (R-DI-24): the controller dropped the pGD (claims without 32): its
// frozen screen must not re-publish on an unrelated graphic frame.
static void test_w06_frozen_terminal() {
  Wire w;
  w.polls(100, SCR_TERM_PGD);
  w.init(SCR_TERM_PGD, D02);
  w.send(rollcall(0x1E, bit(32) | bit(31) | bit(1), bit(32) | bit(31)));  // gap walk, both served
  assert(w.scr.pending(SCR_TERM_PGD) == 0 && w.scr.row(SCR_TERM_PGD, 4)[0] != '\0');
  w.send(rollcall(0x1E, bit(32) | bit(31) | bit(1), bit(31)));  // pGD locked out
  assert(w.scr.pending(SCR_TERM_PGD) == 0xFF && w.scr.pending(SCR_TERM_ESP) == 0);
  Bytes icon{0x02, 0xB0, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x10, 0x00, 0x12, 0x00, 0x10, 0x00, 0x00};
  w.send(disp(SCR_TERM_PGD, 0x64, icon));
  w.polls(1000, SCR_TERM_ESP);
  assert(!w.scr.settled(SCR_TERM_PGD, w.now, NAV_QUIET_MS) && emitted_io(w.scr, SCR_TERM_PGD).empty());
  std::printf("ok   W-06 frozen pGD: no b4 from the stale screen after an icon frame\n");
}

// W-07 (R-DI-13): the edit cursor names the row whose value is a candidate.
static void test_w07_edit_focus() {
  Wire w;
  w.polls(100, SCR_TERM_PGD);
  w.init(SCR_TERM_PGD, ANCHOR);
  assert(w.scr.focus_row(SCR_TERM_PGD) == -1);
  w.send(disp(SCR_TERM_PGD, 0x0D, {0x00, 0x00, 0x01}));  // page with editable fields
  assert(w.scr.focus_row(SCR_TERM_PGD) == -1);
  w.send(disp(SCR_TERM_PGD, 0x0D, {0x04, 0x11, 0x01}));  // Enter: focus on row 4
  w.send(row_frame(SCR_TERM_PGD, 4, "Heating:        12.0\xDF" "C"));
  assert(w.scr.focus_row(SCR_TERM_PGD) == 4);
  w.send(disp(SCR_TERM_PGD, 0x0D, {0x00, 0x00, 0x01}));  // Esc / commit
  assert(w.scr.focus_row(SCR_TERM_PGD) == -1);
  std::printf("ok   W-07 edit focus tracked (focus_row) for the passive publisher\n");
}

// W-10: the open-probe reading is not a temperature.
static void test_w10_open_probe() {
  const char *rows[FIELDS_ROWS] = {" Input/Output      D05", "", "", "B9 =Discharge comp.",
                                   "              -204.8\xDF" "C", "B10=Suction",
                                   "               12.3\xDF" "C", ""};
  std::string out;
  extract_io("D05", rows, [&](const char *, const char *name, const char *val, const char *) {
    out += std::string(name) + "=" + val + ";";
  });
  assert(out == "b9=nan;b10=12.3;");
  std::printf("ok   W-10 B9 open probe -> nan (unavailable), not -204.8\n");
}

int main() {
  test_healthy_settle_unchanged();
  test_w01_link_fault_cut_page();
  test_w01_cut_session_init();
  test_w02_lost_row_then_cell();
  test_w04_clipped_rows();
  test_w05_settle_cap_fails();
  test_w05_scrape_emits_nothing();
  test_w06_frozen_terminal();
  test_w07_edit_focus();
  test_w10_open_probe();
  std::printf("ok\n");
  return 0;
}
