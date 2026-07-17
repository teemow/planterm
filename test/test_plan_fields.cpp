// Host-compilable self-check for the field-extraction engine: every matcher
// kind (FxKind), page identity, the content-driven Input/Output scan, and
// the extraction runner, exercised over a consumer-style spec table against
// verbatim page dumps captured from the reference device (docs/protocol.md
// provenance). The full device spec table and its content assertions live
// with the consumer firmware; this test owns the engine.
//   c++ -std=c++17 test/test_plan_fields.cpp -o /tmp/t && /tmp/t

#include "../src/plan_fields.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace plan;

// A consumer-style spec table covering every matcher kind. The rows mirror
// entries of the reference device's table (the full table stays with the
// consumer); what is under test here is the matchers, not the contents.
static const FieldSpec SPECS[] = {
    // FX_LABEL_NUM unanchored + FX_TEXT unanchored (status anchor)
    {"status", 2, FX_LABEL_NUM, false, "Hotwater:", "\xDF", "hotwater", "°C"},
    {"status", 7, FX_TEXT, false, "", "", "status", ""},
    // FX_WORD (the row is one word)
    {"A01", 4, FX_WORD, false, "", "", "mode", ""},
    // FX_LABEL_NUM anchored
    {"B01", 4, FX_LABEL_NUM, true, "Heating:", "\xDF", "heating_setpoint", "°C"},
    // FX_LABEL_TOK
    {"C01", 5, FX_LABEL_TOK, true, "Date:", "", "clock_date", ""},
    // FX_TIMER_STATE / FX_TIMER_HH / FX_TIMER_MI (scheduler slot row)
    {"C02", 4, FX_TIMER_STATE, true, "F1 ", "", "timer_f1_state", ""},
    {"C02", 4, FX_TIMER_HH, true, "F1 ", "", "timer_f1_hh", ""},
    {"C02", 4, FX_TIMER_MI, true, "F1 ", "", "timer_f1_mi", ""},
    // FX_LABEL_SEP_WORD (label may itself carry spaces before the colon)
    {"Gg01", 3, FX_LABEL_SEP_WORD, true, "NO2 Heatsourcepump", "", "manual_no2", ""},
    // FX_LABEL_NUM with unit suffixes, FX_NUM, FX_NUM_LEAD, FX_WORD_PREFIX
    {"D14", 1, FX_LABEL_NUM, false, "SH:", "K", "superheat", "K"},
    {"D14", 2, FX_NUM_LEAD, false, "", "\xDF", "suction_temp", "°C"},
    {"D14", 4, FX_NUM, false, "", "stp", "eev_steps", "stp"},
    {"D14", 5, FX_NUM_LEAD, false, "", "%", "eev_pos", "%"},
    {"D14", 7, FX_WORD_PREFIX, false, "", "", "valve_state", ""},
    {"D14", 7, FX_NUM, false, "", "\xDF", "evap_temp", "°C"},
    // FX_TEXT anchored + FX_LABEL_NUM2 (second number on the row)
    {"alarm", 2, FX_TEXT, true, "", "", "alarm", ""},
    {"alarm", 4, FX_LABEL_NUM, true, "Plant:", "\xDF", "plant_inlet", "°C"},
    {"alarm", 4, FX_LABEL_NUM2, true, "Plant:", "", "plant_outlet", "°C"},
};
static constexpr size_t SPECS_N = sizeof(SPECS) / sizeof(SPECS[0]);

struct F {
  std::string page, name, value, unit;
  bool operator==(const F &o) const {
    return page == o.page && name == o.name && value == o.value && unit == o.unit;
  }
};

// Run the spec table plus the Input/Output scan over up-to-8 dump lines
// (row 0 first, missing rows "") -- the consumer-side wiring: the app
// decides which screens get the content-driven scan (their row-0 title).
static std::vector<F> extract(std::vector<const char *> lines) {
  const char *rows[FIELDS_ROWS];
  for (size_t i = 0; i < FIELDS_ROWS; i++)
    rows[i] = i < lines.size() ? lines[i] : "";
  std::vector<F> out;
  auto emit = [&](const char *page, const char *name, const char *value, const char *unit) {
    out.push_back({page, name, value, unit});
  };
  extract_fields(SPECS, SPECS_N, rows, emit);
  if (std::strstr(rows[0], "Input/Output") != nullptr) {
    char page[FIELDS_PAGE_MAX];
    page_of(rows, page);
    extract_io(page, rows, emit);
  }
  return out;
}

static void expect(const char *tag, const std::vector<F> &got, const std::vector<F> &want) {
  if (got == want)
    return;
  std::printf("FAIL %s:\n", tag);
  std::printf(" got:\n");
  for (const F &f : got)
    std::printf("  {%s %s %s %s}\n", f.page.c_str(), f.name.c_str(), f.value.c_str(), f.unit.c_str());
  std::printf(" want:\n");
  for (const F &f : want)
    std::printf("  {%s %s %s %s}\n", f.page.c_str(), f.name.c_str(), f.value.c_str(), f.unit.c_str());
  std::exit(1);
}

static std::string page(std::vector<const char *> lines) {
  const char *rows[FIELDS_ROWS];
  for (size_t i = 0; i < FIELDS_ROWS; i++)
    rows[i] = i < lines.size() ? lines[i] : "";
  char out[FIELDS_PAGE_MAX];
  page_of(rows, out);
  return out;
}

int main() {
  expect("status (unanchored label + text)",
         extract({"02:18 03/07/26 Ekobee1", "", "    Hotwater:   29.5\xDF" "C",
                  "    OutsideT:   16.8\xDF" "C", "", "", "             STATUS:",
                  "             Auto-Off"}),
         {{"status", "hotwater", "29.5", "°C"}, {"status", "status", "Auto-Off", ""}});

  expect("A01 word",
         extract({" On/Off Unit       A01", "", "Heat Pump unit", "", "AUTO", "",
                  "  Heizen+Warmwasser"}),
         {{"A01", "mode", "AUTO", ""}});

  expect("B01 anchored label+num",
         extract({" Thermoreg. Unit   B01", "Heat pump temperature", "",
                  "Nominal setpoint (ON)", "Heating:        10.0\xDF" "C", "",
                  "Energy save setpoint", "Heating:        10.0\xDF" "C"}),
         {{"B01", "heating_setpoint", "10.0", "°C"}});

  expect("C01 label token",
         extract({" Clock             C01", "", "Day:    Friday", "", "",
                  "Date:   03/07/26", "", "Hour:   02:18"}),
         {{"C01", "clock_date", "03/07/26", ""}});

  expect("C02 timer state + hh + mi",
         extract({" Week Timer        C02", " Heatprogram", "Day     MONDAY",
                  "Copy in MONDAY      NO", "F1 06:10   ENERGY SAVE", "F2 08:30   OFF",
                  "F3 16:00   OFF", "F4 22:00   OFF"}),
         {{"C02", "timer_f1_state", "ENERGY SAVE", ""},
          {"C02", "timer_f1_hh", "06", ""},
          {"C02", "timer_f1_mi", "10", ""}});

  expect("Gg01 label:word",
         extract({" Manual mng.      Gg01", "", "", "NO2 Heatsourcepump:AUT", "", "", "", ""}),
         {{"Gg01", "manual_no2", "AUT", ""}});

  expect("D14 num shapes",
         extract({" Valve             D14", "    SH:  -3.7K", "                21.4\xDF" "C",
                  "", " 157stp", "  32%", " EEV:       9.2barg", " Std-by    25.1\xDF" "C"}),
         {{"D14", "superheat", "-3.7", "K"},
          {"D14", "suction_temp", "21.4", "°C"},
          {"D14", "eev_steps", "157", "stp"},
          {"D14", "eev_pos", "32", "%"},
          {"D14", "valve_state", "Std-by", ""},
          {"D14", "evap_temp", "25.1", "°C"}});

  expect("alarm text + second number",
         extract({"Alarms", "", "Sys B/H Int. Alarm", "       Inlet   Outlet",
                  "Plant:  54.0\xDF" "C  45.8\xDF" "C"}),
         {{"alarm", "alarm", "Sys B/H Int. Alarm", ""},
          {"alarm", "plant_inlet", "54.0", "°C"},
          {"alarm", "plant_outlet", "45.8", "°C"}});

  // Input/Output content scan: bare temp, next-row temp, pressure pair,
  // digital outputs, and mixed rows mid-repaint
  expect("IO probes bare + pair-row temp",
         extract({" Input/Output      D01", "Analogue inputs", "",
                  "B6 =Outside temp.:    ", "                16.8\xDF ", "",
                  "B2 =Heat source in    ", "    temp.:      21.7\xDF "}),
         {{"D01", "b6", "16.8", "°C"}, {"D01", "b2", "21.7", "°C"}});

  expect("IO pressure probes",
         extract({" Input/Output      D01", "Analogic inputs", "",
                  "B11 =Condensation:    ", "    9.7barg .   26.7\xDF ", "",
                  "B12 =Evaporation:     ", "    temp.:      21.7\xDF "}),
         {{"D01", "b11_press", "9.7", "barg"},
          {"D01", "b11_temp", "26.7", "°C"},
          {"D01", "b12", "21.7", "°C"}});

  expect("IO digital outputs",
         extract({" Input/Output      D10", "Digital outputs", "",
                  "01=Compres.1:      Off", "02=Heatsourcepump: Off", "",
                  "03=Heating pump  : Off", "04=DHW pump      : Off"}),
         {{"D10", "compres_1", "Off", ""},
          {"D10", "heatsourcepump", "Off", ""},
          {"D10", "heating_pump", "Off", ""},
          {"D10", "dhw_pump", "Off", ""}});

  // probe label whose value row is a digital output (real capture, D walk):
  // probe skipped, dout still extracted
  expect("IO mid-walk mixed rows",
         extract({" Input/Output      D10", "Analogic inputs", "",
                  "B11 =Condensation:    ", "02=Heatsourcepump: Off", "",
                  "03=Heating pump  : Off", "04=DHW pump      : Off"}),
         {{"D10", "heatsourcepump", "Off", ""},
          {"D10", "heating_pump", "Off", ""},
          {"D10", "dhw_pump", "Off", ""}});

  expect("menu: nothing to extract",
         extract({"Main menu          1/8", "", "", "   A.On/Off Unit", "",
                  "   B.Setpoint", "", "   C.Clock/Scheduler"}),
         {});

  // page_of corners
  assert(page({"02:18 03/07/26 Ekobee1"}) == "status");
  assert(page({"  Alarms  "}) == "alarm");
  assert(page({" Service           Gd01"}) == "Gd01");
  assert(page({" Emergency heat.  EcH01"}) == "EcH01");
  assert(page({"Main menu          1/8"}) == "");
  assert(page({""}) == "");
  // digit-middle page IDs (the Power+ Inverter family, live 2026-07-17)
  assert(page({" Power+ Config   H1a01"}) == "H1a01");
  assert(page({" Power+ Custom   H1c14"}) == "H1c14");
  assert(page({" Data logger       E35"}) == "E35");
  assert(page({" EVD Config.     Haa01"}) == "Haa01");
  // glued header: the long title consumes the gap before the ID cell,
  // page identity falls back to the token's page-ID suffix
  assert(page({" Compressor conf.H1a05"}) == "H1a05");
  assert(page({" Compressor conf.H1a06"}) == "H1a06");
  assert(page({"02:18 03/07/26 Ekobee1"}) == "status");  // no accidental tail
  // ...without over-matching menu/status headers
  assert(page({"P.Plus Menu' config   "}) == "");
  assert(page({" Manufacturer         "}) == "");
  assert(page({"Manufacturer menu  1/5"}) == "");

  // find_spec
  assert(find_spec(SPECS, SPECS_N, "mode") == &SPECS[2]);
  assert(find_spec(SPECS, SPECS_N, "nope") == nullptr);

  // snake (planscope TestSnake)
  struct {
    const char *in, *want;
  } snakes[] = {
      {"Compres.1", "compres_1"},
      {"Heatsourcepump", "heatsourcepump"},
      {"Heating pump  ", "heating_pump"},
      {"DHW circ.pump", "dhw_circ_pump"},
      {" Emergencyheater", "emergencyheater"},
  };
  for (auto &s : snakes) {
    char out[FIELDS_NAME_MAX];
    fx_snake(s.in, s.in + std::strlen(s.in), out, sizeof out);
    if (std::strcmp(out, s.want) != 0) {
      std::printf("FAIL snake(%s) = %s, want %s\n", s.in, out, s.want);
      return 1;
    }
  }

  std::printf("ok\n");
  return 0;
}
