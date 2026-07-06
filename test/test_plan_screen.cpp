// Host-compilable self-check for the screen reconstructor: replay an
// archived menu-walk capture through PlanScreen and assert per-settled-page
// invariants -- the C++ port of planscope's replay_test.go:
//   * at most one selection band below the title bar (menu-cursor truth),
//   * the row content of documented pages matching their reference dumps
//     (digit-normalized: live values drift),
//   * transport sanity (the raw-stream frame parser digests the capture).
//
//   c++ -std=c++17 test/test_plan_screen.cpp -o /tmp/t && /tmp/t
//   /tmp/t <capture>            run against a capture file
//   /tmp/t <capture> --live     relaxed run over a fresh hardware capture
//
// The reference capture is not part of this repo; the test exits 0 with a
// "skip" note when no capture is available.

#include "walk_replay.h"

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace plan;
using namespace walk_replay;

// --- page identity (test-local) ---------------------------------------------
// A minimal port of planscope's page identification, enough to key the dump
// table: the app's page ID from row 0 (pageIDRe ^[A-Z][A-Za-z]{0,2}\d{2}$),
// else the synthetic "status" anchor (clockRe ^\d{2}:\d{2} \d{2}/\d{2}/\d{2}),
// else "". The full field-extraction engine lives with its tables, not here.
static constexpr size_t PAGE_MAX = 8;  // "status" + NUL

static bool pg_digit(char c) { return c >= '0' && c <= '9'; }
static bool pg_alpha(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }

static bool pg_page_id(const char *tok, size_t n) {
  if (n < 3 || !(tok[0] >= 'A' && tok[0] <= 'Z'))
    return false;
  size_t i = 1;
  while (i < n && pg_alpha(tok[i]))
    i++;
  return i <= 3 && n - i == 2 && pg_digit(tok[i]) && pg_digit(tok[i + 1]);
}

static bool pg_clock(const char *r) {
  const char *pat = "dd:dd dd/dd/dd";
  for (int i = 0; pat[i] != '\0'; i++) {
    if (pat[i] == 'd' ? !pg_digit(r[i]) : r[i] != pat[i])
      return false;
  }
  return true;
}

// out must hold PAGE_MAX bytes.
static void page_of(const char *r0, char *out) {
  const char *last = nullptr;
  size_t lastn = 0;
  for (const char *p = r0; *p != '\0';) {
    while (*p == ' ')
      p++;
    if (*p == '\0')
      break;
    const char *q = p;
    while (*q != '\0' && *q != ' ')
      q++;
    last = p;
    lastn = static_cast<size_t>(q - p);
    p = q;
  }
  if (last != nullptr && lastn < PAGE_MAX && pg_page_id(last, lastn)) {
    std::memcpy(out, last, lastn);
    out[lastn] = '\0';
    return;
  }
  if (pg_clock(r0)) {
    std::strcpy(out, "status");
    return;
  }
  out[0] = '\0';
}

// --- reference page dumps ----------------------------------------------------
// Verbatim page dumps from the reference unit's documented menu walk (pages
// whose full sub-page set is documented), one string per row, blank tail
// rows omitted. \xDF is the CAREL degree glyph exactly as PlanScreen stores
// it. A "*" row matches any content: the status value on row 7 is an open
// set (Auto-Off, Auto-On, Energy saving, ...) read by a fields layer, not
// enumerable here.
static const std::map<std::string, std::vector<std::vector<const char *>>> PAGE_DUMPS = {
    {"status",  // anchor; row 0 is the clock, no page ID
     {{"02:18 03/07/26 Ekobee1", "", "    Hotwater:   29.5\xDF" "C",
       "    OutsideT:   16.8\xDF" "C", "    System T:   23.9\xDF" "C", "",
       "             STATUS:", "*"}}},
    {"A01",
     {{" On/Off Unit       A01", "", "Heat Pump unit", "", "AUTO", "",
       "  Heizen+Warmwasser"}}},
    {"B01",
     {{" Thermoreg. Unit   B01", "Heat pump temperature", "",
       "Nominal setpoint (ON)", "Heating:        10.0\xDF" "C", "",
       "Energy save setpoint", "Heating:        10.0\xDF" "C"},
      {" Thermoreg. Unit   B01", "Heat pump temperature", "",
       "Nominal setpoint (ON)", "Domestic:       39.0\xDF" "C", "",
       "Energy save setpoint", "Domestic:       20.0\xDF" "C"}}},
    {"C01",
     {{" Clock             C01", "", "Day:            Friday", "",
       "              dd/mm/yy", "Date:         03/07/26", "",
       "Hour:            01:48"}}},
    {"D14",
     {{" Valve             D14", "    SH:  -3.7K", "                21.4\xDF" "C",
       "", " 157stp", "  32%", " EEV:       9.2barg", " Std-by    25.1\xDF" "C"}}},
    {"Ga01",
     {{" Change language  Ga01", "", "", "Language:      ENGLISH", "",
       "   ENTER to change"}}},
    {"Gd01",
     {{" Working hours    Gd01", "", "Compressor  :  019654h", "",
       "Heat source", " pump        : 022786h", "",
       "Heating pump:  030070h"},
      {" Working hours    Gd01", "", "DHW pump:      001223h"}}},
};

// norm_row makes a row comparable against its dump: digits, '.' and '-'
// (values, signs) go, whitespace collapses (planscope normRow).
static std::string norm_row(const char *s) {
  std::string out;
  bool pending_space = false;
  for (const char *p = s; *p != '\0'; p++) {
    char c = *p;
    if ((c >= '0' && c <= '9') || c == '.' || c == '-')
      continue;
    if (c == ' ') {
      pending_space = !out.empty();
      continue;
    }
    if (pending_space)
      out.push_back(' ');
    pending_space = false;
    out.push_back(c);
  }
  return out;
}

// Whether the settled screen is consistent with one of the page's
// documented sub-page dumps; fills `diff` with the first mismatch.
static bool matches_dump(const ReplayScreen &s, const std::vector<std::vector<const char *>> &variants,
                         std::string &diff) {
  for (const auto &v : variants) {
    bool ok = true;
    for (int r = 0; r < static_cast<int>(SCR_ROWS); r++) {
      std::string got = norm_row(s.row(SCR_TERM_ESP, r));
      if (got.empty())
        continue;  // undelivered/blank row: tolerated (log loss)
      if (r < static_cast<int>(v.size()) && std::strcmp(v[r], "*") == 0)
        continue;  // wildcard row: open-set live value
      std::string want = r < static_cast<int>(v.size()) ? norm_row(v[r]) : "";
      if (got != want) {
        ok = false;
        diff = "row " + std::to_string(r) + ": got \"" + got + "\", dump has \"" + want + "\"";
        break;
      }
    }
    if (ok)
      return true;
  }
  return false;
}

static std::string fmt_ts(int ms) {
  char b[16];
  int s = ms / 1000;
  std::snprintf(b, sizeof b, "%02d:%02d:%02d", s / 3600, s / 60 % 60, s % 60);
  return b;
}

int main(int argc, char **argv) {
  ReplayArgs args = parse_args(argc, argv);

  int fails = 0;
  std::map<std::string, int> seen, matched;
  std::vector<std::string> diverged;
  ReplayScreen scr;
  int settled = replay_walk(args.path, scr, [&](int ts, const ReplayScreen &s) {
    // selection invariant: never two lit bands below the title bar
    if (int n = s.inverse_body_bands(); n > 1) {
      std::printf("FAIL %s: %d lit bands below the title bar\n", fmt_ts(ts).c_str(), n);
      fails++;
    }
    // page dumps: settled documented pages must match their dump rows
    char page[PAGE_MAX];
    page_of(s.row(SCR_TERM_ESP, 0), page);
    auto it = PAGE_DUMPS.find(page);
    if (it == PAGE_DUMPS.end())
      return;
    seen[page]++;
    std::string diff;
    if (matches_dump(s, it->second, diff))
      matched[page]++;
    else
      diverged.push_back(fmt_ts(ts) + " " + page + ": " + diff);
  });

  if (settled < 0) {
    std::printf("skip: capture not available at %s\n", args.path);
    return 0;
  }

  // transport sanity: the raw-stream parser digested the capture
  std::printf("%d settled pages, %u frames (%u bad)\n", settled, scr.frames(), scr.bad());
  int min_settled = args.live ? 5 : 50;
  if (settled < min_settled) {
    std::printf("FAIL only %d settled pages -- capture not exercised\n", settled);
    fails++;
  }
  if (scr.frames() < 500) {
    std::printf("FAIL only %u display frames parsed\n", scr.frames());
    fails++;
  }

  // a garbling/duplication bug would never match a dump even once
  for (const auto &[id, n] : seen) {
    std::printf("page %-6s %d/%d settled snapshots match the dump\n", id.c_str(), matched[id], n);
    if (matched[id] == 0) {
      std::printf("FAIL page %s never matched its dump in %d settled snapshots\n", id.c_str(), n);
      fails++;
    }
  }
  // coverage: the reference archive walked the whole menu; a live observe
  // capture only visits the scrape route (status + D14 of the documented
  // dumps)
  std::vector<const char *> required =
      args.live ? std::vector<const char *>{"status", "D14"}
                : std::vector<const char *>{"status", "A01", "B01", "D14"};
  for (const char *id : required) {
    if (seen[id] == 0) {
      std::printf("FAIL page %s never settled in the capture -- dump untested\n", id);
      fails++;
    }
  }
  // stale snapshots (log loss leaves genuinely stale rows the legacy
  // transport cannot flag) stay the rare exception, not the rule
  int total = 0, good = 0;
  for (const auto &[id, n] : seen) {
    total += n;
    good += matched[id];
  }
  if ((total - good) * 5 > total) {
    std::printf("FAIL %d of %d settled snapshots diverge from their dumps:\n", total - good, total);
    for (const auto &d : diverged)
      std::printf("  %s\n", d.c_str());
    fails++;
  }

  if (fails > 0)
    return 1;
  std::printf("ok\n");
  return 0;
}
