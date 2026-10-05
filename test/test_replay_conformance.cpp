// Replay conformance: PlanTerminal (the exact code the RX ISR runs) replayed
// over recorded bus captures, byte by byte, and compared with what the bridge
// really transmitted, with a pGD-faithful reference terminal, and (to keep
// the reference honest) the reference against the real pGD. See
// test/replay/conformance.h for the method, test/replay/fixtures/ for the
// captures (each file's header names its source capture, sha256 and the
// ekobeescope command that produced it).
//
// Every divergence is an EXPECTATION in test/replay/expected.tsv, one row
// per (fixture, comparison, class, tag) with its count and first source
// line. The test passes on today's code and fails on any change of the
// table, so a firmware fix (or regression) shows up in review as a diff of
// that file. Run:
//   c++ -std=c++17 test/test_replay_conformance.cpp -o /tmp/t && /tmp/t
//   REPLAY_UPDATE=1 /tmp/t   # rewrite expected.tsv after an intended change
//   /tmp/t --report          # print every divergence, slot by slot

#include "replay/conformance.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <string>
#include <vector>

using namespace replay;

// Order = table order. Add a fixture: generate it (header + `ekobeescope
// frames --text --live`), list it here, run with REPLAY_UPDATE=1, review.
static const char *const FIXTURES[] = {
    "jul17-soak-join",      "jul17-soak-steady",     "jul17-nofwd-join",   "oct02-passive-ffwalk",
    "oct02-passive-gapwalk", "oct05-enrolled-loop",  "oct05-passive-loop",
};

static std::string find_dir() {
  for (const char *d : {"test/replay", "replay", "../test/replay"}) {
    std::ifstream probe(std::string(d) + "/fixtures/" + FIXTURES[0] + ".frames.txt");
    if (probe)
      return d;
  }
  return "";
}

static std::vector<std::string> read_rows(const std::string &path) {
  std::vector<std::string> rows;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line))
    if (!line.empty() && line[0] != '#')
      rows.push_back(line);
  return rows;
}

int main(int argc, char **argv) {
  const bool report = argc > 1 && std::string(argv[1]) == "--report";
  const bool update = std::getenv("REPLAY_UPDATE") != nullptr;
  const std::string dir = find_dir();
  if (dir.empty()) {
    std::fprintf(stderr, "replay fixtures not found (run from the repo root)\n");
    return 1;
  }

  std::vector<std::string> rows;
  int failures = 0;
  for (const char *name : FIXTURES) {
    Fixture fx;
    if (!load_fixture(dir + "/fixtures/" + name + ".frames.txt", name, fx) || fx.frames.empty()) {
      std::fprintf(stderr, "%s: cannot load fixture\n", name);
      return 1;
    }
    FixtureResult res = run_fixture(fx);
    // Harness invariant, not a firmware property: every recorded
    // transmission must find the frame it answered.
    if (res.unaligned != 0) {
      std::fprintf(stderr, "%s: recorded TX without a trigger: %s\n", name, res.harness.c_str());
      failures++;
    }
    rows.push_back(std::string(name) + "\tharness\t" + res.harness);
    for (const Row &r : tabulate(name, res.divs))
      rows.push_back(row_line(r));
    if (report)
      for (const Div &d : res.divs)
        if (d.cls != "same")
          std::printf("%s\t%s\t%s\t%s\tL%d %s\tslot[%s]\tplanterm[%s]\tother[%s]\trule=%s\n", name,
                      d.cmp.c_str(), d.cls.c_str(), d.tag.c_str(), d.line, fmt_ms(d.t_ms).c_str(),
                      d.slot.c_str(), d.p.c_str(), d.o.c_str(), d.rule.c_str());
  }

  // The comparison is not vacuous: the enrolled loop replayed with the bridge
  // passive must report every recorded transmission as missing.
  {
    Fixture fx;
    load_fixture(dir + "/fixtures/oct05-enrolled-loop.frames.txt", "mutant", fx);
    fx.meta["task"] = "enroll=0 fwd=1 tx_mode=2";
    int missing = 0, same = 0;
    for (const Div &d : run_fixture(fx).divs)
      if (d.cmp == "vs-recorded") {
        missing += d.cls == "missing";
        same += d.cls == "same";
      }
    if (missing != 45 || same != 0) {
      std::fprintf(stderr, "mutant (passive replay of an enrolled capture): missing=%d same=%d, want 45/0\n",
                   missing, same);
      failures++;
    }
  }

  const std::string path = dir + "/expected.tsv";
  if (update) {
    std::ofstream out(path);
    out << "# Replay-conformance expectations: test/test_replay_conformance.cpp (REPLAY_UPDATE=1 rewrites).\n"
           "# fixture\tcomparison\tclass\ttag\tcount\tfirst slot (source line of the answered frame)\n"
           "# vs-recorded: planterm replay vs the bridge's recorded TX. vs-pgd-model: vs the pGD-faithful\n"
           "# reference at 0x1F (A6 model + R-RC-12/R-RC-30). walk-detector: link_reset_/tel_walks_ vs every\n"
           "# FF-walk start. pgd-model-vs-pgd: the reference at 0x20 vs the real pGD (calibration).\n"
           "# counterfactual-vs-pgd-model: replayed with the fixture's `# counterfactual:` task state.\n"
           "# class is from planterm's side (missing: the other side sends, planterm does not).\n";
    for (const std::string &r : rows)
      out << r << "\n";
    std::printf("wrote %s (%zu rows)\n", path.c_str(), rows.size());
    return failures ? 1 : 0;
  }

  const std::vector<std::string> want = read_rows(path);
  const std::set<std::string> want_set(want.begin(), want.end()), got_set(rows.begin(), rows.end());
  int diffs = 0;
  for (const std::string &w : want)
    if (!got_set.count(w)) {
      std::fprintf(stderr, "- %s\n", w.c_str());
      diffs++;
    }
  for (const std::string &g : rows)
    if (!want_set.count(g)) {
      std::fprintf(stderr, "+ %s\n", g.c_str());
      diffs++;
    }
  if (diffs) {
    std::fprintf(stderr,
                 "replay conformance table changed (%d rows): review, then REPLAY_UPDATE=1 to accept\n", diffs);
    failures++;
  }
  if (failures)
    return 1;
  std::printf("ok\n");
  return 0;
}
