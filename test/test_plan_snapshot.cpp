// Host-compilable self-check for the screen-snapshot cache: latest valid
// display frame per (terminal, row) survives, garble and non-display frames
// don't, and the visit order carries the trailer flag for 0x20 frames.
//   c++ -std=c++17 test/test_plan_snapshot.cpp -o /tmp/t && /tmp/t

#include "../src/plan_snapshot.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace plan;

// Build a text-row display frame ADDR 0B LEN 01 ROW <text> CK (sum 0xFF).
static std::vector<uint8_t> text_frame(uint8_t addr, uint8_t row, const char *text) {
  std::vector<uint8_t> f = {addr, 0x0B, 0, 0x01, row};
  for (const char *p = text; *p; p++)
    f.push_back(static_cast<uint8_t>(*p));
  f[2] = static_cast<uint8_t>(f.size() + 1);
  uint8_t s = 0;
  for (uint8_t b : f)
    s += b;
  f.push_back(static_cast<uint8_t>(0xFF - s));
  return f;
}

// Feed a frame (bit9 on the first byte) followed by a poll token, whose
// bit9-marked address byte closes the frame -- like the real bus.
static void feed_frame(PlanSnapshot &snap, const std::vector<uint8_t> &f, uint32_t ms) {
  for (size_t i = 0; i < f.size(); i++)
    snap.feed(f[i], i == 0 ? 1 : 0, ms);
  const uint8_t poll[4] = {0x1F, 0x01, 0x01, 0xDE};
  for (size_t i = 0; i < 4; i++)
    snap.feed(poll[i], i == 0 ? 1 : 0, ms);
}

struct Seen {
  std::vector<uint8_t> bytes;
  uint32_t at_ms;
  bool trailer;
};

static std::vector<Seen> collect(const PlanSnapshot &snap) {
  std::vector<Seen> out;
  snap.visit([&](const uint8_t *b, size_t n, uint32_t ms, bool trailer) {
    out.push_back({std::vector<uint8_t>(b, b + n), ms, trailer});
  });
  return out;
}

int main() {
  PlanSnapshot snap;
  assert(collect(snap).empty());

  // pGD row 2, then own-session row 0: cached under their terminals, right
  // trailer flag
  auto pgd2 = text_frame(0x20, 2, "    Hotwater:   39.0");
  auto esp0 = text_frame(0x1F, 0, " On/Off Unit       A01");
  feed_frame(snap, pgd2, 1000);
  feed_frame(snap, esp0, 2000);
  auto got = collect(snap);
  assert(got.size() == 2);
  assert(got[0].bytes == pgd2 && got[0].trailer && got[0].at_ms == 1000);
  assert(got[1].bytes == esp0 && !got[1].trailer && got[1].at_ms == 2000);

  // a repaint of the same row replaces, never duplicates
  auto pgd2b = text_frame(0x20, 2, "    Hotwater:   41.5");
  feed_frame(snap, pgd2b, 3000);
  got = collect(snap);
  assert(got.size() == 2 && got[0].bytes == pgd2b && got[0].at_ms == 3000);

  // garble (bad checksum) must never replace a cached frame
  auto bad = text_frame(0x20, 2, "GARBLE");
  bad[5] ^= 0xFF;
  feed_frame(snap, bad, 4000);
  got = collect(snap);
  assert(got.size() == 2 && got[0].bytes == pgd2b);

  // polls, keypad reports, graphics (overlong), foreign addresses: ignored
  {
    PlanSnapshot s2;
    const uint8_t key[7] = {0x01, 0x1E, 0x07, 0x20, 0x0E, 0x01, 0xAA};
    for (size_t i = 0; i < 7; i++)
      s2.feed(key[i], i == 0 ? 1 : 0, 1);
    std::vector<uint8_t> graphic = {0x20, 0x64, 0x95, 0x01};
    graphic.resize(0x95, 0xFF);  // 149-byte bitmap frame, runs past SNAP_FRAME_MAX
    feed_frame(s2, graphic, 1);
    feed_frame(s2, text_frame(0x05, 1, "foreign"), 1);
    assert(collect(s2).empty());
    // ...and an overlong run must not poison the NEXT frame
    feed_frame(s2, esp0, 5);
    assert(collect(s2).size() == 1);
  }

  // a frame split across feed chunks (mid-frame silence) still caches once
  // its closing bit9 mark arrives; a partial run at start never caches
  {
    PlanSnapshot s3;
    for (size_t i = 3; i < pgd2.size(); i++)
      s3.feed(pgd2[i], 0, 1);  // joined mid-frame: headless fragment
    feed_frame(s3, esp0, 2);
    got = collect(s3);
    assert(got.size() == 1 && got[0].bytes == esp0);
  }

  // 0x0C numeric field frames cache per selector
  {
    PlanSnapshot s4;
    std::vector<uint8_t> var = {0x20, 0x0C, 0x08, 0x01, 0x04, 0x13, 0x36, 0};
    uint8_t s = 0;
    for (size_t i = 0; i < 7; i++)
      s += var[i];
    var[7] = static_cast<uint8_t>(0xFF - s);
    feed_frame(s4, var, 7);
    got = collect(s4);
    assert(got.size() == 1 && got[0].bytes == var && got[0].trailer);

    // ...and a later full repaint of the same row evicts the cached cell:
    // replaying stale drift onto a new page painted ghost digits.
    auto row4 = text_frame(0x20, 4, "B.Setpoint");
    feed_frame(s4, row4, 8);
    got = collect(s4);
    assert(got.size() == 1 && got[0].bytes == row4);

    // drift arriving AFTER the row repaint is valid again and replays
    feed_frame(s4, var, 9);
    got = collect(s4);
    assert(got.size() == 2 && got[0].bytes == row4 && got[1].bytes == var);

    // a repaint of a DIFFERENT row must not evict it
    feed_frame(s4, text_frame(0x20, 2, "unrelated"), 10);
    got = collect(s4);
    assert(got.size() == 3 && got[2].bytes == var);
  }

  std::printf("ok\n");
  return 0;
}
