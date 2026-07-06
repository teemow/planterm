// Host-compilable self-check for the pLAN frame-boundary oracle and the
// 12-byte display-record parser. No hardware needed. Run with:
//   c++ -std=c++17 test/test_plan_decode.cpp -o /tmp/t && /tmp/t

#include "../src/plan_frame.h"
#include <cassert>
#include <cstdio>
#include <vector>

using plan::DisplayRecord;
using plan::decode_display_record;
using plan::frame_len_at;
using plan::sum8;

// Real frames captured off a live bus (regrouped).
static const uint8_t POLL[9] = {0x20, 0x01, 0x01, 0xDD, 0x01, 0x01, 0x20, 0xDD, 0x01};
static const uint8_t KEEPALIVE[12] = {0x02, 0x02, 0x01, 0x80, 0x00, 0x00,
                                      0x01, 0x80, 0x00, 0x00, 0x00, 0xF9};
static const uint8_t KEYPAD[6] = {0x1E, 0x07, 0x20, 0x0E, 0xC8, 0xE3};
// 20 0C 08 01 04 13 36 7D 01 03 20 DB  -> selector 0x04, value 0x1336
static const uint8_t DISPLAY[12] = {0x20, 0x0C, 0x08, 0x01, 0x04, 0x13,
                                    0x36, 0x7D, 0x01, 0x03, 0x20, 0xDB};

int main() {
  // --- frame_len_at: each real frame type is recognised at its true length ---
  assert(frame_len_at(POLL, 9) == 9);
  assert(frame_len_at(KEYPAD, 6) == 6);
  assert(frame_len_at(KEEPALIVE, 12) == 12);
  assert(frame_len_at(DISPLAY, 12) == 12);
  // Shortest-match must not mis-split a 9/12-byte frame as a 6-byte one.
  assert(frame_len_at(POLL, 6) == 0);      // 6-byte prefix of poll doesn't sum
  assert(frame_len_at(DISPLAY, 6) == 0);   // nor the display frame's
  // Not enough bytes yet -> 0 (wait for more).
  assert(frame_len_at(DISPLAY, 4) == 0);

  // --- decode_display_record: the confirmed 6 real display frames ---
  struct Sample {
    uint8_t bytes[12];
    uint8_t selector;
    uint16_t value;
  };
  const Sample samples[] = {
      {{0x20, 0x0C, 0x08, 0x01, 0x04, 0x13, 0x36, 0x7D, 0x01, 0x03, 0x20, 0xDB}, 0x04, 0x1336},
      {{0x20, 0x0C, 0x08, 0x01, 0x00, 0x04, 0x31, 0x95, 0x01, 0x03, 0x20, 0xDB}, 0x00, 0x0431},
      {{0x20, 0x0C, 0x08, 0x01, 0x02, 0x13, 0x37, 0x7E, 0x01, 0x03, 0x20, 0xDB}, 0x02, 0x1337},
      {{0x20, 0x0C, 0x08, 0x01, 0x04, 0x13, 0x37, 0x7C, 0x01, 0x03, 0x20, 0xDB}, 0x04, 0x1337},
      {{0x20, 0x0C, 0x08, 0x01, 0x02, 0x13, 0x38, 0x7D, 0x01, 0x03, 0x20, 0xDB}, 0x02, 0x1338},
      {{0x20, 0x0C, 0x08, 0x01, 0x03, 0x13, 0x33, 0x81, 0x01, 0x03, 0x20, 0xDB}, 0x03, 0x1333},
  };
  for (const auto &s : samples) {
    DisplayRecord r{};
    assert(decode_display_record(s.bytes, 12, r));
    assert(r.selector == s.selector);
    assert(r.value == s.value);
  }

  // --- rejection cases ---
  DisplayRecord r{};
  assert(!decode_display_record(POLL, 9, r));       // wrong length
  assert(!decode_display_record(KEEPALIVE, 12, r));  // wrong header
  {
    // corrupt the check byte -> sum no longer 0xFF -> reject
    uint8_t bad[12];
    for (int i = 0; i < 12; i++)
      bad[i] = DISPLAY[i];
    bad[7] ^= 0x01;
    assert(!decode_display_record(bad, 12, r));
  }
  {
    // corrupt the trailer -> reject
    uint8_t bad[12];
    for (int i = 0; i < 12; i++)
      bad[i] = DISPLAY[i];
    bad[11] = 0x00;
    assert(!decode_display_record(bad, 12, r));
  }

  // --- end-to-end: split a merged stream, then decode ---
  std::vector<uint8_t> stream;
  auto push = [&](const uint8_t *f, size_t n) { stream.insert(stream.end(), f, f + n); };
  push(POLL, 9);
  push(DISPLAY, 12);
  push(KEEPALIVE, 12);
  push(POLL, 9);
  size_t i = 0, decoded = 0;
  while (i < stream.size()) {
    size_t len = frame_len_at(stream.data() + i, stream.size() - i);
    if (len == 0) {  // resync: drop one byte
      i++;
      continue;
    }
    DisplayRecord rec{};
    if (decode_display_record(stream.data() + i, len, rec)) {
      assert(rec.selector == 0x04 && rec.value == 0x1336);
      decoded++;
    }
    i += len;
  }
  assert(decoded == 1);  // exactly the one DISPLAY frame, no mis-splits

  std::printf("ok\n");
  return 0;
}
