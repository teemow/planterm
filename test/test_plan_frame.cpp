// Host-compilable self-check for the pLAN keypad-frame logic: the
// keypad-report encoder + validator and the poll-token matcher. No hardware
// needed. Run with:
//   c++ -std=c++17 test/test_plan_frame.cpp -o /tmp/t && /tmp/t

#include "../src/plan_frame.h"
#include <cassert>
#include <cstdio>

using namespace plan;

int main() {
  // --- encode_keypad reproduces frames captured from the real pGD ---
  // (Enter held, hold counter ramping, sum == 0xFE)
  struct Sample {
    uint8_t keycode, hold, expect[KEYPAD_LEN];
  };
  const Sample samples[] = {
      {KEY_ENTER, 0x72, {0x1E, 0x07, 0x20, 0x0E, 0x72, 0x39}},
      {KEY_ENTER, 0xC8, {0x1E, 0x07, 0x20, 0x0E, 0xC8, 0xE3}},
      {KEY_ENTER, 0xAC, {0x1E, 0x07, 0x20, 0x0E, 0xAC, 0xFF}},
      {KEY_ENTER, 0xAA, {0x1E, 0x07, 0x20, 0x0E, 0xAA, 0x01}},
  };
  for (const auto &s : samples) {
    uint8_t f[KEYPAD_LEN];
    assert(encode_keypad(s.keycode, s.hold, f) == KEYPAD_LEN);
    for (size_t i = 0; i < KEYPAD_LEN; i++)
      assert(f[i] == s.expect[i]);
    assert(keypad_frame_ok(f, KEYPAD_LEN));
    assert(sum8(f, KEYPAD_LEN) == 0xFE);
  }

  // --- every keycode encodes to a valid, sum-to-0xFE frame for any hold ---
  const uint8_t keys[] = {KEY_ESC, KEY_PRG, KEY_ALARM, KEY_ENTER, KEY_UP, KEY_DOWN};
  for (uint8_t k : keys) {
    for (int hold = 0; hold <= 0xFF; hold++) {
      uint8_t f[KEYPAD_LEN];
      encode_keypad(k, static_cast<uint8_t>(hold), f);
      assert(keypad_frame_ok(f, KEYPAD_LEN));
      assert(f[3] == k);
    }
  }

  // --- keypad_frame_ok rejects malformed frames ---
  {
    uint8_t f[KEYPAD_LEN];
    encode_keypad(KEY_DOWN, 0x10, f);
    assert(!keypad_frame_ok(f, KEYPAD_LEN - 1));  // wrong length
    uint8_t bad[KEYPAD_LEN];
    for (size_t i = 0; i < KEYPAD_LEN; i++)
      bad[i] = f[i];
    bad[5] ^= 0x01;  // corrupt check byte
    assert(!keypad_frame_ok(bad, KEYPAD_LEN));
    for (size_t i = 0; i < KEYPAD_LEN; i++)
      bad[i] = f[i];
    bad[0] = 0x20;  // wrong source address
    assert(!keypad_frame_ok(bad, KEYPAD_LEN));
  }

  // --- encode_reply9 matches the 9-bit ground-truth capture ---
  // Real pGD Esc tap on the wire: 01' 1E 07 20 01 01 B7  01' 01 20 DD
  {
    const uint8_t expect[REPLY9_LEN] = {0x01, 0x1E, 0x07, 0x20, 0x01, 0x01,
                                        0xB7, 0x01, 0x01, 0x20, 0xDD};
    uint8_t f[REPLY9_LEN];
    assert(encode_reply9(KEY_ESC, 0x01, f) == REPLY9_LEN);
    for (size_t i = 0; i < REPLY9_LEN; i++)
      assert(f[i] == expect[i]);
    // bit9 sits on the two address bytes (0 and 7) and nowhere else
    assert(REPLY9_BIT9_MASK == ((1u << 0) | (1u << 7)));
  }

  // --- poll-token matcher ---
  const uint8_t poll[POLL_LEN] = {0x20, 0x01, 0x01, 0xDD, 0x01, 0x01, 0x20, 0xDD, 0x01};
  assert(is_poll_token(poll, POLL_LEN));
  assert(!is_poll_token(poll, POLL_LEN - 1));  // too short
  uint8_t notpoll[POLL_LEN];
  for (size_t i = 0; i < POLL_LEN; i++)
    notpoll[i] = poll[i];
  notpoll[4] = 0x00;
  assert(!is_poll_token(notpoll, POLL_LEN));

  std::printf("ok\n");
  return 0;
}
