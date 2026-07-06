// Host-compilable self-check for the capture-log hex formatting. Run with:
//   c++ -std=c++17 test/test_hex_format.cpp -o /tmp/t && /tmp/t

#include "../src/hex_format.h"
#include <cassert>
#include <cstdio>
#include <vector>

static std::string fmt(std::vector<uint8_t> v) { return plan::format_hex_line(v.data(), v.size()); }

int main() {
  assert(fmt({}) == "");
  assert(fmt({0x00}) == "00");
  assert(fmt({0x06}) == "06");
  // No trailing space, upper-case, zero-padded, in order.
  assert(fmt({0x02, 0x20, 0xFF, 0x03}) == "02 20 FF 03");

  std::printf("ok\n");
  return 0;
}
