#pragma once

// Hex line formatting for capture logs: upper-case, zero-padded, space
// separated, no trailing space -- the format planscope's offline tools
// (regroup/analyze/correlate) parse. Pure and host-testable
// (test/test_hex_format.cpp); heap use is one std::string per line.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace plan {

inline std::string format_hex_line(const uint8_t *d, size_t n) {
  std::string out;
  out.reserve(n * 3);
  char b[4];
  for (size_t i = 0; i < n; i++) {
    snprintf(b, sizeof(b), "%02X ", d[i]);
    out += b;
  }
  if (!out.empty())
    out.pop_back();  // drop the trailing space
  return out;
}

}  // namespace plan
