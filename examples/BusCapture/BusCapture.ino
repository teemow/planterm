// BusCapture -- minimal passive pLAN bus tap.
//
// The newcomer's "is my tap point right?" tool: listens on the RS-485 bus,
// splits the byte stream into atomic pLAN frames with the library's
// byte-sum boundary oracle (plan::frame_len_at) and prints each frame as a
// hex line. It never transmits -- the transceiver is pinned to receive mode,
// so tapping a live bus carries no risk of contention.
//
// Wiring (MAX3485 module -> ESP32; bare chip pin names in parentheses.
// GPIOs below are a lolin_c3_mini, any ESP32 works since the GPIO matrix
// routes UART1 to arbitrary pins):
//
//   pLAN bus            MAX3485 module     ESP32
//   ------------------  -----------------  -----------------------------
//   RX+/TX+             A                  --
//   RX-/TX-             B                  --
//   GND                 GND                GND   (common ground, required)
//                       VCC                3V3
//                       RXD (RO)           RX_PIN (GPIO20)
//                       TXD (DI)           TX_PIN (GPIO21, unused here)
//                       EN  (DE+RE tied)   DE_PIN (GPIO7, driven low = receive)
//
//   - 120 ohm termination only if the ESP is a physical bus end.
//   - Galvanic isolation (an isolated RS-485 module) is recommended.
//   - Leave the real pGD terminal connected as your live reference display.
//
// The 8N2-receive caveat: pLAN is 9-bit multidrop framing -- every frame's
// first byte carries a 9th "address" bit that a plain 8N2 UART cannot see.
// Passive capture works anyway: the 9th bit only surfaces as occasional
// parity/framing noise, and frames are still split reliably by the byte-sum
// oracle. Full terminal emulation (enrollment, key injection) needs a
// 9-bit-capable transport that reads and sets bit9 per byte -- see the
// "Physical layer" transport notes in docs/protocol.md.
//
// If frames stream in (and change when you press keys on the real pGD), the
// tap point is right. Analyze a saved capture offline with planscope
// (https://github.com/teemow/planscope): regroup / analyze / correlate.

#include <hex_format.h>
#include <plan_frame.h>

#include <cstring>

static const int RX_PIN = 20;
static const int TX_PIN = 21;  // unused while capturing, but the UART wants a pin
static const int DE_PIN = 7;
static const uint32_t PLAN_BAUD = 62500;
// Idle gap that flushes a garbled tail. One 9-bit character is ~176 us at
// 62500 baud, so 3 ms of silence (~17 byte-times) safely marks end-of-burst.
static const uint32_t GAP_MS = 3;
// The controller's 9-byte poll token is ~93% of idle traffic; hide it by
// default so real frames stand out. Set true to see the raw heartbeat.
static const bool LOG_POLL_TOKENS = false;

static uint8_t buf[256];
static size_t len = 0;
static uint32_t last_byte_ms = 0;

static void emit(const char *tag, const uint8_t *d, size_t n) {
  Serial.printf("%s[%3u] %s\n", tag, static_cast<unsigned>(n), plan::format_hex_line(d, n).c_str());
}

static void consume(size_t n) {
  std::memmove(buf, buf + n, len - n);
  len -= n;
}

void setup() {
  Serial.begin(115200);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);  // receiver enabled, driver disabled: passive
  Serial1.begin(PLAN_BAUD, SERIAL_8N2, RX_PIN, TX_PIN);
  Serial.println("plan bus capture: listening...");
}

void loop() {
  while (Serial1.available() && len < sizeof(buf)) {
    buf[len++] = static_cast<uint8_t>(Serial1.read());
    last_byte_ms = millis();
  }

  // Split complete frames off the front of the buffer.
  for (;;) {
    size_t flen = plan::frame_len_at(buf, len);
    if (flen > 0) {
      if (LOG_POLL_TOKENS || !plan::is_poll_token(buf, flen))
        emit("", buf, flen);
      consume(flen);
      continue;
    }
    // 12 bytes is the longest atomic frame: once that many are waiting and
    // the head still doesn't checksum out, it can never start a frame.
    if (len >= 12) {
      emit("junk ", buf, 1);
      consume(1);
      continue;
    }
    break;
  }

  // Bus went idle with a partial tail left over: flush it so nothing hides.
  if (len > 0 && millis() - last_byte_ms >= GAP_MS) {
    emit("tail ", buf, len);
    len = 0;
  }
}
