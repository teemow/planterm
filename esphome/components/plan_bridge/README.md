# `plan_bridge` — ESP32 pLAN bridge (ESPHome component)

ESPHome **external component** that puts an ESP32 on the CAREL pLAN bus as
a terminal: it captures every byte (with its 9th bit) losslessly, joins the
bus as an additional pGD terminal via roll-call enrollment, and injects key
presses — the reference firmware for the
[planterm](https://github.com/teemow/planterm) library and the device end
of the [PLANCAP capture protocol](../../../docs/capture-protocol.md) that
host tools such as [planscope](https://github.com/teemow/planscope) speak.

Two channels, two audiences:

- **ESPHome native API** (Home Assistant): user-defined services
  (`set_armed`, `set_enroll`, `inject_key`, …) and whatever template
  switches/buttons the device YAML declares — see the
  [example YAML](../../plan-bridge-example.yaml).
- **PLANCAP socket** (host tools): TCP port 6054 serves the raw bus
  stream, typed device events, diagnostics and the command channel
  (arm/enroll/inject with acks) — authenticated and encrypted with the
  device's API key. This is the machine interface host tools consume; a
  tool needs no other channel for its live features.

## Hardware

The [reference rig](../../../README.md#reference-hardware): any ESP32 with a
MAX3485 RS-485 transceiver. Three GPIOs:

| GPIO | MAX3485 | role |
|---|---|---|
| `rx_pin` | RO | receive; poll detection + capture |
| `tx_pin` | DI | transmit (keypad reports) |
| `de_pin` | DE+RE tied (EN) | transmit enable, driven by the RX ISR around each 9-bit transmit; RE low keeps the receiver live while idle |

**esp-idf framework required** (the component owns the UART interrupt;
arduino won't link). The device YAML must also keep FreeRTOS in IRAM:

```yaml
esp32:
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH: "n"
```

The RX ISR is IRAM-resident so it keeps answering polls while the flash
cache is disabled (WiFi/NVS/OTA writes); every function it calls must be
IRAM too, or the first flash write panics the device.

## The 9-bit problem

pLAN is a **9-bit multidrop protocol** ([protocol.md §1](../../../docs/protocol.md)):
each frame's leading address byte carries a 9th "address" bit, and the
controller's UART filters on it — byte-identical 8N2 transmissions are
silently ignored. The component receives as 8E1 (bit9 = even-parity(data)
XOR parity-error flag) and transmits bit9 by selecting EVEN/ODD parity per
byte.

## Why an ISR owns the UART

The controller polls its terminal and the terminal must start its reply
within ~1 byte-time (~176 µs at 62500 baud) or the slot is gone. The
buffered ESP-IDF UART driver (driver ISR → ring buffer → task →
`uart_write_bytes`) was proven on hardware to never make that window. So
the component bypasses the driver: a custom RX interrupt (RX-FIFO
threshold 1) feeds each byte to planterm's `PlanTerminal` state machine,
and when it returns a transmit action the ISR itself raises DE, shifts the
reply out with the per-byte parity trick, and drops DE —
single-digit-microsecond reaction.

**This code is frozen in its proven shape.** The ISR structure is
live-tested to be shape-sensitive in ways that defy local reasoning (see
the inline comments in `plan_bridge.cpp` and planterm's `plan_terminal.h`
before restructuring anything). New behavior belongs host-side, or in the
task-side slow path — never in the ISR.

## Safety model

Transmit is gated behind an **`armed`** flag, default **false**, so a
reboot never comes up able to move the machine. Arm it at runtime (a
template switch in the YAML, the `set_armed` API service, or PLANCAP's
arm command). A key press requested while disarmed is dropped, logged
and — over PLANCAP — rejected in the ack. Enrollment (`set_enroll`) is
passive with respect to the controlled machine — key injection stays
gated behind `armed` even while enrolled.

Injection self-limits: a press answers at most `repeat` polls, spaced by
`repeat_interval_ms`, and aborts on a deadline if polls stop — it can
never hold the bus.

## Configuration

```yaml
external_components:
  - source: github://teemow/planterm@main
    components: [plan_bridge]

plan_bridge:
  id: bridge
  rx_pin: GPIO20   # MAX3485 RO
  tx_pin: GPIO21   # MAX3485 DI
  de_pin: GPIO7    # MAX3485 DE+RE tied (EN)
```

| Option | Default | Notes |
|---|---|---|
| `rx_pin` | — (required) | MAX3485 `RO`; poll detection + capture |
| `tx_pin` | — (required) | MAX3485 `DI` |
| `de_pin` | — (required) | DE+RE tied; plain GPIO, driven by the ISR around TX |
| `baud_rate` | `62500` | pLAN bus speed |
| `uart_num` | `1` | ESP32 UART peripheral (avoid 0 = logger) |
| `stop_bits` | `2` | ignored: the wire format is 8+bit9, handled internally |
| `gap_ms` | `3` | stream-drain tick: capture-record boundary |
| `repeat` | `1` | poll slots answered per single press (1 = a tap) |
| `repeat_interval_ms` | `60` | min spacing between poll-slot responses |
| `armed` | `false` | compile-time default of the write-enable gate |
| `enroll` | `false` | join the pLAN as a second terminal at boot (address 31 must be in the controller's terminal list) |
| `capture_key` | `api.encryption.key` | the PLANCAP PSK (base64, 32 bytes). Defaults to the device's API key so one key secures both channels; set it explicitly only on devices without an encrypted API. **No key at all = unauthenticated plaintext mode: trusted networks only.** |

The component pulls the planterm library from its own checkout — pinning
the `external_components` ref pins library and component together. Do not
add a second planterm `lib_deps` entry.

Public methods for YAML lambdas: `press_key(uint8_t keycode)`,
`set_armed(bool)`, `set_enroll(bool)`, `set_tx_mode(int)`,
`set_turnaround_us(uint32_t)`. Keycodes are in
[protocol.md §6](../../../docs/protocol.md).

### Hooks for components building on top

A higher-level component (a menu scraper / field extractor) can ride the
bridge without touching the ISR:

- `set_stream_listener(fn)` — a second consumer of the drained
  `(byte, bit9)` stream, called from the bus task (never the ISR).
- `press_key_internal(keycode)` — same queue as `press_key()` but bypasses
  the `armed` gate, for autonomous navigation that the write-enable switch
  should not gate.
- `enrolled()` / `joined()` / `link_reset()` — link state; `joined()` is
  the *actual* join (first poll answered), not the enroll wish flag.
- `post_hold_event(bool)` — surface a walk-yield handshake to host tools
  as an `EV_HOLD` event on the capture stream.

## PLANCAP (TCP 6054) — capture, events, diagnostics, commands

The device end of **[PLANCAP](../../../docs/capture-protocol.md)** (the
normative spec), served on TCP port 6054 (= ESPHome API port + 1); the
ESPHome logger carries human prose only. Log lines proved structurally
lossy as a data transport (dropped whole lines under burst load,
truncated long ones), so nothing machine-parses log prose.

Single client (newest connection wins), 7-byte banner `PLANCAP`, then a
Noise `NNpsk0_25519_ChaChaPoly_SHA256` handshake (prologue `PLANCAP`,
PSK = the `capture_key`, i.e. by default the device's
`api.encryption.key`) — or, on a keyless device, the plaintext hello.
Every record travels in one `0x01`/`0x00` + `u16 BE len` frame. Record
types (all integers little-endian; exact layouts in the spec):

| Type | Direction | Content |
|---|---|---|
| `0x00` bus bytes | → client | `seq`, `ts_ms`, in-band drop count, `n` (byte, bit9) pairs |
| `0x01` device event | → client | kind + 2 payload bytes, see below |
| `0x02` diagnostic | → client | severity + one line of prose, ordered with the bus bytes it refers to |
| `0x03` command | client → | arm / enroll / inject_key, dispatching into the same internals as the API services |
| `0x04` ack | → client | echoes the command id; accepted / rejected / unknown op |

| Event | Payload | Meaning |
|---|---|---|
| `EV_STATE` (1) | armed, enroll (no/yes/drain), tx_mode | device truth, on change + every 10 s (the in-band keepalive) + once on session establish |
| `EV_JOIN` (2) | — | the actual link join: first poll to our address answered since `set_enroll(true)` |
| `EV_TX_FIRED` (3) | keycode, attempt | the keypad report went out |
| `EV_KEY_ACCEPTED` (4) | keycode, attempt | verdict: no rejection signature followed |
| `EV_HOLD` (5) | paused flag | a rider component's menu walk yielded the session |

**Screen-snapshot replay:** the controller repaints only *changed* rows,
so a client connecting mid-session would start from a blank
reconstruction. The component caches the latest checksum-valid display
frame per (terminal, row) (planterm's `plan_snapshot.h`) and replays the
cache as soon as the session is established, as **seq-0 records**; live
records start at seq 1, so gap accounting is untouched. One `EV_STATE`
event follows the replay, so a fresh client holds the device truth
immediately.

The transport session (handshake, framing, encryption) is planterm's
`plan_cap.h`, host-tested against a real Noise initiator in
`test/test_plan_cap.cpp` — the firmware and the Go clients implement the
same spec, not each other's quirks.

## Injection verdict

After each transmitted report the task watches for the two rejection
signatures — the controller's immediate re-poll and the link-reset FF-walk
— and retries on failure. In **tx_mode 2** (enrolled slot, the default
worth using) silent discards were never observed and the accepted verdict
arrives ~40 ms after the TX; modes 0 (slot-racing the pGD) and 1 are
legacy experiments and unreliable.

Disenroll (`set_enroll(false)`) is a graceful drain that ends at a fixed
~15 s deadline: the controller never roll-calls an established terminal,
so the renounce slot never comes. This is measured, known, and deliberately
left as is (an immediate leave broke re-enrollment on hardware — the
frozen-shape rule again).

## Tests

The protocol state machine the ISR runs is planterm's `PlanTerminal`,
integration-tested in this repo's host suite (`test/test_integration.cpp`)
against a mock controller whose every frame cites live-bus behavior. The
PLANCAP transport session is host-tested against a real noise-c initiator
(`test/test_plan_cap.cpp`): handshake, encrypted records both ways,
command dispatch, the wrong-key reject, plaintext mode, framing errors.
The component itself is the hardware glue (FIFO checks, turnaround delay,
DE GPIO, 9-bit transmit) plus the record content and backlog policy; CI
compiles it standalone against real ESPHome + esp-idf with the
[example YAML](../../plan-bridge-example.yaml).
