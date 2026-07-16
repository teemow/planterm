# planterm

CAREL pLAN terminal protocol library — act as a pGD terminal: decode, enroll,
navigate, edit.

CAREL's proprietary **pLAN** protocol runs many HVAC controllers (pCO/µPC
families) and their pGD display terminals over RS-485: 62500 baud, 9-bit
multidrop framing where address bytes carry a 9th bit. There is no public
documentation and, to our knowledge, no other open-source implementation.
planterm implements the *terminal* role: it lets your microcontroller behave
like a pGD — read what the controller displays, join the bus as an additional
terminal, press keys, and edit settings.

## Honest scope statement

This protocol was reverse-engineered from scratch against **one** unit: a µPC
controller running a heat-pump application, with a pGD terminal on the bus.
Everything in this library traces to captures from that bus. It is *expected*
— but **unverified** — to generalize across the pGD line and other pLAN
devices. If you run it against different hardware, captures and reports are
very welcome.

## Design

Header-only, platform-agnostic C++17. Namespace `plan::`. The boundary is
strict: bytes (plus the 9th bit) in, TX bytes/actions out, time injected via
parameters. No UART code, no interrupts, no board support — you supply the
transport.

- Passive capture/decode works with an ordinary 8N2 UART (the 9th bit shows up
  as a parity/framing artifact you can ignore).
- Full terminal emulation (enrollment, key injection) needs a 9-bit-capable
  transport: you must *read* which bytes carried the 9th bit and *set* it on
  designated TX bytes. How you achieve that (parity tricks, bit-banging) is
  platform-specific and out of scope for the core.

## Reference hardware

The library is transport-agnostic, but everything in it was developed and
proven on one concrete rig: an **ESP32** with a **MAX3485** RS-485
transceiver (the 3.3 V sibling of the MAX485 — ESP32 GPIOs are not
5 V-tolerant). The reference board is a lolin_c3_mini (ESP32-C3), but any
ESP32 works: the GPIO matrix routes UARTs to arbitrary pins, and the ESP32
generates the 62500 baud rate exactly (80 MHz / 1280).

The pin labels below are those of the common MAX3485 breakout module (bare
chip names in parentheses):

```
pLAN bus            MAX3485 module     ESP32
------------------  -----------------  ------------------------------
RX+/TX+             A                  --
RX-/TX-             B                  --
GND                 GND                GND  (common ground, required)
                    VCC                3V3
                    RXD (RO)           UART RX  (reference: GPIO20)
                    TXD (DI)           UART TX  (reference: GPIO21)
                    EN  (DE+RE tied)   any GPIO (reference: GPIO7)
```

- 120 Ω termination only if the ESP32 sits at a physical end of the bus.
- Galvanic isolation (an isolated RS-485 module) is recommended when tapping
  mains-powered HVAC equipment.
- For passive capture, drive EN low permanently — the rig then cannot
  disturb the bus. For terminal emulation the EN pin is raised around each
  transmit; the 9-bit requirement is met with the UART parity trick described
  in the transport notes of [docs/protocol.md](docs/protocol.md#1-physical-layer).

## Protocol documentation

[docs/protocol.md](docs/protocol.md) is a standalone reference for the pLAN
protocol as reverse-engineered here: physical layer, addressing, both
checksum grammars, the poll/reply heartbeat, roll-call enrollment, keypad
reports, the display session, the captured attach sequence, multi-terminal
behavior, and measured timings. For an otherwise undocumented protocol, the
doc is as much the product as the code.

[docs/capture-protocol.md](docs/capture-protocol.md) specifies **PLANCAP**,
the authenticated capture-and-control protocol between a pLAN bridge device
(a planterm-based firmware on the bus) and host tools such as
[planscope](https://github.com/teemow/planscope): Noise-encrypted transport,
lossless bus capture with 9th-bit fidelity, typed device events and
diagnostics, and the command channel (arm, enroll, key injection).

## Contents

- `src/plan_frame.h` — the frame grammar: byte-sum boundary oracle
  (`frame_len_at`), keypad-report encode/validate, poll-token matcher,
  response-slot reply with 9th-bit mask, display/measurement record decode.
- `src/plan_terminal.h` — `PlanTerminal`, the terminal-side protocol state
  machine: byte + 9th bit in, `TxAction` out. Poll matching, roll-call
  enrollment (claim and graceful renounce), session-frame acking with both
  checksum grammars, ident reply, key injection into the response slot,
  link-reset and rejection detection, bus telemetry. Time is injected via
  parameters; the caller owns the transport and reports back via
  `tx_sent()`/`tx_not_sent()`. This code has run an ISR on a live bus for
  weeks — it is deliberately frozen in its proven shape (see the inline
  comments before restructuring anything).
- `src/plan_screen.h` — `PlanScreen`, the pGD screen reconstructor:
  accumulates display frames (`0x0B` text rows, `0x0C` single cells, `0x64`
  graphics bands, `0x65` page sync) into 2 terminals × 8 rows × 22 chars,
  with inverse-video band tracking (the menu cursor) and settle timestamps.
- `src/plan_snapshot.h` — `PlanSnapshot`, the screen-snapshot cache: keeps
  the latest checksum-valid display frame per (terminal, row) as raw wire
  bytes, so a capture client connecting mid-session can be replayed a full
  screen instead of waiting hours for a static page to repaint.
- `src/plan_fields.h` — the field-extraction engine: matcher kinds for every
  observed row shape (anchored label + number, tokens, scheduler slots, ...),
  page identity, and the content-driven Input/Output scan. The spec *table* —
  which page paints what where — is device application data and lives with
  the consumer; this header ships the matchers and the runner.
- `src/plan_nav.h` — the navigation engine: verified primitives (settle-then-
  verify step, Esc-to-anchor, band-verified menu select, selection-band
  seek) shared by both machines below, plus `PlanNav`, a scheduled scrape
  runner that executes a consumer-supplied `ScrapeStep[]` route — every step
  screen-verified, each visited page emitted for extraction, exponential
  backoff on failure. Routes, menu tables, and walk budgets are device
  application data.
- `src/plan_edit.h` — `PlanEdit`, the transactional edit engine (navigate
  with whole-route retry, focus-hop, one press per read-back, commit +
  verify, Esc abort on any divergence, digit-by-digit PIN gate entry),
  multi-op page visits (`set_ops`) and non-committing selector sweeps
  (`read_sweep`), plus `EditArbiter`: one nav owner at a time on the single
  enrolled session, writes jumping ahead of queued reads. The macro
  registry (`MacroDef` tables) is device application data.
- `src/hex_format.h` — hex line formatting for capture logs, in the format
  planscope's offline tools parse.
- `esphome/components/plan_bridge/` — the **ESP32 bridge firmware**: an
  ESPHome external component that puts the library on a live bus — 9-bit
  UART RX interrupt with in-ISR response-slot transmit, the capture-stream
  server host tools attach to, and Home-Assistant-facing services.
  **Strictly optional**: the library itself has zero ESPHome dependency, the
  PlatformIO registry package excludes this directory entirely, and Arduino
  builds never compile it — it is used only when you consume the repo as an
  ESPHome external component. See its
  [README](esphome/components/plan_bridge/README.md); consume it with

```yaml
external_components:
  - source: github://teemow/planterm@main
    components: [plan_bridge]
```

## Install

- **Arduino IDE**: Library Manager → search "planterm".
- **PlatformIO**: add `lib_deps = teemow/planterm` to `platformio.ini`.

Or vendor it directly — it is header-only; `#include <planterm.h>` pulls in
everything, or include the individual headers you need.

## Getting started: capture the bus

[examples/BusCapture](examples/BusCapture/BusCapture.ino) is the place to
start: a passive, transmit-never sketch that taps the RS-485 bus on the
[reference hardware](#reference-hardware) above, splits the byte stream into
pLAN frames with the boundary oracle and prints them as hex — the "is my tap
point right?" tool. The sketch header documents the wiring and the
8N2-receive caveat (passive capture needs no 9-bit handling; full terminal
emulation does — see the transport notes in
[docs/protocol.md](docs/protocol.md#1-physical-layer)).

## Tests

Every header is host-testable with a plain C++17 compiler, one standalone
binary per test:

```sh
c++ -std=c++17 test/test_plan_frame.cpp    -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_plan_decode.cpp   -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_integration.cpp   -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_plan_screen.cpp   -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_plan_snapshot.cpp -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_hex_format.cpp    -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_plan_fields.cpp   -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_plan_nav.cpp      -o /tmp/t && /tmp/t  # prints "ok"
c++ -std=c++17 test/test_plan_edit.cpp     -o /tmp/t && /tmp/t  # prints "ok"
```

`test_integration.cpp` drives `PlanTerminal` end to end against a mock µPC
controller (`test/mock_controller.h`) whose every emitted frame and
validation rule cites a behavior captured off a live bus: enrollment at
address 31, 100 uncontested poll cycles, key injection in the enrolled
slot, session acks under both checksum grammars, the ident reply, the
re-poll rejection signal, FF-walk link-reset detection, and the graceful
disenroll drain.

CI runs all of them on every push and pull request, and additionally builds
the BusCapture example for an ESP32-C3 board with PlatformIO.

`test_plan_screen.cpp` replays an archived menu-walk capture (parsed by the
shared `test/walk_replay.h` harness) through `PlanScreen` and asserts the
reconstruction invariants ported from planscope's `replay_test.go`: at most
one selection band below the title bar on every settled page, and the row
content of documented pages matching their reference dumps
(digit-normalized — live values drift). The reference capture is not part
of this repo; without one the test skips cleanly (still exits 0, as in CI).
To run it against a capture, tee one with `planscope observe --raw live.log`
(or `planscope log`) and pass it as the first argument — add `--live` to
relax the coverage assertions to what a scrape-route capture visits.

## Companion tooling

[planscope](https://github.com/teemow/planscope) is the interactive pLAN
debugger and capture workbench this library was developed with.

## License

MIT — see [LICENSE](LICENSE).
