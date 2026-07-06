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

## Contents

- `src/plan_frame.h` — the frame grammar: byte-sum boundary oracle
  (`frame_len_at`), keypad-report encode/validate, poll-token matcher,
  response-slot reply with 9th-bit mask, display/measurement record decode.

More of the terminal stack (screen reconstruction, enrollment state machine,
navigation/edit engines) is being extracted here from the firmware it was
developed in; see the commit history.

## Tests

Every header is host-testable with a plain C++17 compiler, one standalone
binary per test:

```sh
c++ -std=c++17 test/test_plan_frame.cpp -o /tmp/t && /tmp/t   # prints "ok"
c++ -std=c++17 test/test_plan_decode.cpp -o /tmp/t && /tmp/t  # prints "ok"
```

CI runs all of them on every push and pull request.

## Companion tooling

[planscope](https://github.com/teemow/planscope) is the interactive pLAN
debugger and capture workbench this library was developed with.

## License

MIT — see [LICENSE](LICENSE).
