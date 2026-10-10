# can-j1939

SAE J1939-21 multi-packet transport for microcontrollers. **No dynamic
allocation, no exceptions, no RTTI**, and the protocol logic is fully testable
on a host with no CAN hardware attached.

Header-only C++17. The whole library is two headers and compiles to about
**4.4 KB of code**.

## Why this exists

Classic CAN carries 8 bytes per frame. J1939 parameter groups run up to 1785
bytes, so anything longer is fragmented into 7-byte chunks and reassembled. The
fragmentation is the easy part. What makes a J1939 stack hard is the session
state: two different transport protocols, five timers, flow control, abort
handling, and the fact that a vehicle bus will hand you malformed and
out-of-order frames at three in the morning on a truck in Poland.

Most implementations either pull in the heap, or bury the protocol logic inside
a driver so it can only be tested on target with a scope attached. This does
neither.

## What it does

| | |
|---|---|
| **29-bit identifier codec** | Priority, EDP, DP, PF, PS, SA. Fully `constexpr`, so PGN tables live in flash. |
| **PDU1 / PDU2 handling** | PS is a destination address below PF 0xF0 and part of the PGN at or above it. Getting this wrong is the classic J1939 bug; see below. |
| **BAM** | Broadcast Announce Message. TP.CM control 32, then unacknowledged data frames with a configurable inter-frame gap. |
| **CMDT** | Connection Mode Data Transfer. RTS → CTS → data → EndOfMsgAck, with receiver-driven flow control and hold handling. |
| **Abort** | Sent and handled, with the J1939-21 reason codes (busy, resources freed, timeout, unexpected data packet). |
| **Timers** | Tr, Th, T1, T2, T3, T4 from J1939-21 table 10, injected rather than read from a global clock. |

## The PDU1 / PDU2 trap

The same byte position means two different things depending on the PDU format,
and treating it uniformly produces a stack that looks fine on the bench and
misroutes broadcast messages in the field.

```cpp
// PGN 61444, Electronic Engine Controller 1. PF = 0xF0, so PDU2.
const CanId a = decode(0x0CF00400);
a.pgn();          // 61444 - the 0x04 is a group extension, part of the PGN
a.destination();  // 0xFF  - broadcast. NOT node 0x04.

// TP.CM from 0x21 to 0x17. PF = 0xEC, so PDU1.
const CanId b = decode(0x1CEC1721);
b.pgn();          // 0x00EC00 - low byte zeroed, it is not part of the PGN
b.destination();  // 0x17    - here the byte really is an address
```

`make_id()` enforces this in the other direction: pass a destination for a
PDU2 PGN and it is discarded rather than written over the group extension,
because silently changing the PGN is worse than ignoring an argument.

## No allocation, enforced by the build

The claim is checked, not asserted in prose. `tools/freestanding_main.cpp`
exercises every public entry point and is compiled with
`-fno-exceptions -fno-rtti -Werror -Os`. Then:

```sh
$ ./tools/check_no_heap.sh build/j1939_freestanding
PASS: no allocator or unwinder symbols in build/j1939_freestanding
```

The script fails the build if `operator new`, `malloc`, `_Unwind_` or
`__cxa_throw` appear in the symbol table. Measured footprint of that binary,
configured with two sessions:

```
   text    data     bss     dec
   4366     648    3968    8982
```

Nearly all of the bss is reassembly buffers: each session owns a 1785-byte
buffer, which is the price of J1939's maximum message size. `Transport<N>` is
the only sizing knob, so a Cortex-M0 picks 2 and a gateway picks 16.

Everything else is deliberate too: no recursion, bounded loops only, errors as
return values, and `MessageHandler` is a function pointer plus a context
pointer rather than `std::function`, which would allocate.

## Testing on a host, not a bench

`CanBus` is a two-method interface. On target it wraps the CAN peripheral; in
tests `FakeBus` records frames in a vector. Time is a `std::uint32_t`
millisecond argument rather than a call into a clock, so a 1250 ms timeout test
runs instantly and deterministically instead of sleeping and hoping.

That combination is what makes the 31 tests possible:

```sh
$ ./build/j1939_tests
[==========] 31 tests from 2 test suites ran. (0 ms total)
[  PASSED  ] 31 tests.
```

They cover the protocol, not the happy path:

- PDU1 and PDU2 decoding, round-trip across every encodable field, 29-bit
  bounds, data-page bits surviving the trip, and the codec working in a
  `static_assert`
- BAM and CMDT round trips, including the **1785-byte maximum** and the full
  255-packet case
- the sender honouring the inter-frame gap, and padding the final frame with
  `0xFF` rather than `0x00`, so a receiver that ignores the length field reads
  "not available" instead of a real zero
- CTS with zero packets treated as a hold, not an error
- out-of-order data frames aborting **and telling the peer why**
- an RTS whose byte count and packet count disagree being rejected rather than
  trusted
- session-pool exhaustion returning a status and sending a busy abort
- receiver timeout after T2, with no partial message delivered
- a jammed mailbox mid-transfer releasing the session instead of leaking a slot
- two peers reassembled independently with their frames interleaved
- frames addressed to other nodes and non-transport frames not being claimed

ASan and UBSan builds pass clean.

## A bug these tests actually caught

`BamHandlesTheMaximumMessageSize` failed on the first run. The sequence counter
was a `std::uint8_t`, and at the 1785-byte maximum there are exactly 255
packets, so after sending packet 255 the counter wrapped to 0 and the
completion test `next_seq > total_packets` became `0 > 255`, which is false
forever. The message hung until the session timed out.

It only appears at the exact protocol maximum, which is why the test uses that
size rather than a round number. The counter is now 16-bit internally while the
wire field stays 8-bit, with a comment at the declaration explaining why.

## Using it

```cpp
#include "j1939/transport.hpp"

class StmCanBus final : public j1939::CanBus {
public:
  bool send(const j1939::CanFrame& f) override {
    return HAL_CAN_AddTxMessage(&hcan_, &header_from(f), f.data, &mailbox_) == HAL_OK;
  }
};

StmCanBus bus;
j1939::Transport<2> transport(bus, /*own_address=*/0x21);

void on_message(void* ctx, j1939::pgn_t pgn, j1939::addr_t src,
                const std::uint8_t* data, std::size_t len) {
  handle_parameter_group(pgn, src, data, len);
}

void setup() { transport.set_handler(&on_message, nullptr); }

// From the CAN receive interrupt or a driver task:
void on_can_rx(const j1939::CanFrame& f) { transport.on_frame(f, millis()); }

// From the main loop, at whatever rate you already run it:
void loop() {
  transport.tick(millis());

  static const std::uint8_t dm1[42] = { /* ... */ };
  transport.send(0x00FECA, j1939::kBroadcastAddress, dm1, sizeof(dm1), millis());
}
```

`send()` picks the protocol from the destination: `kBroadcastAddress` selects
BAM, anything else selects CMDT. Payloads of 8 bytes or fewer go out as a
single frame and never touch a session.

## What this does not do

- **No address claiming.** J1939-81 network management, the Name/address
  negotiation that decides who owns address 0x21, is a separate concern and
  not implemented.
- **No PGN database.** Decoding engine speed out of the reassembled bytes is
  the application's job. There is no DBC parser and no SPN scaling.
- **No CAN FD.** Transport exists precisely because classic CAN is 8 bytes; on
  CAN FD most of this is unnecessary.
- **No retransmission of individual packets.** An out-of-order data frame
  aborts the session rather than attempting mid-stream recovery. Recovery costs
  state and code space for a case a sender handles better by starting over.
- **The CMDT receiver accepts the whole message in one CTS burst.** A receiver
  with less RAM than the message would issue several smaller CTS frames
  instead; the hooks are there but that policy is not implemented.
- **Single-threaded.** No internal locking. Call `on_frame` and `tick` from the
  same context, or serialise them yourself.
- **Not certified for anything.** Tested against the specification as written,
  not against a conformance suite or real ECUs.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/j1939_tests
./tools/check_no_heap.sh build/j1939_freestanding
```

CI runs gcc and clang, the ASan/UBSan build, and the no-heap check on every
push.

## Layout

```
include/j1939/can_id.hpp      29-bit identifier codec, constexpr, ~120 lines
include/j1939/transport.hpp   BAM and CMDT sessions, ~560 lines
tests/test_can_id.cpp         codec tests, including PDU1/PDU2
tests/test_transport.cpp      protocol tests against a fake bus
tests/fake_bus.hpp            recording bus and message sink
tools/freestanding_main.cpp   compiled -fno-exceptions -fno-rtti
tools/check_no_heap.sh        symbol-table assertion
```
