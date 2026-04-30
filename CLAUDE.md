# Modbus Client (Plain C++ + boost.asio) — Project Context

This file is the source of truth for design intent and rules in this codebase.
Claude Code reads this on every session start.

---

## Project Goal

Extend an **existing C++ Modbus client** that:

- Implements the **Modbus protocol from scratch** in plain C++ (no Modbus
  library — no libmodbus, no QModbus, no third-party Modbus stack).
- Uses **`boost.asio` for I/O** — currently TCP only; RS-485 serial is
  planned but not yet implemented.

The goal is to robustly support a mixed fleet of protection relays (SEL, GE,
and possibly ABB / Siemens later) by making **targeted, minimal, well-tested
changes** that move the codebase toward the target architecture below.

We are NOT rewriting the client. We are NOT adding new libraries beyond
the ones already approved (see below).

**Verified hardware (as of 2026-04-29):** SEL-735 firmware R206-V1 (build
date 2021-12-14), serial 3220400087, Form 5 wiring, reachable at
`192.168.0.2:502`. **Full FC test suite passes 10/10** — all 7
SEL-735-supported function codes (01, 02, 03, 04, 05, 06, 10) verified
end-to-end:

- FC 03: FID string, serial, meter form, comm counters — all decoded.
- FC 04: returns identical FID as FC 03 (confirms FC 03 ≡ FC 04 on SEL).
- FC 02: 6 digital input contacts read.
- FC 01: 23 coil bits (outputs + RB1..RB16) read.
- FC 05: RB01 toggle ON→OFF round-trip with read-back verification.
- FC 06: Reset Communication Counters executed (counters cleared post-write).
- FC 10: Reset Max/Min + Reset Peak Demand executed in one multi-register
  write.

This confirms TCP, MBAP, big-endian byte order, address mapping, transaction
ID handling, and standard Modbus exception detection all work end-to-end.

---

## Dependency Policy

This project deliberately keeps dependencies minimal.

### ✅ Allowed

- **C++ standard library** — `<vector>`, `<string>`, `<cstdint>`, `<chrono>`,
  `<thread>`, `<mutex>`, `<fstream>`, `<optional>`, `<variant>`, etc.
- **`boost.asio`** — for all I/O work:
  - `boost::asio::serial_port` for RS-485 serial.
  - `boost::asio::ip::tcp::socket` for Modbus TCP.
  - `boost::asio::steady_timer` for timeouts and inter-frame silent intervals.
  - `boost::asio::io_context` for the async event loop.
- **Other Boost headers already used in the project** (check existing
  includes — e.g., `boost.system` comes with asio). Match what's already
  there; don't pull in new Boost libraries without asking.

### ❌ Not allowed without explicit approval

- libmodbus, QModbus, or any other Modbus library.
- nlohmann/json, yaml-cpp, RapidJSON, or any other config-parsing library.
- spdlog, glog, or any other logging library.
- gtest, Catch2, doctest, or any test framework (unless project already uses one).
- Qt, Poco, fmt, or any other framework.

If a task seems to need an unapproved library, **ask first** — most things
(CRC-16, simple JSON/INI parsing, basic logging, hand-rolled test harness)
fit in <100 lines of plain C++ when needed.

---

## Working Mode (read this first, every session)

- **Modify, don't rewrite.** Default to the smallest change that solves the
  problem. Refactors are allowed only when explicitly requested or when
  impossible without one.
- **Show diffs, not full file rewrites**, unless the file is genuinely new.
- **Don't touch unrelated code.** Even if it looks suboptimal. Leave it alone
  and flag it in the response if needed.
- **Match the existing code style** — naming, indentation, header conventions,
  error-handling pattern. Read 2–3 nearby files before writing new code.
- **Preserve the existing public API** of the Modbus client unless the user
  explicitly asks for a breaking change.
- **No new dependencies** without explicit approval (see policy above).

---

## Stack

- **Language:** C++17 (`set(CMAKE_CXX_STANDARD 17)` in `CMakeLists.txt:4`).
- **Modbus library:** **NONE — implemented from scratch in plain C++.**
- **I/O library:** `boost.asio` (header-only via `find_package(Boost 1.70)`).
  `boost::asio::ip::tcp::socket` is implemented; `boost::asio::serial_port`
  is **not yet written** (RS-485 is future work — see Current Implementation
  Status below).
- **Build system:** **CMake 3.16+** (`cmake_minimum_required(VERSION 3.16)`).
- **Platform:** **Windows + Linux**. CMakeLists.txt links `ws2_32 mswsock` on
  Windows for Winsock; Linux uses POSIX sockets via Boost.Asio directly.
- **Testing:** **No test framework.** Validation is via `master/main_master.cpp`,
  a single executable that performs a first-contact integration test against
  a real SEL-735 (reads FID string, serial number, meter form, and
  communication counters; pass/fail printed to stdout).
- **Compiler warnings:** MSVC `/W4`; GCC/Clang `-Wall -Wextra -Wpedantic`
  (CMakeLists.txt:47-53).

---

## Current Implementation Status (as of 2026-04-29)

The repo is **not yet** the 6-layer design described below. Snapshot of what
exists vs. what's planned, derived from code analysis:

### Layout

```
Modbus_ASN/
├── master/         main_master.cpp, modbus_master.{cpp,hpp}, transport.{cpp,hpp}
├── slave/          main_asn1_slave.cpp, modbus_asn1_slave.{cpp,hpp}
├── asn/            asn1.hpp, modbus_asn1_tlv.hpp     (legacy — see ASN.1 status)
├── functions/      one .cpp per FC (master-side FC implementations)
├── modbus_common.hpp   (shared FC enums, MBAP, builders/parsers)
└── CMakeLists.txt
```

### What's done

- **Layer 0 (Transport)** — `TcpTransport` only. Synchronous blocking I/O.
  Per-call timeout via `SO_RCVTIMEO`/`SO_SNDTIMEO` (default 3000 ms,
  `master/transport.hpp:65`). `disconnect()` on any I/O error.
- **Layer 1 (Protocol)** — `Modbus::Master` class, 11 FCs implemented
  (01, 02, 03, 04, 05, 06, 0F, 10, 16, 17, 18). One translation unit per FC
  in `functions/`. Wire format is **standard Modbus TCP** — no ASN.1 on
  the wire (master skips it entirely; an earlier round-trip was removed as
  decorative — see `master/modbus_master.cpp:9-13`).
- **Slave** — listens on TCP, dispatches incoming PDUs through an
  `FcDispatcher` that internally round-trips through ASN.1 TLV
  (`slave/modbus_asn1_slave.cpp:351-392`). Wire format on both sides is
  standard Modbus; the TLV is purely internal and is identity-equivalent to
  the raw PDU (vestigial — see "ASN.1 status" below).
  Handlers registered for FC 01, 02, 03, 04, 05, 06, 0F, 10. **FC 16, 17, 18
  are NOT registered** — slave returns `ILLEGAL_FUNCTION` for them.

### What's missing

- **Serial / RS-485 transport** — no `SerialTransport` class exists. RTU
  framing, CRC-16, and inter-frame silent intervals are all unimplemented.
  Talking to RS-485-only relays will not work without writing this.
- **Layer 2 (Capability / Device Profile)** — not present. There's no
  capability cache, no FC fallback table, no per-vendor profiles. An
  unsupported FC will keep being retried on every call.
- **Layer 3 (Data Model / Point)** — not present. No tag database, no
  built-in 32-bit/float combine, no scaling factor application, no
  STRING/ENUM/BITMAP decoders. App code must combine and scale itself.
- **Layer 4 / 5** — not present.
- **Multi-slave support** — `Master` is bound to a single `Transport`. No
  `SlaveManager`, no per-slave health/state tracking.
- **Auto-chunking** — `readHoldingRegisters(addr, qty)` does not split
  requests above the 125-register limit; caller must respect it.
- **Capability cache / retry policy** — every call is independent. Timeout
  drops the entire socket (`master/transport.cpp:104-107`).

### Naming oddity to be aware of

Several files carry the `modbus_asn1_*` prefix from an earlier design where
ASN.1 TLV was on the wire. **It isn't anymore.** The slave still does an
internal TLV round-trip (no functional value, vestigial). The master has
none. When touching these files, consider whether a rename to drop the
`asn1` infix is appropriate — `modbus_asn1_common.hpp` was already renamed
to `modbus_common.hpp`.

### ASN.1 status

`asn/asn1.hpp` is a hand-rolled BER encoder/decoder. `asn/modbus_asn1_tlv.hpp`
wraps it as a per-FC TLV layer + dispatcher. **Neither is on the wire.**
Real protection relays (SEL, GE) speak standard Modbus and have no parser
for ASN.1 — the TLV layer was a closed-system experiment between this
repo's master and slave. The master no longer uses it; the slave's
`processRequest()` round-trips through it but returns standard Modbus PDUs.
Safe to delete the `asn/` folder + slave's TLV usage if the slave is
retired, since SEL/GE testing only exercises the master.

---

## Target Architecture — 6-Layer Design

The existing code may not yet be cleanly split this way. That's fine.
This is the **direction**, not the starting point. New code should land
in the right layer; old code is migrated only when touched for another reason.

```
┌─────────────────────────────────────────────────┐
│  Layer 5: Application / HMI / API                │  "Show Feeder-3 current on screen"
├─────────────────────────────────────────────────┤
│  Layer 4: Business Logic                         │  Scheduling, control workflows,
│          (orchestration, policies)               │  interlocks, alarm logic
├─────────────────────────────────────────────────┤
│  Layer 3: Data Model / Point Layer               │  Named points, data types,
│          (tag → register translation)            │  scaling, units, quality
├─────────────────────────────────────────────────┤
│  Layer 2: Capability / Device Profile Layer      │  Per-model profiles, FC
│          (what this device supports)             │  fallback, address maps
├─────────────────────────────────────────────────┤
│  Layer 1: Protocol Layer                         │  Modbus PDU build/parse,
│          (Modbus ADU/PDU, FC semantics, CRC-16)  │  exception-code handling
├─────────────────────────────────────────────────┤
│  Layer 0: Transport Layer                        │  RTU serial framing & timing,
│          (boost.asio serial / TCP)               │  TCP sockets, reconnection
└─────────────────────────────────────────────────┘
     ↕  Cross-cutting:  logging, metrics, config, security, health
```

### Layer Responsibilities (one-line each)

- **Layer 0 — Transport:** Owns `boost::asio::serial_port` /
  `boost::asio::ip::tcp::socket` / `boost::asio::steady_timer`. Handles
  RS-485 inter-frame silent intervals (≥3.5 char times), reconnect-on-drop,
  read/write timeouts. Exposes `send_bytes` / `recv_bytes` style primitives.
  **No Modbus knowledge.** This is the only layer that includes asio headers.
- **Layer 1 — Protocol:** Hand-rolled Modbus, **plain C++ only — no asio**.
  Builds the PDU byte-by-byte for each FC, computes CRC-16 (RTU) or wraps
  with MBAP header (TCP), parses replies, returns typed errors. Knows
  nothing about specific relays.
- **Layer 2 — Capability / Device Profile:** Per-model profile says which FCs
  the relay supports. Implements the FC fallback table.
- **Layer 3 — Data Model / Point:** Tag DB. Translates `FDR3.Ia` to
  `{slave, fc, addr, type, scale, unit}`. Handles 16/32-bit pairing,
  byte order, scaling, quality flags.
- **Layer 4 — Business Logic:** Polling schedules, control workflows,
  alarms, interlocks, setting-group switching.
- **Layer 5 — Application:** HMI, API, historian feed.

### The Asio Boundary Rule (important)

**Asio lives only in Layer 0.** Layers 1+ are plain C++ and must not
`#include <boost/asio.hpp>`. This keeps the protocol logic testable without
spinning up an `io_context`, and lets you swap the transport later (e.g.,
to a mock for unit tests, or to a different I/O library) without touching
protocol code.

The contract between Layer 0 and Layer 1 is byte-level:

```cpp
// Layer 0 exposes something like (sync example):
class Transport {
public:
    virtual ~Transport() = default;
    virtual void send(const std::vector<uint8_t>& frame) = 0;
    virtual std::vector<uint8_t> recv(std::chrono::milliseconds timeout) = 0;
};
```

Layer 1 (Protocol) takes a `Transport&` reference and never knows whether
it's talking to a real serial port, a TCP socket, or a mock.

### Cross-Cutting (present in all layers)

- **Logging** — plain C++ wrapper, write to file / stderr. Always include
  slave addr, FC, register, elapsed time.
- **Metrics** — simple counters in a struct, exposed via a status query.
- **Config** — flat text or hand-parsed key=value / INI / minimal JSON.
- **Security** — Modbus has no native auth. Guard writes at Layer 4.
- **Health state** — per-slave + per-point, surfaced upward.

---

## Golden Rules (non-negotiable)

1. **One-way dependency.** A layer only calls the one directly below it.
   No layer-skipping.
2. **Asio only in Layer 0.** No `#include <boost/asio.hpp>` above Layer 0.
3. **Fallback logic stays in Layer 2.** Layer 4 must never see
   "FC 04 not supported, retrying FC 03." Any `if (vendor == SEL)` in
   business logic is a bug.
4. **Every write response is checked.** Success reply ≠ correct write.
   Range-check on the master side before sending.
5. **Treat timeouts and Exception 01 as the same outcome:** capability
   missing. Some old slaves silently drop unknown FCs.
6. **Don't repeatedly poll an unsupported FC.** First Exception 01 →
   mark unsupported in cached profile → stop trying.
7. **Exception 02 (Illegal Data Address) marks the point bad — not the
   slave.** Keep polling the rest.
8. **No new dependencies without approval.** See Dependency Policy above.

---

## Modbus Wire Format — Quick Reference (we implement it ourselves)

### RTU Frame (over serial)

```
[ Slave Addr | Function Code |    Data    | CRC-Lo | CRC-Hi ]
   1 byte         1 byte       N bytes      1 byte    1 byte
```

- CRC-16 (Modbus polynomial 0xA001, reflected). Computed over slave addr +
  FC + data. Appended **low-byte first**.
- Frames separated by ≥3.5 character times of silence on the line.
  At 9600 8N1 that's ~4 ms; at 19200 it's ~2 ms; at 115200 it's ~0.3 ms.
- Inter-character gap inside a frame must be < 1.5 character times.
- Use `boost::asio::steady_timer` to enforce the 3.5-char silent gap
  between transactions.

### TCP Frame (over Ethernet)

```
[ MBAP Header (7 bytes) | Function Code | Data ]
```

MBAP header:
```
[ Transaction ID | Protocol ID | Length | Unit ID ]
    2 bytes         2 bytes      2 bytes   1 byte
```

- Transaction ID: master picks (often a counter), slave echoes. Used to
  match req/reply.
- Protocol ID: always 0x0000 for Modbus.
- Length: number of bytes following (Unit ID + FC + Data).
- Unit ID: same role as RTU slave address (often 0xFF when irrelevant
  through a TCP-direct device, or the actual slave ID through a gateway).
- **No CRC** in TCP — TCP itself provides integrity.

### CRC-16 Reference Implementation (plain C++)

```cpp
uint16_t modbus_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;  // append low byte first, then high byte
}
```

A faster table-driven variant is fine if performance matters — both produce
identical results.

---

## Modbus Function Codes — Reference

| Code | Width  | Description                  | Master | Slave | SEL-735 |
|------|--------|------------------------------|:------:|:-----:|:-------:|
| 01   | 1-bit  | Read coils                   | ✅     | ✅    | ✅       |
| 02   | 1-bit  | Read contacts (discrete in)  | ✅     | ✅    | ✅       |
| 05   | 1-bit  | Write single coil            | ✅     | ✅    | ✅       |
| 15   | 1-bit  | Write multiple coils         | ✅     | ✅    | ❌       |
| 03   | 16-bit | Read holding registers       | ✅     | ✅    | ✅ (max 125 regs) |
| 04   | 16-bit | Read input registers         | ✅     | ✅    | ✅ (max 125 regs) |
| 06   | 16-bit | Write single register        | ✅     | ✅    | ✅ (password-gated) |
| 16   | 16-bit | Write multiple registers     | ✅     | ❌    | ✅ (max 100 regs, password) |
| 22   | 16-bit | Mask write register          | ✅     | ❌    | ❌       |
| 23   | 16-bit | Read/write multiple registers| ✅     | ❌    | ❌       |
| 24   | 16-bit | Read FIFO queue              | ✅     | ❌    | ❌       |

Master-side I/O ranges per Modicon convention: coils 00001–10000,
discrete inputs 10001–20000, input registers 30001–40000, holding
registers 40001–50000. **On the wire, addresses are 0-based.**

### FC Fallback Table (Layer 2)

| Preferred FC | Fallback                              | Notes                          |
|--------------|----------------------------------------|--------------------------------|
| 04           | retry with 03                          | UR treats them as identical    |
| 06           | use 16 with quantity = 1               | Universally safe               |
| 15           | loop FC 05 per coil                    | Slower, always works           |
| 22           | FC 03 read → modify → FC 06/16 write   | NOT atomic — needs app lock    |
| 23           | FC 03 then FC 16                       | Costs one extra round trip     |
| 24           | use vendor-specific event/log regs     | FC 24 essentially never on relays |

### Modbus Exception Codes

| Code | Name                  | In `ExCode` enum? | App Action                                    |
|------|-----------------------|:-----------------:|-----------------------------------------------|
| 01   | Illegal Function      | ✅ ILLEGAL_FUNCTION | Mark FC unsupported on this slave (permanent) |
| 02   | Illegal Data Address  | ✅ ILLEGAL_DATA_ADDRESS | Mark this point unavailable; keep polling     |
| 03   | Illegal Data Value    | ✅ ILLEGAL_DATA_VALUE | Log offending value + source; programming bug |
| 04   | Slave Device Failure  | ✅ SERVER_DEVICE_FAILURE | Retry with backoff; alarm on N consecutive  |
| 06   | Slave Device Busy     | ❌ **MISSING**     | Retry later with backoff                      |
| 08   | Memory Parity Error   | ❌ **MISSING**     | SEL-735 returns this on stored-data checksum error |
| 0B   | Gateway Target Failed | ❌ **MISSING**     | Common via serial-to-Ethernet gateways        |

Exception reply format: FC byte has high bit set (e.g. FC 03 → 0x83),
followed by a 1-byte exception code. The current `Modbus::ExCode` enum
(`modbus_common.hpp:68-73`) covers only codes 01-04. Codes 06, 08, and 0B
will surface as "Unknown" in `exCodeStr()` — extending the enum is a small
follow-up that costs nothing.

---

## Target Relays — Vendor Notes

### SEL (SEL-751A, SEL-787, SEL-735, etc.)

- **Reads:** FC 01, 02, 03, 04. 32-bit values often `LONG100` (two regs,
  pre-scaled by 100).
- **Writes:** Control only. FC 05 to pulse outputs / set-clear remote bits
  (RB1..RBn). FC 06/16 on some models for specific command registers.
  **Setpoint editing via Modbus is NOT supported** — use AcSELerator.
- **Transport:** RTU on RS-485 (up to 115.2 kbps), TCP on Ethernet models.
- **Addressing:** Manuals show 4xxxx (Modicon); on the wire, 0-based.

#### SEL-735 specifics (verified against `735_IM_20231006.pdf`, Appendix E)

- **Supported FCs (Table E.2):** 01, 02, 03, 04, 05, 06, 10. **NOT
  supported:** 0F, 16, 17, 18 — these will return Exception 01.
- **Read limits:** 2000 bits per FC 01/02 query; **125 registers** per FC
  03/04; **100 registers** per FC 10.
- **Modbus TCP:** Port 502, up to **5 simultaneous sessions**, requires
  Ethernet card. Standard MBAP with `Protocol ID = 0`.
- **Exception codes used (Table E.3):** 01, 02, 03, 04, 06, 08. Code 06 =
  "Busy" (transient — retry); code 08 = "Memory Error" (stored data
  checksum failed).
- **Data types (Table E.23):** `INT`, `INTx` (with scale 10/100/1000),
  `UINT`, `UINTx`, `LONG` (32-bit signed), `LONGy` (with scale
  10/100/1000/10000), `BITMAP`, `ENUM`, `STRING` (null-terminated ASCII).
- **Word order for 32-bit:** *"Most significant word in lower address
  register"* — i.e., big-endian word order (high word first / `ABCD`).
  Naturally correct given the rest of Modbus is big-endian.
- **STRING decoding:** each register holds two ASCII chars; high byte is
  the first char. Loop until NUL byte.
- **Settable parameters require password handshake** (Section E.8-E.9):
  write Access Level E password to addr 70-74 via FC 10, then issue write,
  then save by writing `0x0001` to addr 76. Access times out after 15 min
  of inactivity. Without this handshake, writes return Exception 04
  ("Device Error — Invalid Access Level"). **Not yet implemented in code.**
- **Control I/O / reset commands (addr 75-80) DO NOT need a password** —
  empirically verified 2026-04-29 on R206-V1 firmware: FC 06 to addr 78
  (Reset Comm Counters) and FC 10 to addr 79-80 (Reset Max/Min + Reset
  Peak) both succeeded without any access-level setup. The password
  requirement applies only to *settable parameters* (MID, TID, Password
  registers, Device Time, User Map Registers) — not to control commands.
- **First-contact identity registers** (Table E.26 sheet 1, useful for
  initial validation):
  - `0..19` Firmware Identifier (STRING)
  - `20..39` Serial Number (STRING)
  - `62` Meter Form (ENUM: 0=Form 9, 1=Form 5, 2=Form 36)
  - `100..149` Device Word Bit Status (BITMAP)
  - `160..168` Communication Counters (UINT — sanity check)

### GE (Multilin UR, 750/760, 489, 8-Series)

- **Reads:** FC 01, 03, 04. UR explicitly treats FC 03 ≡ FC 04.
- **Writes:** Goes further than SEL.
  - **FC 05** — Execute Operation (trip / close / reset / virtual inputs)
  - **FC 06** — Store Single Setting (write one setpoint)
  - **FC 16** — Store Multiple Settings (block-write setpoints)
- 16-bit big-endian, max 125 registers per read.
- **Transport:** RTU on RS-232/RS-485 + Modbus TCP. Default port 502.
- **Caution:** Setpoint writes are powerful — guard at Layer 4.

---

## C++ / Asio Conventions

- **Error handling:** **`std::runtime_error` exceptions** at the public
  API. Asio is used internally with **error-code overloads** (e.g.,
  `asio::write(*socket_, buf, ec)` in `master/transport.cpp:89`); on `ec`
  the transport disconnects and rethrows as `std::runtime_error` with a
  human-readable message. Modbus exception responses (FC byte high bit set)
  are detected in `master/modbus_master.cpp:96-101` and rethrown as
  `std::runtime_error("Modbus exception: <code>")`. Callers must `try/catch`.
  **Caveat:** error type is currently a string — a typed `ModbusException`
  carrying the `ExCode` is defined in `asn/modbus_asn1_tlv.hpp:143` but is
  not used by the master. Consider switching when capability cache logic
  is added (avoids string parsing).
- **Asio usage:**
  - One `boost::asio::io_context` per transport instance (or one shared
    across the app — match existing pattern).
  - Prefer the **error-code overloads** of asio operations
    (`socket.read_some(buffer, ec)`) over the throwing overloads when the
    surrounding code returns status — keep error handling uniform.
  - Use `boost::asio::steady_timer` (not `wait_for` on a mutex or sleep)
    for the 3.5-char RS-485 silent interval — accurate and async-friendly.
  - For sync code, `boost::asio::read` / `write` with timeouts via timers
    is fine. For async code, callbacks or `boost::asio::awaitable`
    (C++20) — match the existing style.
- **Ownership:** RAII. `std::unique_ptr` / `std::shared_ptr` over raw owning
  pointers. Asio objects are not copyable — pass by reference or store as
  members.
- **Strings:** `std::string` / `std::string_view`. C strings only when
  required by OS APIs.
- **Buffers:** `std::vector<uint8_t>` or `std::array<uint8_t, N>` for Modbus
  frames. Wrap with `boost::asio::buffer(...)` when handing to asio.
- **Endianness:** Modbus is **big-endian on the wire**. Always serialize
  16-bit values explicitly with `(hi << 8) | lo` rather than `memcpy`-ing
  a `uint16_t` (which is host-endian).
- **Threading:** **Master is fully synchronous, single-threaded.** Each
  Modbus transaction is a blocking `send`/`recv` round-trip. There is no
  `io_context.run()` loop on the master side — `boost::asio` is used in
  blocking mode. The slave runs an `io_context` with `async_accept` and
  one `TcpSession` per client, also single-threaded. No strands, no
  thread pool. Adding parallel polling for multiple slaves will require
  switching to async (asio coroutines or callbacks).
- **Headers:** Match existing include-guard or `#pragma once` style.
- **Naming:** Match existing.
- **Const-correctness:** Mandatory on new code.

---

## What to Do When Asked to Make a Change

1. **Identify the layer.** If it's I/O work, it's Layer 0 (asio allowed).
   If it's protocol/CRC/PDU work, it's Layer 1 (plain C++ only — no asio).
2. **Read the surrounding code** before editing — match style, error handling,
   patterns.
3. **Make the smallest change that works.** Don't refactor opportunistically.
4. **No new dependencies** without explicit approval.
5. **Add or update a test.** Layer 1 protocol code should be testable
   without asio — feed it a mock `Transport` with hand-crafted byte vectors.
6. **Update this `CLAUDE.md`** if the change introduces a new rule, vendor
   quirk, or convention worth remembering.
7. **Show the diff and explain it.** Don't silently rewrite files.

---

## Notes on This File

- Living document. Update as the codebase evolves.
- New relay model → add a section under "Target Relays."
- New rule learned the hard way → add it to "Golden Rules."
- Fields marked _[FILL IN]_ should be replaced with real values from the
  actual repo on the first session.
