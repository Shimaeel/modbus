# Modbus Client (Plain C++ + boost.asio) — Project Context

This file is the source of truth for design intent and rules in this codebase.
Claude Code reads this on every session start.

---

## Project Goal

Extend an **existing C++ Modbus client** that:

- Implements the **Modbus protocol from scratch** in plain C++ (no Modbus
  library — no libmodbus, no QModbus, no third-party Modbus stack).
- Uses **`boost.asio` for I/O** — serial port (RS-485) and TCP socket
  communication, timers, async event loop.

The goal is to robustly support a mixed fleet of protection relays (SEL, GE,
and possibly ABB / Siemens later) by making **targeted, minimal, well-tested
changes** that move the codebase toward the target architecture below.

We are NOT rewriting the client. We are NOT adding new libraries beyond
the ones already approved (see below).

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

- **Language:** C++ (assume C++17 unless `CMakeLists.txt` / compiler flags
  say otherwise — check first).
- **Modbus library:** **NONE — implemented from scratch in plain C++.**
- **I/O library:** `boost.asio` (serial port + TCP socket + timers).
- **Build system:** _[FILL IN — CMake / Makefile / MSBuild]_
- **Platform:** _[FILL IN — Linux / Windows / embedded]_
- **Testing:** _[FILL IN — most likely a hand-rolled test harness, since
  gtest/Catch2 are external. Confirm with user.]_

> Inspect the repo first (`CMakeLists.txt`, `Makefile`, `*.vcxproj`,
> `find_package(Boost ...)`) before assuming any of the `[FILL IN]` values.

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

| Code | Width  | Description                  | I/O Range       |
|------|--------|------------------------------|-----------------|
| 01   | 1-bit  | Read coils                   | 00001 – 10000   |
| 02   | 1-bit  | Read contacts (discrete in)  | 10001 – 20000   |
| 05   | 1-bit  | Write single coil            | 00001 – 10000   |
| 15   | 1-bit  | Write multiple coils         | 00001 – 10000   |
| 03   | 16-bit | Read holding registers       | 40001 – 50000   |
| 04   | 16-bit | Read input registers         | 30001 – 40000   |
| 06   | 16-bit | Write single register        | 40001 – 50000   |
| 16   | 16-bit | Write multiple registers     | 40001 – 50000   |
| 22   | 16-bit | Mask write register          | 40001 – 50000   |
| 23   | 16-bit | Read/write multiple registers| 40001 – 50000   |
| 24   | 16-bit | Read FIFO queue              | 40001 – 50000   |

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

| Code | Name                  | App Action                                    |
|------|-----------------------|-----------------------------------------------|
| 01   | Illegal Function      | Mark FC unsupported on this slave (permanent) |
| 02   | Illegal Data Address  | Mark this point unavailable; keep polling     |
| 03   | Illegal Data Value    | Log offending value + source; programming bug |
| 04   | Slave Device Failure  | Retry with backoff; alarm on N consecutive    |
| 06   | Slave Device Busy     | Retry later with backoff                      |
| 0B   | Gateway Target Failed | Common via serial-to-Ethernet gateways        |

Exception reply format: FC byte has high bit set (e.g. FC 03 → 0x83),
followed by a 1-byte exception code.

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

- **Error handling:** _[FILL IN — exceptions? status enums? `std::optional`?
  Match the existing code.]_ Asio supports both error-code and exception
  styles; pick whichever the existing transport code uses and stay consistent.
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
- **Threading:** _[FILL IN — single-threaded `io_context.run()`? Thread pool?
  Strands? Match existing.]_ Whatever the existing pattern, document it.
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
