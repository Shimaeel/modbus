#pragma once
/**
 * @file verify.hpp
 * @brief Round-trip + counter verification helpers for SEL-735.
 *
 * @details
 * Manager's ask:
 *   1. "Is there a R/W holding register and read/write counters in SEL?"
 *   2. "Try this with your code and see write + read gives the same result."
 *
 * On SEL-735, addresses **160..168** (Communication Counters, per
 * CLAUDE.md and Table E.26) increment as messages flow. They are the
 * cleanest way to prove a request actually reached the relay — far more
 * trustworthy than "no exception thrown".
 *
 * Two helpers are exposed:
 *
 *   - `verifyReadRoundTrip()`     — read addr 0..N twice, compare; also
 *                                   check counter incremented by ≥2.
 *
 *   - `verifyWriteRoundTrip()`    — read counters, do a single-register
 *                                   write (FC 06), read counters again,
 *                                   confirm increment. For genuinely R/W
 *                                   addresses, also reads the value back
 *                                   (FC 03) and compares.
 *
 * Neither helper changes protection settings — they only target Control
 * I/O / counter registers that are safe to operate per CLAUDE.md.
 */

#include <cstdint>
#include <string>
#include <vector>

namespace Modbus {

class Master;  // forward

/** @brief Outcome bundle returned by verify functions. Plain struct so the UI
 *         can render fields without pulling in the implementation. */
struct VerifyResult {
    bool                    ok            {false};
    std::vector<uint16_t>   countersBefore;     ///< 9 regs from addr 160
    std::vector<uint16_t>   countersAfter;      ///< 9 regs from addr 160
    int                     totalCounterDelta {0};   ///< sum of (after - before) per reg
    bool                    readBackMatched   {false}; ///< only meaningful for write tests
    uint16_t                writtenValue      {0};
    uint16_t                readBackValue     {0};
    std::string             note;
};

/**
 * @brief Read counters → identity read → counters → confirm increment.
 * @param m       Connected master (SEL-735).
 * @return        Result struct; `ok` is true when counter delta ≥ 2.
 *
 * The "≥2" target reflects: 1 message for the second counter read +
 * (typically) 1+ for the identity read. Exact counter semantics depend on
 * which of the 9 counters represent "messages received" — we sum the delta
 * across all 9 to stay robust against per-counter ambiguity.
 */
VerifyResult verifyReadRoundTrip(Master& m);

/**
 * @brief Write a single register and verify via counter delta.
 *
 * @param m              Connected master (SEL-735).
 * @param writeAddr      0-based register address. **Caller is responsible**
 *                       for picking a safe address. Defaults are tuned for
 *                       Control I/O (addr 79 = Reset Max/Min) which is
 *                       password-free and side-effect-bounded.
 * @param writeValue     Value to write (default 0x0001).
 * @param readBackAddr   If non-zero, read this address via FC 03 after the
 *                       write and compare to `writeValue`. For pure
 *                       command registers (78/79/80) leave at 0 — they are
 *                       not meaningfully readable. For genuine R/W
 *                       registers (e.g. User Map), set to the same as
 *                       writeAddr to confirm value persists.
 *
 * @return Result struct. `ok` true when counter delta confirms the write
 *         reached the relay (and, if readback requested, the values match).
 */
VerifyResult verifyWriteRoundTrip(Master&  m,
                                  uint16_t writeAddr    = 79,
                                  uint16_t writeValue   = 0x0001,
                                  uint16_t readBackAddr = 0);

} // namespace Modbus
