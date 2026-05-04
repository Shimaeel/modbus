/**
 * @file verify.cpp
 * @brief Implementation of round-trip + counter verification helpers.
 */

#include "verify.hpp"
#include "modbus_master.hpp"
#include <numeric>
#include <stdexcept>

namespace Modbus {

namespace {

constexpr uint16_t COMM_COUNTER_ADDR  = 160;   ///< per CLAUDE.md / Table E.26
constexpr uint16_t COMM_COUNTER_COUNT = 9;

/** Sum of element-wise (after - before), tolerating uint wrap. */
int counterDelta(const std::vector<uint16_t>& before,
                 const std::vector<uint16_t>& after)
{
    int total = 0;
    size_t n  = std::min(before.size(), after.size());
    for (size_t i = 0; i < n; ++i) {
        // 16-bit unsigned wrap-aware diff — counters can roll over.
        uint16_t d = static_cast<uint16_t>(after[i] - before[i]);
        total += static_cast<int>(d);
    }
    return total;
}

} // namespace

// ---------------------------------------------------------------------------
// verifyReadRoundTrip
// ---------------------------------------------------------------------------
VerifyResult verifyReadRoundTrip(Master& m)
{
    VerifyResult r;
    try {
        r.countersBefore = m.readHoldingRegisters(COMM_COUNTER_ADDR, COMM_COUNTER_COUNT);

        // Two identity reads — if the relay is talking, these should be
        // identical and the counters should advance.
        auto fid1 = m.readHoldingRegisters(0, 20);
        auto fid2 = m.readHoldingRegisters(0, 20);

        r.countersAfter      = m.readHoldingRegisters(COMM_COUNTER_ADDR, COMM_COUNTER_COUNT);
        r.totalCounterDelta  = counterDelta(r.countersBefore, r.countersAfter);

        bool fidStable = (fid1 == fid2);
        // We did 3 extra messages between the two counter snapshots
        // (fid1, fid2, second-counter-read). Expect delta >= 3 on the
        // "messages received" counter; we sum all 9 to be robust.
        r.ok   = fidStable && r.totalCounterDelta >= 3;
        r.note = fidStable
            ? "FID stable across two reads; counter delta=" + std::to_string(r.totalCounterDelta)
            : "FID mismatch between back-to-back reads — relay is unstable or reply-stream corrupt";
    }
    catch (const std::exception& e) {
        r.ok   = false;
        r.note = std::string("verifyReadRoundTrip threw: ") + e.what();
    }
    return r;
}

// ---------------------------------------------------------------------------
// verifyWriteRoundTrip
// ---------------------------------------------------------------------------
VerifyResult verifyWriteRoundTrip(Master&  m,
                                  uint16_t writeAddr,
                                  uint16_t writeValue,
                                  uint16_t readBackAddr)
{
    VerifyResult r;
    r.writtenValue = writeValue;
    try {
        r.countersBefore = m.readHoldingRegisters(COMM_COUNTER_ADDR, COMM_COUNTER_COUNT);

        // The actual write under test.
        m.writeSingleRegister(writeAddr, writeValue);

        if (readBackAddr != 0) {
            auto v = m.readHoldingRegisters(readBackAddr, 1);
            r.readBackValue   = v.empty() ? 0 : v[0];
            r.readBackMatched = (r.readBackValue == writeValue);
        } else {
            // Command registers (78/79/80) don't read back the written value —
            // they latch a side-effect, not a value. Skip readback compare.
            r.readBackMatched = true;
        }

        r.countersAfter      = m.readHoldingRegisters(COMM_COUNTER_ADDR, COMM_COUNTER_COUNT);
        r.totalCounterDelta  = counterDelta(r.countersBefore, r.countersAfter);

        // Minimum messages between counter snapshots: write (1) + counter
        // re-read (1), plus optional readback (1). Floor expected at 2.
        int minExpected = (readBackAddr != 0) ? 3 : 2;
        r.ok = r.readBackMatched && r.totalCounterDelta >= minExpected;
        r.note = "write FC 06 -> counter delta=" + std::to_string(r.totalCounterDelta)
               + (readBackAddr ? (r.readBackMatched ? "; readback matched"
                                                    : "; readback MISMATCH")
                               : "; readback skipped (command register)");
    }
    catch (const std::exception& e) {
        r.ok   = false;
        r.note = std::string("verifyWriteRoundTrip threw: ") + e.what();
    }
    return r;
}

} // namespace Modbus
