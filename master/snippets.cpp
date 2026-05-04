/**
 * @file snippets.cpp
 * @brief Implementation of read snippets that populate SharedMemory.
 */

#include "snippets.hpp"
#include "modbus_master.hpp"
#include <stdexcept>
#include <string>

namespace Modbus {

namespace {

/**
 * @brief Decode a SEL-735 register block (high byte = first ASCII char) into
 *        a NUL-terminated string. Per CLAUDE.md: each register holds two ASCII
 *        chars, high byte first; loop until a NUL byte.
 */
std::string regsToString(const std::vector<uint16_t>& regs)
{
    std::string out;
    out.reserve(regs.size() * 2);
    for (uint16_t r : regs) {
        char hi = static_cast<char>((r >> 8) & 0xFF);
        char lo = static_cast<char>( r       & 0xFF);
        if (hi == '\0') break;
        out.push_back(hi);
        if (lo == '\0') break;
        out.push_back(lo);
    }
    // Trim trailing whitespace SEL relays sometimes pad with.
    while (!out.empty() && (out.back() == ' ' || out.back() == '\t'))
        out.pop_back();
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// identity.* — FID, serial, meter form
// ---------------------------------------------------------------------------
void readDeviceIdentity(Master& m, SharedMemory& sm)
{
    try {
        // FID  : addr 0..19  (20 regs, STRING)
        // SN   : addr 20..39 (20 regs, STRING)
        // Form : addr 62     (1 reg,  ENUM: 0=Form 9, 1=Form 5, 2=Form 36)
        // Reading the contiguous 0..62 block in one shot keeps it to one
        // round-trip (63 regs, well under SEL-735's 125 limit).
        auto regs = m.readHoldingRegisters(0, 63);

        std::vector<uint16_t> fidRegs   (regs.begin(),         regs.begin() + 20);
        std::vector<uint16_t> serialRegs(regs.begin() + 20,    regs.begin() + 40);
        uint16_t              meterForm = regs[62];

        sm.set("identity.fid",        regsToString(fidRegs));
        sm.set("identity.serial",     regsToString(serialRegs));
        sm.set("identity.meter_form", meterForm);
    }
    catch (const std::exception& e) {
        sm.markBad("identity.fid",        e.what());
        sm.markBad("identity.serial",     e.what());
        sm.markBad("identity.meter_form", e.what());
    }
}

// ---------------------------------------------------------------------------
// io.contact_inputs — FC 02 (6 contacts on SEL-735 Form 5)
// ---------------------------------------------------------------------------
void readContactInputs(Master& m, SharedMemory& sm)
{
    try {
        auto bits = m.readDiscreteInputs(0, 6);
        if (bits.size() > 6) bits.resize(6);   // strip byte-padding
        sm.set("io.contact_inputs", bits);
    }
    catch (const std::exception& e) {
        sm.markBad("io.contact_inputs", e.what());
    }
}

// ---------------------------------------------------------------------------
// io.contact_outputs — FC 01 (verified 2026-04-29: 23 coils on SEL-735)
// ---------------------------------------------------------------------------
void readContactOutputs(Master& m, SharedMemory& sm)
{
    try {
        auto bits = m.readCoils(0, 23);
        if (bits.size() > 23) bits.resize(23);
        sm.set("io.contact_outputs", bits);
    }
    catch (const std::exception& e) {
        sm.markBad("io.contact_outputs", e.what());
    }
}

// ---------------------------------------------------------------------------
// io.device_word — Device Word Bit Status bitmap (CLAUDE.md addr 100..149)
// ---------------------------------------------------------------------------
void readDeviceWordBitmap(Master& m, SharedMemory& sm)
{
    try {
        auto regs = m.readHoldingRegisters(100, 50);
        sm.set("io.device_word", regs);
    }
    catch (const std::exception& e) {
        sm.markBad("io.device_word", e.what());
    }
}

// ---------------------------------------------------------------------------
// counters.comm — Communication counters (CLAUDE.md addr 160..168, 9 regs)
// ---------------------------------------------------------------------------
void readCommCounters(Master& m, SharedMemory& sm)
{
    try {
        auto regs = m.readHoldingRegisters(160, 9);
        sm.set("counters.comm", regs);
    }
    catch (const std::exception& e) {
        sm.markBad("counters.comm", e.what());
    }
}

} // namespace Modbus
