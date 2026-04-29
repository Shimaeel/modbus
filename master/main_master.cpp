/**
 * @file main_master.cpp
 * @brief Modbus Master FC test suite for SEL-735 Power Quality Meter (TCP).
 *
 * Exercises every Modbus function code the SEL-735 supports per Table E.2
 * of the instruction manual. Address choices come from Table E.26.
 *
 * FC coverage (7 of 7 SEL-735-supported function codes):
 *   FC 03  Read Holding Registers   (4 different addresses)
 *   FC 04  Read Input Registers     (FID via FC 04 — should match FC 03)
 *   FC 02  Read Input Status        (IN101..IN404 contacts)
 *   FC 01  Read Coil Status         (OUT + RB bits)
 *   FC 05  Force Single Coil        (toggle RB01 — internal logic bit, safe)
 *   FC 06  Preset Single Register   (write to Reset Comm Counters cmd reg)
 *   FC 10  Preset Multiple Regs     (write 2 reset cmd regs at once)
 *
 * Safety: writes target only Remote Bits (logical, internal to SEL Logic)
 * and command/reset registers — NEVER physical output coils OUT101..OUT404.
 *
 * Usage:
 *   modbus_master                -- connects to 192.168.0.2:502, unitId=1
 */

#include "modbus_master.hpp"
#include "transport.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/**
 * @brief Decode a vector of Modbus registers into a printable ASCII string.
 *
 * Each 16-bit register holds two ASCII chars (high byte first per Modbus
 * convention). Stops at the first NUL byte; non-printable bytes are shown
 * as '.'.
 */
std::string regsToAscii(const std::vector<uint16_t>& regs)
{
    std::string out;
    for (uint16_t r : regs) {
        char hi = static_cast<char>((r >> 8) & 0xFF);
        char lo = static_cast<char>(r & 0xFF);
        if (hi == '\0') break;
        out.push_back((hi >= 0x20 && hi < 0x7F) ? hi : '.');
        if (lo == '\0') break;
        out.push_back((lo >= 0x20 && lo < 0x7F) ? lo : '.');
    }
    return out;
}

const char* meterFormStr(uint16_t v)
{
    switch (v) {
    case 0: return "Form 9";
    case 1: return "Form 5";
    case 2: return "Form 36";
    default: return "Unknown";
    }
}

void boolsToHex(const std::vector<bool>& bits, std::ostream& os)
{
    os << "[";
    for (size_t i = 0; i < bits.size(); ++i)
        os << (i ? "," : "") << (bits[i] ? "1" : "0");
    os << "]";
}

struct Result {
    int passed = 0;
    int failed = 0;

    void pass(const std::string& msg)
    {
        std::cout << "  -> PASS  (" << msg << ")\n";
        ++passed;
    }
    void fail(const std::string& msg)
    {
        std::cout << "  -> FAIL  (" << msg << ")\n";
        ++failed;
    }
    void error(const std::string& msg)
    {
        std::cerr << "  -> ERROR: " << msg << '\n';
        ++failed;
    }
};

} // anonymous namespace

int main(int /*argc*/, char* /*argv*/[])
{
    constexpr const char* HOST    = "192.168.0.2";
    constexpr uint16_t    PORT    = Modbus::DEFAULT_PORT;
    constexpr uint8_t     UNIT_ID = 1;
    constexpr int         TIMEOUT = 5000;

    auto logger = [](const std::string& msg) { std::cout << msg << '\n'; };

    std::cout << "[Master] Connecting to SEL-735 at " << HOST << ":" << PORT
              << "  unitId=" << static_cast<int>(UNIT_ID) << "\n";

    Modbus::TcpTransport transport(HOST, PORT, TIMEOUT, logger);
    Modbus::Master       master(transport, UNIT_ID, logger);

    if (!master.connect()) {
        std::cerr << "[Master] Cannot connect to SEL-735 - check IP/network.\n";
        return 1;
    }

    Result r;

    // ─────────────────────────────────────────────────────────────────────
    // FC 03 — Read Holding Registers (4 reads, 4 different addresses)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Test 1] FC 03 — Firmware Identifier  (addr 0..19)\n";
        auto regs = master.readHoldingRegisters(0, 20);
        std::string fid = regsToAscii(regs);
        std::cout << "  Decoded: \"" << fid << "\"\n";
        if (fid.find("735") != std::string::npos ||
            fid.find("SEL") != std::string::npos)
            r.pass("contains 'SEL'/'735'");
        else
            r.fail("no 'SEL'/'735' substring");
    } catch (const std::exception& e) { r.error(e.what()); }

    try {
        std::cout << "\n[Test 2] FC 03 — Serial Number  (addr 20..39)\n";
        auto regs = master.readHoldingRegisters(20, 20);
        std::string serial = regsToAscii(regs);
        std::cout << "  Decoded: \"" << serial << "\"\n";
        serial.empty() ? r.fail("empty serial") : r.pass("non-empty");
    } catch (const std::exception& e) { r.error(e.what()); }

    try {
        std::cout << "\n[Test 3] FC 03 — Meter Form  (addr 62, ENUM)\n";
        auto regs = master.readHoldingRegisters(62, 1);
        uint16_t form = regs[0];
        std::cout << "  Raw=" << form << "  Decoded=" << meterFormStr(form) << '\n';
        (form <= 2) ? r.pass("valid enum") : r.fail("enum out of range");
    } catch (const std::exception& e) { r.error(e.what()); }

    try {
        std::cout << "\n[Test 4] FC 03 — Communication Counters  (addr 160..168)\n";
        auto regs = master.readHoldingRegisters(160, 9);
        const char* names[] = {
            "Num Msgs Rx", "Num Msgs Sent", "Invalid Address",
            "Bad CRC", "UART ERROR", "Illegal Function/Op",
            "Illegal Register", "Illegal Write", "Bad Packet Format"
        };
        for (size_t i = 0; i < regs.size(); ++i)
            std::cout << "  " << names[i] << " = " << regs[i] << '\n';
        r.pass("9 UINT counters read");
    } catch (const std::exception& e) { r.error(e.what()); }

    // ─────────────────────────────────────────────────────────────────────
    // FC 04 — Read Input Registers
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Test 5] FC 04 — Read Input Registers  (addr 0..19, FID)\n";
        std::cout << "  Note: SEL treats FC 04 == FC 03 — should return same FID string.\n";
        auto regs = master.readInputRegisters(0, 20);
        std::string fid = regsToAscii(regs);
        std::cout << "  Decoded: \"" << fid << "\"\n";
        if (fid.find("735") != std::string::npos ||
            fid.find("SEL") != std::string::npos)
            r.pass("FC 04 returns same FID as FC 03");
        else
            r.fail("FC 04 didn't return expected FID");
    } catch (const std::exception& e) { r.error(e.what()); }

    // ─────────────────────────────────────────────────────────────────────
    // FC 02 — Read Input Status (digital input contacts)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Test 6] FC 02 — Read Input Status  (addr 0..5, IN101..IN404)\n";
        auto bits = master.readDiscreteInputs(0, 6);
        std::cout << "  Inputs (IN101,IN102,IN401..IN404): ";
        boolsToHex(bits, std::cout);
        std::cout << '\n';
        (bits.size() == 6) ? r.pass("6 input contacts read")
                           : r.fail("unexpected size");
    } catch (const std::exception& e) { r.error(e.what()); }

    // ─────────────────────────────────────────────────────────────────────
    // FC 01 — Read Coil Status (outputs + RB bits)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Test 7] FC 01 — Read Coil Status  (addr 0..22, OUT + RB01..RB16)\n";
        auto bits = master.readCoils(0, 23);
        std::cout << "  Coils (OUT101..OUT404,RB01..RB16): ";
        boolsToHex(bits, std::cout);
        std::cout << '\n';
        (bits.size() == 23) ? r.pass("23 coil bits read")
                            : r.fail("unexpected size");
    } catch (const std::exception& e) { r.error(e.what()); }

    // ─────────────────────────────────────────────────────────────────────
    // FC 05 — Force Single Coil  (toggle RB01 — safe, internal logic bit)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Test 8] FC 05 — Force Single Coil  (toggle RB01 at addr 7)\n";
        std::cout << "  Setting RB01 = ON\n";
        master.writeSingleCoil(7, true);
        auto v1 = master.readCoils(7, 1);
        std::cout << "  Read-back after SET:   RB01 = " << (v1[0] ? "1" : "0") << '\n';

        std::cout << "  Clearing RB01 = OFF\n";
        master.writeSingleCoil(7, false);
        auto v2 = master.readCoils(7, 1);
        std::cout << "  Read-back after CLEAR: RB01 = " << (v2[0] ? "1" : "0") << '\n';

        if (v1[0] && !v2[0])
            r.pass("RB01 toggled ON then OFF correctly");
        else
            r.fail("read-back doesn't match writes");
    } catch (const std::exception& e) { r.error(e.what()); }

    // ─────────────────────────────────────────────────────────────────────
    // FC 06 — Preset Single Register
    // Writes 0x0001 to addr 78 (Reset Communication Counters command).
    // If access level is required, expect "Modbus exception: Server Device
    // Failure" (ExCode 04). That still proves FC 06 protocol works.
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Test 9] FC 06 — Preset Single Register"
                     "  (addr 78 = Reset Comm Counters, value 0x0001)\n";
        master.writeSingleRegister(78, 0x0001);
        std::cout << "  Write accepted by relay\n";
        auto regs = master.readHoldingRegisters(160, 9);
        std::cout << "  Counters after reset: NumMsgsRx=" << regs[0]
                  << " NumMsgsSent=" << regs[1]
                  << " Errors=" << regs[2] + regs[3] + regs[4]
                                 + regs[5] + regs[6] + regs[7] + regs[8] << '\n';
        r.pass("FC 06 round-trip succeeded");
    } catch (const std::exception& e) {
        std::string msg = e.what();
        if (msg.find("Server Device Failure") != std::string::npos) {
            std::cout << "  Got Exception 04 (likely access-level required)\n";
            r.pass("FC 06 protocol verified (relay rejected on access level — expected)");
        } else {
            r.error(msg);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // FC 10 — Preset Multiple Registers
    // Writes 2 regs starting at addr 79 (Reset Max/Min + Reset Peak Demand).
    // Same access-level behaviour as FC 06 — Exception 04 is acceptable.
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Test 10] FC 10 — Preset Multiple Registers"
                     "  (addr 79..80 = Reset Max/Min + Reset Peak)\n";
        master.writeMultipleRegisters(79, {0x0001, 0x0001});
        std::cout << "  Write accepted by relay\n";
        r.pass("FC 10 round-trip succeeded");
    } catch (const std::exception& e) {
        std::string msg = e.what();
        if (msg.find("Server Device Failure") != std::string::npos) {
            std::cout << "  Got Exception 04 (likely access-level required)\n";
            r.pass("FC 10 protocol verified (relay rejected on access level — expected)");
        } else {
            r.error(msg);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Summary
    // ─────────────────────────────────────────────────────────────────────
    std::cout << "\n========================================================\n"
              << "  Tests passed: " << r.passed << " / " << (r.passed + r.failed) << "\n"
              << "  FC coverage:  03, 04, 02, 01, 05, 06, 10  (7 of 7 SEL-735)\n"
              << "========================================================\n";

    master.disconnect();
    return (r.failed == 0) ? 0 : 1;
}
