/**
 * @file main_master.cpp
 * @brief Modbus Master first-contact test for SEL-735 Power Quality Meter (TCP).
 *
 * Reads identification registers from a SEL-735 over Modbus TCP and decodes
 * them against the published register map (Appendix E, Table E.26 of the
 * SEL-735 instruction manual). A successful run proves the full protocol
 * stack (TCP, MBAP, FC 03, byte order, address mapping) end-to-end.
 *
 * Tested registers:
 *   0..19   Firmware Identifier (STRING, 20 regs)
 *   20..39  Serial Number       (STRING, 20 regs)
 *   62      Meter Form          (ENUM:  0=Form 9, 1=Form 5, 2=Form 36)
 *   160..168 Communication Counters (UINT, 9 regs) — numeric sanity check
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

/**
 * @brief Decode the Meter Form ENUM (per Table E.24 of SEL-735 manual).
 */
const char* meterFormStr(uint16_t v)
{
    switch (v) {
    case 0: return "Form 9";
    case 1: return "Form 5";
    case 2: return "Form 36";
    default: return "Unknown";
    }
}

} // anonymous namespace

int main(int /*argc*/, char* /*argv*/[])
{
    constexpr const char* HOST    = "192.168.0.2";
    constexpr uint16_t    PORT    = Modbus::DEFAULT_PORT;   // 502
    constexpr uint8_t     UNIT_ID = 1;
    constexpr int         TIMEOUT = 5000;                   // ms

    auto logger = [](const std::string& msg) {
        std::cout << msg << '\n';
    };

    std::cout << "[Master] Connecting to SEL-735 at " << HOST << ":" << PORT
              << "  unitId=" << static_cast<int>(UNIT_ID) << "\n\n";

    Modbus::TcpTransport transport(HOST, PORT, TIMEOUT, logger);
    Modbus::Master       master(transport, UNIT_ID, logger);

    if (!master.connect()) {
        std::cerr << "[Master] Cannot connect to SEL-735 - check IP/network.\n";
        return 1;
    }

    int passed = 0;
    int failed = 0;

    // ----------------------------------------------------------------
    // Test 1: Firmware Identifier (address 0..19, STRING)
    // ----------------------------------------------------------------
    try {
        std::cout << "\n[Test 1] Firmware Identifier  (addr 0..19, STRING)\n";
        auto regs = master.readHoldingRegisters(0, 20);
        std::string fid = regsToAscii(regs);
        std::cout << "  Decoded: \"" << fid << "\"\n";
        if (fid.find("735") != std::string::npos ||
            fid.find("SEL") != std::string::npos) {
            std::cout << "  -> PASS  (expected 'SEL' or '735' substring)\n";
            ++passed;
        } else {
            std::cout << "  -> FAIL  (no 'SEL' or '735' in decoded string)\n";
            ++failed;
        }
    } catch (const std::exception& e) {
        std::cerr << "  -> ERROR: " << e.what() << '\n';
        ++failed;
    }

    // ----------------------------------------------------------------
    // Test 2: Serial Number (address 20..39, STRING)
    // ----------------------------------------------------------------
    try {
        std::cout << "\n[Test 2] Serial Number  (addr 20..39, STRING)\n";
        auto regs = master.readHoldingRegisters(20, 20);
        std::string serial = regsToAscii(regs);
        std::cout << "  Decoded: \"" << serial << "\"\n";
        if (!serial.empty()) {
            std::cout << "  -> PASS  (non-empty serial)\n";
            ++passed;
        } else {
            std::cout << "  -> FAIL  (empty serial)\n";
            ++failed;
        }
    } catch (const std::exception& e) {
        std::cerr << "  -> ERROR: " << e.what() << '\n';
        ++failed;
    }

    // ----------------------------------------------------------------
    // Test 3: Meter Form (address 62, ENUM)
    // ----------------------------------------------------------------
    try {
        std::cout << "\n[Test 3] Meter Form  (addr 62, ENUM)\n";
        auto regs = master.readHoldingRegisters(62, 1);
        uint16_t form = regs[0];
        std::cout << "  Raw value: " << form
                  << "  Decoded: " << meterFormStr(form) << '\n';
        if (form <= 2) {
            std::cout << "  -> PASS  (valid enum value)\n";
            ++passed;
        } else {
            std::cout << "  -> FAIL  (enum out of range)\n";
            ++failed;
        }
    } catch (const std::exception& e) {
        std::cerr << "  -> ERROR: " << e.what() << '\n';
        ++failed;
    }

    // ----------------------------------------------------------------
    // Test 4: Communication Counters (address 160..168, UINT)
    // ----------------------------------------------------------------
    try {
        std::cout << "\n[Test 4] Communication Counters  (addr 160..168, UINT)\n";
        auto regs = master.readHoldingRegisters(160, 9);
        const char* names[] = {
            "Num Msgs Rx", "Num Msgs Sent", "Invalid Address",
            "Bad CRC", "UART ERROR", "Illegal Function/Op",
            "Illegal Register", "Illegal Write", "Bad Packet Format"
        };
        for (size_t i = 0; i < regs.size(); ++i)
            std::cout << "  " << names[i] << " = " << regs[i] << '\n';
        std::cout << "  -> PASS  (numeric reads succeeded)\n";
        ++passed;
    } catch (const std::exception& e) {
        std::cerr << "  -> ERROR: " << e.what() << '\n';
        ++failed;
    }

    // ----------------------------------------------------------------
    // Summary
    // ----------------------------------------------------------------
    std::cout << "\n========================================\n"
              << "  Tests passed: " << passed << " / " << (passed + failed)
              << "\n========================================\n";

    master.disconnect();
    return (failed == 0) ? 0 : 1;
}
