/**
 * @file main_master.cpp
 * @brief Modbus Master single-function test — runs only FC 03 (Read Holding Registers).
 *
 * @details
 * Connects to the SEL-735 over Modbus TCP and performs a single FC 03
 * (Read Holding Registers) call to fetch the **Firmware Identifier**
 * string. Per Table E.26 of the SEL-735 manual, addresses 0..19 hold
 * the FID as a 20-register STRING (40 bytes, NUL-terminated, two ASCII
 * chars per register, high byte first).
 *
 * A successful read of the FID is the canonical "first contact" test
 * for any Modbus master — it exercises the entire stack (TCP, MBAP, FC
 * 03 framing, big-endian byte order, address mapping) and returns
 * data with a known expected pattern (`"SEL-735-..."`), so any
 * corruption shows up immediately.
 *
 * ### Usage
 * @code
 *   $ ./modbus_master
 * @endcode
 *
 * Defaults: host `192.168.0.2`, port `502`, unitId `1`.
 */

#include "modbus_master.hpp"
#include "transport.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/**
 * @brief Decode a register vector into a printable ASCII string.
 *
 * Each 16-bit register holds two ASCII characters with the **high byte
 * first** (Modbus convention). Stops at the first NUL byte so it works
 * for the SEL-735's NUL-terminated string fields. Non-printable bytes
 * are shown as `.`.
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

} // anonymous namespace

/**
 * @brief Entry point — read the SEL-735 Firmware Identifier via FC 03.
 * @return `0` on success, `1` on connect or transaction failure.
 */
int main(int /*argc*/, char* /*argv*/[])
{
    constexpr const char* HOST    = "192.168.0.2";
    constexpr uint16_t    PORT    = Modbus::DEFAULT_PORT;   // 502
    constexpr uint8_t     UNIT_ID = 1;
    constexpr int         TIMEOUT = 5000;                   // ms

    auto logger = [](const std::string& msg) { std::cout << msg << '\n'; };

    std::cout << "[Master] Connecting to SEL-735 at " << HOST << ":" << PORT
              << "  unitId=" << static_cast<int>(UNIT_ID) << "\n";

    Modbus::TcpTransport transport(HOST, PORT, TIMEOUT, logger);
    Modbus::Master       master(transport, UNIT_ID, logger);

    if (!master.connect()) {
        std::cerr << "[Master] Cannot connect — check IP/network.\n";
        return 1;
    }

    // ─────────────────────────────────────────────────────────────────────
    // FC 03 — Read Holding Registers (Firmware Identifier, addr 0..19, STRING)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[FC 03] Reading 20 holding registers starting at address 0...\n";
        auto regs = master.readHoldingRegisters(0, 20);

        std::cout << "\nRaw 16-bit register values (decimal):\n";
        for (size_t i = 0; i < regs.size(); ++i)
            std::cout << "  reg[" << i << "] = " << regs[i] << '\n';

        std::cout << "\nDecoded as ASCII (Firmware Identifier):\n"
                  << "  \"" << regsToAscii(regs) << "\"\n";

        std::cout << "\n[FC 03] Read complete.\n";
    } catch (const std::exception& e) {
        std::cerr << "[FC 03] Error: " << e.what() << '\n';
        master.disconnect();
        return 1;
    }

    master.disconnect();
    return 0;
}
