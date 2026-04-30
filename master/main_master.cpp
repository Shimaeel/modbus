/**
 * @file main_master.cpp
 * @brief Modbus Master single-function test — runs only FC 04 (Read Input Registers).
 *
 * @details
 * Connects to the SEL-735 over Modbus TCP and performs a single FC 04
 * (Read Input Registers) call to fetch the **Firmware Identifier**
 * string at addresses 0..19.
 *
 * Per the SEL-735 manual (verified empirically), the relay treats
 * **FC 04 identically to FC 03** — both functions index into the same
 * Modbus Register Map (Table E.26). Reading the FID via FC 04 should
 * therefore return exactly the same `"SEL-735-..."` string that FC 03
 * returned in the previous run. This confirms the FC 04 wire framing
 * works and that the SEL-equivalence quirk holds on this firmware.
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
 * first** (Modbus convention). Stops at the first NUL byte; non-printable
 * bytes are shown as `.`.
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
 * @brief Entry point — read the SEL-735 Firmware Identifier via FC 04.
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
    // FC 04 — Read Input Registers (FID via FC 04, addr 0..19)
    // SEL-735 treats FC 04 == FC 03, so the result should be the FID string.
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[FC 04] Reading 20 input registers starting at address 0...\n";
        auto regs = master.readInputRegisters(0, 20);

        std::cout << "\nRaw 16-bit register values (decimal):\n";
        for (size_t i = 0; i < regs.size(); ++i)
            std::cout << "  reg[" << i << "] = " << regs[i] << '\n';

        std::cout << "\nDecoded as ASCII (Firmware Identifier):\n"
                  << "  \"" << regsToAscii(regs) << "\"\n";

        std::cout << "\n[FC 04] Read complete.\n";
        std::cout << "  Note: SEL-735 maps FC 04 to the same area as FC 03,\n"
                  << "        so this should match what FC 03 returned.\n";
    } catch (const std::exception& e) {
        std::cerr << "[FC 04] Error: " << e.what() << '\n';
        master.disconnect();
        return 1;
    }

    master.disconnect();
    return 0;
}
