/**
 * @file main_master.cpp
 * @brief Modbus Master single-function test — runs only FC 01 (Read Coils).
 *
 * @details
 * Connects to the SEL-735 over Modbus TCP, performs a single FC 01
 * (Read Coils) call covering all 23 coil bits per the SEL-735 manual
 * Table E.15 (`OUT101..OUT404` + `RB01..RB16`), prints the decoded
 * states, and exits.
 *
 * This is intentionally one-FC-at-a-time. To test a different function
 * code, swap the call to `master.readCoils(...)` for the FC you want to
 * exercise, rebuild, and run.
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
 * @brief Print a boolean vector with friendly labels for the SEL-735
 *        coil layout (per Table E.15 of the manual).
 *
 * Indices 0..6  → physical outputs (`OUT101..OUT103`, `OUT401..OUT404`).
 * Indices 7..22 → Remote Bits (`RB01..RB16`, internal SELOGIC bits).
 */
void printCoils(const std::vector<bool>& bits)
{
    static const char* names[23] = {
        "OUT101", "OUT102", "OUT103",
        "OUT401", "OUT402", "OUT403", "OUT404",
        "RB01", "RB02", "RB03", "RB04", "RB05", "RB06", "RB07", "RB08",
        "RB09", "RB10", "RB11", "RB12", "RB13", "RB14", "RB15", "RB16"
    };
    for (size_t i = 0; i < bits.size() && i < 23; ++i) {
        std::cout << "  " << names[i] << " = " << (bits[i] ? "1 (ON) " : "0 (off)") << '\n';
    }
}

} // anonymous namespace

/**
 * @brief Entry point — read 23 coils via FC 01 and print their state.
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
    // FC 01 — Read Coil Status (OUT101..OUT404 + RB01..RB16)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[FC 01] Reading 23 coils starting at address 0...\n";
        auto coils = master.readCoils(0, 23);
        std::cout << "\nCoil status:\n";
        printCoils(coils);
        std::cout << "\n[FC 01] Read complete.\n";
    } catch (const std::exception& e) {
        std::cerr << "[FC 01] Error: " << e.what() << '\n';
        master.disconnect();
        return 1;
    }

    master.disconnect();
    return 0;
}
