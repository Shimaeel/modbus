/**
 * @file main_master.cpp
 * @brief Modbus Master single-function test — runs only FC 16 (Preset Multiple Registers).
 *
 * @details
 * Connects to the SEL-735 over Modbus TCP and exercises FC 16 (Preset
 * Multiple Registers, on-wire byte `0x10`) by writing two reset
 * commands in a single transaction:
 *
 * | Addr | Command (Table E.26)       | Value written |
 * |------|----------------------------|---------------|
 * | 79   | Reset Max/Min Values       | `0x0001`      |
 * | 80   | Reset Peak Demand          | `0x0001`      |
 *
 * Both registers live in the SEL-735 Control I/O area (75–80), which
 * empirically does not require an access-level password handshake.
 * The relay applies both resets atomically (from its perspective) in
 * one round-trip — that's the whole point of FC 16 vs looping FC 06.
 *
 * ### Why this target is safe
 * - Reset Max/Min only clears tracked min/max measurement values.
 * - Reset Peak Demand only clears the demand peak history.
 * - Neither changes any protection setting or actuates any output.
 *
 * ### Verification
 * The slave echoes `[FC, addr_hi, addr_lo, qty_hi, qty_lo]` on
 * success — `Master::transaction` already throws on any exception
 * response (high bit set on FC), so a clean return implies acceptance.
 * For Control I/O resets there's no easy "read-back" symmetric to FC
 * 06's counter check; protocol acceptance is the test bar here.
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

/**
 * @brief Entry point — write two reset commands via FC 16.
 * @return `0` on a successful round-trip; `1` on connect/IO failure.
 */
int main(int /*argc*/, char* /*argv*/[])
{
    constexpr const char* HOST    
              = "192.168.0.2";
    constexpr uint16_t    PORT              = Modbus::DEFAULT_PORT;   // 502
    constexpr uint8_t     UNIT_ID           = 1;
    constexpr int         TIMEOUT           = 5000;                   // ms
    constexpr uint16_t    RESET_MAXMIN_ADDR = 79;                     // Table E.26
    constexpr uint16_t    RESET_PEAK_ADDR   = 80;                     // Table E.26

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
    // FC 16 — Preset Multiple Registers (Reset Max/Min + Reset Peak Demand)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[FC 16] Writing 2 registers starting at address "
                  << RESET_MAXMIN_ADDR << "\n";
        std::cout << "  addr 79 = 0x0001  (Reset Max/Min Values)\n";
        std::cout << "  addr 80 = 0x0001  (Reset Peak Demand)\n";

        master.writeMultipleRegisters(RESET_MAXMIN_ADDR, {0x0001, 0x0001});

        std::cout << "\n[FC 16] Write accepted by relay.\n"
                  << "        Both reset commands executed in one transaction.\n";
    } catch (const std::exception& e) {
        std::string msg = e.what();
        if (msg.find("Server Device Failure") != std::string::npos) {
            std::cout << "\n[FC 16] Got Exception 04 (Device Error / Invalid Access Level).\n"
                      << "        Protocol verified — relay rejected on access-level policy.\n";
        } else {
            std::cerr << "[FC 16] Error: " << msg << '\n';
            master.disconnect();
            return 1;
        }
    }

    master.disconnect();
    return 0;
}
