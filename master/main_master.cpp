/**
 * @file main_master.cpp
 * @brief Modbus Master single-function test — runs only FC 02 (Read Discrete Inputs).
 *
 * @details
 * Connects to the SEL-735 over Modbus TCP, performs a single FC 02
 * (Read Discrete Inputs) call covering the 6 input contacts per the
 * SEL-735 manual Table E.8 (`IN101, IN102, IN401..IN404`), prints the
 * decoded states, and exits.
 *
 * Discrete inputs are **read-only** 1-bit signals coming **into** the
 * relay from external wiring (breaker auxiliary contacts, manual
 * switches, status from other equipment). Inputs that are not
 * physically installed return `0`.
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
 *        discrete-input layout (per Table E.8 of the manual).
 *
 * Indices 0..5 → physical inputs `IN101, IN102, IN401..IN404`.
 * Inputs that aren't installed return `0`.
 */
void printInputs(const std::vector<bool>& bits)
{
    static const char* names[6] = {
        "IN101", "IN102",
        "IN401", "IN402", "IN403", "IN404"
    };
    for (size_t i = 0; i < bits.size() && i < 6; ++i) {
        std::cout << "  " << names[i] << " = " << (bits[i] ? "1 (ON) " : "0 (off)") << '\n';
    }
}

} // anonymous namespace

/**
 * @brief Entry point — read 6 discrete inputs via FC 02 and print their state.
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
    // FC 02 — Read Discrete Inputs (IN101..IN404)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[FC 02] Reading 6 discrete inputs starting at address 0...\n";
        auto inputs = master.readDiscreteInputs(0, 6);
        std::cout << "\nInput status:\n";
        printInputs(inputs);
        std::cout << "\n[FC 02] Read complete.\n";
    } catch (const std::exception& e) {
        std::cerr << "[FC 02] Error: " << e.what() << '\n';
        master.disconnect();
        return 1;
    }

    master.disconnect();
    return 0;
}
