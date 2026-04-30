/**
 * @file main_master.cpp
 * @brief Modbus Master single-function test — runs only FC 05 (Force Single Coil).
 *
 * @details
 * Connects to the SEL-735 over Modbus TCP and exercises FC 05 (Force
 * Single Coil) by toggling **RB01** ON → OFF, with a read-back via FC 01
 * after each write to verify the relay actually applied the change.
 *
 * ### Safety target — RB01 (coil address 7)
 * Per SEL-735 manual Table E.15, coil addresses are:
 * - `0..2`   → `OUT101..OUT103` — **physical output contacts** (do NOT touch)
 * - `3..6`   → `OUT401..OUT404` — **physical output contacts** (do NOT touch)
 * - `7..22`  → `RB01..RB16`     — **Remote Bits** (logical, internal to SELOGIC)
 * - `23..38` → Pulse RB variants
 *
 * RB01 (address `7`) is a **logical bit only** — it lives inside the
 * relay's SELOGIC engine and does **not actuate any physical wiring**.
 * Toggling it is safe even on an in-service relay.
 *
 * ### Why a read-back?
 * Per Modbus spec the slave echoes the request on success. The presence
 * of a non-exception reply already implies acceptance. We additionally
 * read coil 7 with FC 01 after each write to confirm the **observable
 * state** matches what we wrote — this catches relays that ack writes
 * but ignore them (rare, but a real failure mode).
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
 * @brief Entry point — toggle RB01 ON then OFF via FC 05, verifying via FC 01.
 * @return `0` on a successful round-trip; `1` on connect/IO/verify failure.
 */
int main(int /*argc*/, char* /*argv*/[])
{
    constexpr const char* HOST       = "192.168.0.2";
    constexpr uint16_t    PORT       = Modbus::DEFAULT_PORT;   // 502
    constexpr uint8_t     UNIT_ID    = 1;
    constexpr int         TIMEOUT    = 5000;                   // ms
    constexpr uint16_t    RB01_ADDR  = 7;                      // RB01 coil address per Table E.15

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
    // FC 05 — Force Single Coil (toggle RB01 ON then OFF, verify each step)
    // ─────────────────────────────────────────────────────────────────────
    try {
        // ---- Step 1: Set RB01 = ON ----
        std::cout << "\n[FC 05] Step 1: Force RB01 (coil 7) = ON\n";
        master.writeSingleCoil(RB01_ADDR, true);

        // ---- Step 2: Read-back to confirm RB01 is now 1 ----
        std::cout << "[FC 01] Read-back coil 7...\n";
        auto v1 = master.readCoils(RB01_ADDR, 1);
        std::cout << "  RB01 after SET   = " << (v1[0] ? "1 (ON)" : "0 (off)") << '\n';
        if (!v1[0]) throw std::runtime_error("RB01 did not assert after SET");

        // ---- Step 3: Clear RB01 = OFF ----
        std::cout << "\n[FC 05] Step 2: Force RB01 (coil 7) = OFF\n";
        master.writeSingleCoil(RB01_ADDR, false);

        // ---- Step 4: Read-back to confirm RB01 is now 0 ----
        std::cout << "[FC 01] Read-back coil 7...\n";
        auto v2 = master.readCoils(RB01_ADDR, 1);
        std::cout << "  RB01 after CLEAR = " << (v2[0] ? "1 (ON)" : "0 (off)") << '\n';
        if (v2[0]) throw std::runtime_error("RB01 did not clear after CLEAR");

        std::cout << "\n[FC 05] Toggle round-trip succeeded — RB01 SET then CLEAR verified.\n";
    } catch (const std::exception& e) {
        std::cerr << "[FC 05] Error: " << e.what() << '\n';
        master.disconnect();
        return 1;
    }

    master.disconnect();
    return 0;
}
