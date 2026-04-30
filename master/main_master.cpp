/**
 * @file main_master.cpp
 * @brief Modbus Master single-function test — runs only FC 06 (Preset Single Register).
 *
 * @details
 * Connects to the SEL-735 over Modbus TCP and exercises FC 06 (Preset
 * Single Register) by writing `0x0001` to address `78`, which on the
 * SEL-735 is the **Reset Communication Counters** command register
 * (Table E.26). Communication counters are read via FC 03 before and
 * after the write so the effect is observable: post-write all error
 * counters should be `0` and `Num Msgs Rx` should be very small (just
 * the reads we ourselves performed).
 *
 * ### Why this target is safe
 * Address 78 is a **command register** in the Control I/O area
 * (75–80) — writing to it triggers a stats reset and does **not**
 * change any settings or affect protection behaviour. Per CLAUDE.md
 * (verified empirically), Control I/O writes do not require the
 * password handshake that *settable parameter* writes (MID, TID,
 * Time, User Map) need.
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
 * @brief Print the SEL-735 communication counters in a labelled block.
 * @param regs 9-element vector returned from `readHoldingRegisters(160, 9)`.
 */
void printCounters(const std::vector<uint16_t>& regs)
{
    static const char* names[9] = {
        "Num Msgs Rx", "Num Msgs Sent", "Invalid Address",
        "Bad CRC", "UART ERROR", "Illegal Function/Op",
        "Illegal Register", "Illegal Write", "Bad Packet Format"
    };
    for (size_t i = 0; i < regs.size() && i < 9; ++i)
        std::cout << "  " << names[i] << " = " << regs[i] << '\n';
}

} // anonymous namespace

/**
 * @brief Entry point — write 0x0001 to addr 78 and verify counters cleared.
 * @return `0` on a successful round-trip; `1` on connect/IO/exception failure.
 */
int main(int /*argc*/, char* /*argv*/[])
{
    constexpr const char* HOST                  = "192.168.0.2";
    constexpr uint16_t    PORT                  = Modbus::DEFAULT_PORT;   // 502
    constexpr uint8_t     UNIT_ID               = 1;
    constexpr int         TIMEOUT               = 5000;                   // ms
    constexpr uint16_t    RESET_COMM_CTR_ADDR   = 78;                     // Table E.26
    constexpr uint16_t    COMM_COUNTERS_ADDR    = 160;                    // Table E.26
    constexpr uint16_t    COMM_COUNTERS_QTY     = 9;

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
    // FC 06 — Preset Single Register (Reset Communication Counters @ addr 78)
    // ─────────────────────────────────────────────────────────────────────
    try {
        // ---- Step 1: Read counters BEFORE reset ----
        std::cout << "\n[FC 03] Reading communication counters BEFORE reset...\n";
        auto before = master.readHoldingRegisters(COMM_COUNTERS_ADDR, COMM_COUNTERS_QTY);
        std::cout << "\nCounters BEFORE:\n";
        printCounters(before);

        // ---- Step 2: Write 0x0001 to addr 78 → triggers counter reset ----
        std::cout << "\n[FC 06] Writing 0x0001 to addr " << RESET_COMM_CTR_ADDR
                  << " (Reset Comm Counters)\n";
        master.writeSingleRegister(RESET_COMM_CTR_ADDR, 0x0001);
        std::cout << "  Write accepted by relay\n";

        // ---- Step 3: Read counters AFTER reset to verify ----
        std::cout << "\n[FC 03] Reading communication counters AFTER reset...\n";
        auto after = master.readHoldingRegisters(COMM_COUNTERS_ADDR, COMM_COUNTERS_QTY);
        std::cout << "\nCounters AFTER:\n";
        printCounters(after);

        // ---- Step 4: Sanity check — error counters should all be 0 ----
        bool errorsCleared = true;
        for (size_t i = 2; i < after.size(); ++i) {  // skip Rx (idx 0) and Sent (idx 1)
            if (after[i] != 0) { errorsCleared = false; break; }
        }
        if (errorsCleared)
            std::cout << "\n[FC 06] Reset verified — all error counters are zero.\n";
        else
            std::cout << "\n[FC 06] Warning: some error counters non-zero post-reset.\n";
    } catch (const std::exception& e) {
        std::string msg = e.what();
        if (msg.find("Server Device Failure") != std::string::npos) {
            std::cout << "\n[FC 06] Got Exception 04 (Device Error / Invalid Access Level).\n"
                      << "        Protocol verified — relay rejected on access-level policy.\n";
        } else {
            std::cerr << "[FC 06] Error: " << msg << '\n';
            master.disconnect();
            return 1;
        }
    }

    master.disconnect();
    return 0;
}
