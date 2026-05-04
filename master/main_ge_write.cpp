/**
 * @file main_ge_write.cpp
 * @brief Write-FC probe for GE Multilin UR (L90) — one FC at a time.
 *
 * @warning Writes can change relay behavior. By default every probe in
 *          this file is COMMENTED OUT. Uncomment exactly one at a time,
 *          set the TARGET_* constant for that probe to a known-safe
 *          address from the UR Family Communications Guide / EnerVista,
 *          rebuild, run, observe, and re-comment before moving on.
 *
 * Defaults: host 192.168.0.21, port 502, unitId 254 (L90 factory default).
 * Override on the command line:
 *     modbus_ge_write.exe <host> <port> <unitId>
 *
 * Empirically observed on this L90 (firmware 8.2x):
 *   - FC 03 / 04 supported
 *   - FC 01 / 02 -> Illegal Function (UR exposes bits via holding regs)
 *   - FC 05 / 06 / 15 / 16 / 22 / 23 -> UNTESTED (this file is for that)
 */

#include "modbus_master.hpp"
#include "transport.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// ─────────────────────────────────────────────────────────────────────────
// Edit these to match a SAFE target on your relay before uncommenting
// the corresponding probe. Values below are PLACEHOLDERS — do not trust.
// Look them up in the UR Family Communications Guide memory map or pick
// an unwired Virtual Input from EnerVista.
// ─────────────────────────────────────────────────────────────────────────

// FC 05 — Write Single Coil. UR maps Virtual Input "execute" operations
// here. Pick a Vi that is NOT referenced by any FlexLogic equation.
constexpr uint16_t TARGET_FC05_COIL_ADDR = 0x0000;   // TODO: real Virtual Input addr
constexpr bool     TARGET_FC05_COIL_VAL  = true;

// FC 06 — Write Single Register. Setpoint write — extreme caution.
// Pick a register you can read first, then write the same value back to
// it (idempotent round-trip).
constexpr uint16_t TARGET_FC06_REG_ADDR  = 0x0000;   // TODO: safe register
constexpr uint16_t TARGET_FC06_REG_VAL   = 0x0000;

// FC 15 — Write Multiple Coils.
constexpr uint16_t TARGET_FC15_COIL_ADDR = 0x0000;   // TODO
// Values populated below in the probe body.

// FC 16 — Write Multiple Registers.
constexpr uint16_t TARGET_FC16_REG_ADDR  = 0x0000;   // TODO
// Values populated below in the probe body.

// FC 22 — Mask Write Register (atomic bit set/clear).
constexpr uint16_t TARGET_FC22_REG_ADDR  = 0x0000;   // TODO
constexpr uint16_t TARGET_FC22_AND_MASK  = 0xFFFF;   // keep all bits = no-op
constexpr uint16_t TARGET_FC22_OR_MASK   = 0x0000;   // set no bits   = no-op

// FC 23 — Read/Write Multiple Registers (combined transaction).
constexpr uint16_t TARGET_FC23_READ_ADDR  = 0x0000;  // TODO
constexpr uint16_t TARGET_FC23_READ_QTY   = 1;
constexpr uint16_t TARGET_FC23_WRITE_ADDR = 0x0000;  // TODO

void hr() { std::cout << "------------------------------------------------\n"; }

} // namespace

int main(int argc, char* argv[])
{
    const char* HOST    = (argc > 1) ? argv[1] : "192.168.0.21";
    uint16_t    PORT    = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) : Modbus::DEFAULT_PORT;
    uint8_t     UNIT_ID = (argc > 3) ? static_cast<uint8_t> (std::stoi(argv[3])) : 254;
    constexpr int TIMEOUT = 5000;

    auto logger = [](const std::string& msg) { std::cout << msg << '\n'; };

    std::cout << "================================================\n"
              << "  GE Multilin UR — Modbus WRITE Probe\n"
              << "  Target: " << HOST << ":" << PORT
              << "  unitId=" << static_cast<int>(UNIT_ID) << "\n"
              << "  Mode:   WRITE (one probe at a time)\n"
              << "================================================\n";

    Modbus::TcpTransport transport(HOST, PORT, TIMEOUT, logger);
    Modbus::Master       master(transport, UNIT_ID, logger);

    if (!master.connect()) {
        std::cerr << "[GE-Write] Cannot connect to " << HOST << ":" << PORT << "\n";
        return 1;
    }

    // ─────────────────────────────────────────────────────────────────────
    // [Probe A] FC 05 — Write Single Coil
    // ─────────────────────────────────────────────────────────────────────
    /*
    try {
        std::cout << "\n[Probe A] FC 05 — Write Single Coil\n";
        std::cout << "  addr=0x" << std::hex << TARGET_FC05_COIL_ADDR << std::dec
                  << " val=" << (TARGET_FC05_COIL_VAL ? "ON" : "OFF") << "\n";
        bool ok = master.writeSingleCoil(TARGET_FC05_COIL_ADDR, TARGET_FC05_COIL_VAL);
        std::cout << "  -> " << (ok ? "ACK (echo matched)" : "echo mismatch") << "\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 05 failed: " << e.what() << '\n';
    }
    hr();
    */

    // ─────────────────────────────────────────────────────────────────────
    // [Probe B] FC 06 — Write Single Register
    // ─────────────────────────────────────────────────────────────────────
    /*
    try {
        std::cout << "\n[Probe B] FC 06 — Write Single Register\n";
        std::cout << "  addr=0x" << std::hex << TARGET_FC06_REG_ADDR
                  << " val=0x"   << TARGET_FC06_REG_VAL << std::dec << "\n";
        bool ok = master.writeSingleRegister(TARGET_FC06_REG_ADDR, TARGET_FC06_REG_VAL);
        std::cout << "  -> " << (ok ? "ACK (echo matched)" : "echo mismatch") << "\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 06 failed: " << e.what() << '\n';
    }
    hr();
    */

    // ─────────────────────────────────────────────────────────────────────
    // [Probe C] FC 15 — Write Multiple Coils
    // ─────────────────────────────────────────────────────────────────────
    /*
    try {
        std::cout << "\n[Probe C] FC 15 — Write Multiple Coils\n";
        std::vector<bool> coilVals = { false, false, false, false };  // TODO
        std::cout << "  startAddr=0x" << std::hex << TARGET_FC15_COIL_ADDR << std::dec
                  << " count=" << coilVals.size() << "\n";
        bool ok = master.writeMultipleCoils(TARGET_FC15_COIL_ADDR, coilVals);
        std::cout << "  -> " << (ok ? "ACK" : "no-ack") << "\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 15 failed: " << e.what() << '\n';
    }
    hr();
    */

    // ─────────────────────────────────────────────────────────────────────
    // [Probe D] FC 16 — Write Multiple Registers
    // ─────────────────────────────────────────────────────────────────────
    /*
    try {
        std::cout << "\n[Probe D] FC 16 — Write Multiple Registers\n";
        std::vector<uint16_t> regVals = { 0x0000, 0x0000 };           // TODO
        std::cout << "  startAddr=0x" << std::hex << TARGET_FC16_REG_ADDR << std::dec
                  << " count=" << regVals.size() << "\n";
        bool ok = master.writeMultipleRegisters(TARGET_FC16_REG_ADDR, regVals);
        std::cout << "  -> " << (ok ? "ACK" : "no-ack") << "\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 16 failed: " << e.what() << '\n';
    }
    hr();
    */

    // ─────────────────────────────────────────────────────────────────────
    // [Probe E] FC 22 — Mask Write Register
    // andMask=0xFFFF, orMask=0x0000 is a no-op (value unchanged).
    // Use that first to confirm the FC is even supported, before any
    // bit-changing call.
    // ─────────────────────────────────────────────────────────────────────
    /*
    try {
        std::cout << "\n[Probe E] FC 22 — Mask Write Register\n";
        std::cout << "  addr=0x" << std::hex << TARGET_FC22_REG_ADDR
                  << " AND=0x"   << TARGET_FC22_AND_MASK
                  << " OR=0x"    << TARGET_FC22_OR_MASK << std::dec << "\n";
        bool ok = master.maskWriteRegister(TARGET_FC22_REG_ADDR,
                                           TARGET_FC22_AND_MASK,
                                           TARGET_FC22_OR_MASK);
        std::cout << "  -> " << (ok ? "ACK" : "no-ack") << "\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 22 failed: " << e.what() << '\n';
    }
    hr();
    */

    // ─────────────────────────────────────────────────────────────────────
    // [Probe F] FC 23 — Read/Write Multiple Registers
    // ─────────────────────────────────────────────────────────────────────
    /*
    try {
        std::cout << "\n[Probe F] FC 23 — Read/Write Multiple Registers\n";
        std::vector<uint16_t> writeVals = { 0x0000 };                 // TODO
        std::cout << "  readAddr=0x"  << std::hex << TARGET_FC23_READ_ADDR
                  << " readQty=" << std::dec << TARGET_FC23_READ_QTY
                  << "  writeAddr=0x" << std::hex << TARGET_FC23_WRITE_ADDR
                  << " writeQty=" << std::dec << writeVals.size() << "\n";
        auto regs = master.readWriteMultipleRegisters(TARGET_FC23_READ_ADDR,
                                                      TARGET_FC23_READ_QTY,
                                                      TARGET_FC23_WRITE_ADDR,
                                                      writeVals);
        std::cout << "  -> read-back " << regs.size() << " regs:";
        for (auto v : regs) std::cout << " 0x" << std::hex << v << std::dec;
        std::cout << "\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 23 failed: " << e.what() << '\n';
    }
    hr();
    */

    std::cout << "\n[GE-Write] All probes commented out by default — "
                 "uncomment one and rebuild.\n";

    master.disconnect();
    return 0;
}
