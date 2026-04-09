/**
 * @file main_asn1_master.cpp
 * @brief Modbus Master application for SEL-735 Power Quality Meter (TCP).
 *
 * Connects to SEL-735 relay at 192.168.0.2:502 and reads metering data
 * via Modbus TCP holding registers (FC 03).
 *
 * Usage:
 *   modbus_master          -- use TCP
 */

#include "modbus_asn1_master.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

/**
 * @brief Print a labelled list of register values to stdout.
 */
static void printRegs(const std::string& label,
                      const std::vector<uint16_t>& regs)
{
    std::cout << label;
    for (size_t i = 0; i < regs.size(); ++i)
        std::cout << " [" << i << "]=" << regs[i];
    std::cout << '\n';
}

/**
 * @brief Entry point for the master demo.
 * @return 0 on success, 1 on connection or protocol error.
 */
int main(int argc, char* argv[])
{
    auto logger = [](const std::string& msg) {
        std::cout << msg << '\n';
    };

    std::cout << "[Master] Connecting to SEL-735 at 192.168.0.2\n";

    Modbus::Master master(
        /*host=*/"192.168.0.2",
        /*port=*/Modbus::DEFAULT_PORT,
        /*unitId=*/1,
        /*timeoutMs=*/5000,
        logger);

    if (!master.connect()) {
        std::cerr << "[Master] Cannot connect to SEL-735 – check IP/network.\n";
        return 1;
    }

    try {
        // ================================================================
        //  SEL-735 Holding Register Map (FC 03)
        //  Addresses below are typical — verify with your SEL-735 manual.
        //  SEL-735 uses 0-based addressing in Modbus TCP.
        // ================================================================

        // ----------------------------------------------------------------
        std::cout << "\n=== Voltage Readings (Phase A, B, C) ===\n";
        auto vRegs = master.readHoldingRegisters(0, 6);
        // SEL-735: Registers 0-1 = Va, 2-3 = Vb, 4-5 = Vc (32-bit float, 2 regs each)
        printRegs("  Voltage regs (raw):", vRegs);

        // ----------------------------------------------------------------
        std::cout << "\n=== Current Readings (Phase A, B, C) ===\n";
        auto iRegs = master.readHoldingRegisters(6, 6);
        // SEL-735: Registers 6-7 = Ia, 8-9 = Ib, 10-11 = Ic
        printRegs("  Current regs (raw):", iRegs);

        // ----------------------------------------------------------------
        std::cout << "\n=== Power Readings ===\n";
        auto pRegs = master.readHoldingRegisters(12, 8);
        // SEL-735: Active power, reactive power, apparent power, power factor
        printRegs("  Power regs (raw):", pRegs);

        // ----------------------------------------------------------------
        std::cout << "\n=== Frequency ===\n";
        auto fRegs = master.readHoldingRegisters(20, 2);
        // SEL-735: Frequency (32-bit float across 2 registers)
        printRegs("  Frequency regs (raw):", fRegs);

        // ----------------------------------------------------------------
        std::cout << "\n=== Energy / Demand ===\n";
        auto eRegs = master.readHoldingRegisters(22, 4);
        printRegs("  Energy regs (raw):", eRegs);

    } catch (const std::exception& e) {
        std::cerr << "[Master] Error: " << e.what() << '\n';
        return 1;
    }

    master.disconnect();
    std::cout << "\n[Master] SEL-735 read complete.\n";
    return 0;
}
