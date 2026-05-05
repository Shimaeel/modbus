/**
 * @file main_ge_evt_test.cpp
 * @brief Read-only diagnostic for the GE UR Modbus File Transfer area.
 *
 * @details
 * The first attempt at this test (FC 16 to 0x3100 with "evt.txt") caused
 * the relay to **drop the TCP connection** with no Modbus reply at all.
 * That's a transport-level rejection, not a Modbus exception, and it
 * usually means one of:
 *
 *   1. Setting Password is enabled — relay refuses unauthenticated writes
 *      to settings registers.
 *   2. The file-transfer addresses on this firmware (L90 v8.2x) differ
 *      from the v7.6x Communications Guide values (0x3100 / 0x3200 /
 *      0x3202 / 0x3203).
 *   3. FC 16 is restricted on the file-transfer block in this firmware.
 *
 * **This rewrite probes the area read-only first.** No writes are issued.
 * We attempt:
 *
 *   - **FC 03** at `0x3100` (1 reg) — current filename in the "Name of
 *     File to Read" register. If this returns Exception 02, the address
 *     space is different and we need a fresh memory-map lookup.
 *   - **FC 04** at `0x3202` (1 reg) — "Size of currently-available block".
 *     Always read-only; a successful read confirms the actuals area exists.
 *   - **FC 04** at `0x3200` (2 regs) — "Character position of current
 *     block within file" (UINT32, F003).
 *   - **FC 04** at `0x3203` (4 regs) — first 8 bytes of the data block,
 *     just to see whether *some* file (default state) is already loaded.
 *
 * Each probe is wrapped in its own try/catch so one failure doesn't kill
 * the rest. Output is hex+ASCII so we can see what the relay returns.
 */

#include "modbus_master.hpp"
#include "transport.hpp"

#include <cctype>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* HOST     = "192.168.0.21";
constexpr uint16_t    PORT     = Modbus::DEFAULT_PORT;
constexpr uint8_t     UNIT_ID  = 1;
constexpr int         TIMEOUT  = 5000;

constexpr uint16_t ADDR_NAME_OF_FILE  = 0x3100;
constexpr uint16_t ADDR_FILE_POSITION = 0x3200;
constexpr uint16_t ADDR_BLOCK_SIZE    = 0x3202;
constexpr uint16_t ADDR_DATA_BLOCK    = 0x3203;

/** @brief Big-endian unpack of a register vector into bytes. */
std::vector<uint8_t> regsToBytes(const std::vector<uint16_t>& regs)
{
    std::vector<uint8_t> out;
    out.reserve(regs.size() * 2);
    for (uint16_t r : regs) {
        out.push_back(static_cast<uint8_t>(r >> 8));
        out.push_back(static_cast<uint8_t>(r & 0xFF));
    }
    return out;
}

/** @brief Print 16 bytes per row, hex on the left, printable ASCII on the right. */
void hexDump(const std::vector<uint8_t>& bytes)
{
    for (std::size_t i = 0; i < bytes.size(); i += 16) {
        std::cout << "  " << std::setw(4) << std::setfill('0') << std::hex << i << "  ";
        for (std::size_t j = 0; j < 16; ++j) {
            if (i + j < bytes.size())
                std::cout << std::setw(2) << std::setfill('0') << std::hex
                          << static_cast<int>(bytes[i + j]) << ' ';
            else
                std::cout << "   ";
            if (j == 7) std::cout << ' ';
        }
        std::cout << " |";
        for (std::size_t j = 0; j < 16 && (i + j) < bytes.size(); ++j) {
            const uint8_t c = bytes[i + j];
            std::cout << (std::isprint(c) ? static_cast<char>(c) : '.');
        }
        std::cout << "|\n";
    }
    std::cout << std::dec << std::setfill(' ');
}

/** @brief Run one probe — labelled, error-isolated, hex-dumped. */
void probe(const std::string& label,
           const std::function<std::vector<uint16_t>()>& readFn)
{
    std::cout << "\n── " << label << " ──\n";
    try {
        auto regs  = readFn();
        auto bytes = regsToBytes(regs);
        std::cout << "  ✓ OK — " << regs.size() << " regs / "
                  << bytes.size() << " bytes\n";
        hexDump(bytes);
    } catch (const std::exception& e) {
        std::cout << "  ✗ FAIL — " << e.what() << '\n';
    }
}

} // namespace

int main(int /*argc*/, char* /*argv*/[])
{
    auto logger = [](const std::string& msg) { std::cout << msg << '\n'; };

    std::cout << "[EVT-Diag] Connecting to GE UR at " << HOST << ":" << PORT
              << "  unitId=" << static_cast<int>(UNIT_ID) << "\n";

    Modbus::TcpTransport transport(HOST, PORT, TIMEOUT, logger);
    Modbus::Master       master(transport, UNIT_ID, logger);

    if (!master.connect()) {
        std::cerr << "[EVT-Diag] Cannot connect — check IP/network.\n";
        return 1;
    }

    probe("FC 03 @ 0x3100 (Name of File to Read, 4 regs = 8 bytes)",
          [&]{ return master.readHoldingRegisters(ADDR_NAME_OF_FILE, 4); });

    probe("FC 04 @ 0x3202 (Size of currently-available data block, 1 reg)",
          [&]{ return master.readInputRegisters(ADDR_BLOCK_SIZE, 1); });

    probe("FC 04 @ 0x3200 (Character Position, 2 regs = UINT32)",
          [&]{ return master.readInputRegisters(ADDR_FILE_POSITION, 2); });

    probe("FC 04 @ 0x3203 (Data block, first 4 regs = 8 bytes)",
          [&]{ return master.readInputRegisters(ADDR_DATA_BLOCK, 4); });

    master.disconnect();
    std::cout << "\n[EVT-Diag] Done.\n";
    return 0;
}
