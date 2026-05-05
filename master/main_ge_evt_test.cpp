/**
 * @file main_ge_evt_test.cpp
 * @brief Format-discovery test for the GE UR Event Recorder file (EVT.TXT).
 *
 * @details
 * Single-shot probe that pulls the **first 244-byte block** of the GE UR
 * Event Recorder ASCII file over Modbus TCP and prints it raw, so the
 * on-wire format can be inspected before writing a full parser.
 *
 * Sequence (UR Family Communications Guide v7.6x §2.3.1.6 / v8.7x §3.4.4):
 *   1. **FC 16** at `0x3100` — write the filename "evt.txt" (null-terminated)
 *      into the "Name of File to Read" register block (8 regs / 16 bytes,
 *      zero-padded so any prior filename is overwritten).
 *   2. **FC 04** at `0x3202` — read 1 reg → "Size of Currently-available
 *      Data Block" (bytes available, max 244).
 *   3. **FC 04** at `0x3203` — read ((size+1)/2) regs → "Block of Data
 *      from Requested File" (244 bytes max).
 *
 * Output is dumped both as a hex+ASCII table and a literal ASCII rendering
 * (with `\r` / `\n` markers visible) so the line-ending convention and
 * per-event field layout become obvious without guessing.
 *
 * @note **Unit ID = 254.** GE UR's default Modbus slave address is 254
 *       (L90 manual §5.3.5.8). With any other unit ID the relay silently
 *       closes the TCP session — confirmed both empirically and in the
 *       comment at `main_ge_discover.cpp:111-114`.
 */

#include "modbus_master.hpp"
#include "transport.hpp"

#include <cctype>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* HOST     = "192.168.0.21";
constexpr uint16_t    PORT     = Modbus::DEFAULT_PORT;
constexpr uint8_t     UNIT_ID  = 254;
constexpr int         TIMEOUT  = 5000;

constexpr uint16_t ADDR_NAME_OF_FILE = 0x3100;  ///< FC 16 target — filename
constexpr uint16_t ADDR_BLOCK_SIZE   = 0x3202;  ///< FC 04 — bytes available
constexpr uint16_t ADDR_DATA_BLOCK   = 0x3203;  ///< FC 04 — 122 regs of file data

/**
 * @brief Pack an ASCII filename into Modbus registers (high byte first).
 *
 * UR convention: each register holds 2 chars, the first char in the
 * high byte. The block is null-terminated; we pad up to @p regCount
 * registers with zeros so unused tail bytes stay clean.
 */
std::vector<uint16_t> packFilename(const std::string& name, std::size_t regCount)
{
    std::vector<uint16_t> regs(regCount, 0);
    for (std::size_t i = 0; i < name.size() && (i / 2) < regCount; ++i) {
        const uint8_t c = static_cast<uint8_t>(name[i]);
        if (i % 2 == 0) {
            regs[i / 2] = static_cast<uint16_t>(c) << 8;
        } else {
            regs[i / 2] |= c;
        }
    }
    return regs;
}

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

/** @brief Print `xxd`-style 16-byte rows, hex on the left, ASCII on the right. */
void hexDump(const std::vector<uint8_t>& data, std::size_t validBytes)
{
    const std::size_t n = std::min(data.size(), validBytes);
    for (std::size_t i = 0; i < n; i += 16) {
        std::cout << "  " << std::setw(4) << std::setfill('0') << std::hex << i << "  ";
        for (std::size_t j = 0; j < 16; ++j) {
            if (i + j < n)
                std::cout << std::setw(2) << std::setfill('0') << std::hex
                          << static_cast<int>(data[i + j]) << ' ';
            else
                std::cout << "   ";
            if (j == 7) std::cout << ' ';
        }
        std::cout << " |";
        for (std::size_t j = 0; j < 16 && (i + j) < n; ++j) {
            const uint8_t c = data[i + j];
            std::cout << (std::isprint(c) ? static_cast<char>(c) : '.');
        }
        std::cout << "|\n";
    }
    std::cout << std::dec << std::setfill(' ');
}

} // namespace

int main(int /*argc*/, char* /*argv*/[])
{
    auto logger = [](const std::string& msg) { std::cout << msg << '\n'; };

    std::cout << "[EVT-Test] Connecting to GE UR at " << HOST << ":" << PORT
              << "  unitId=" << static_cast<int>(UNIT_ID) << "\n";

    Modbus::TcpTransport transport(HOST, PORT, TIMEOUT, logger);
    Modbus::Master       master(transport, UNIT_ID, logger);

    if (!master.connect()) {
        std::cerr << "[EVT-Test] Cannot connect — check IP/network.\n";
        return 1;
    }

    try {
        // ── Step 1: FC 16 — write filename "evt.txt" into 0x3100 ──────────
        const std::string     filename = "evt.txt";
        // 16-byte payload (8 regs) — generously larger than 8-char filename
        // so any prior remnants in the register block are zeroed out.
        std::vector<uint16_t> nameRegs = packFilename(filename + '\0', 8);

        std::cout << "\n[Step 1] FC 16 → addr 0x" << std::hex << ADDR_NAME_OF_FILE
                  << std::dec << "  (\"" << filename << "\", "
                  << nameRegs.size() << " regs)\n";

        master.writeMultipleRegisters(ADDR_NAME_OF_FILE, nameRegs);

        // ── Step 2: FC 04 — read available size at 0x3202 ─────────────────
        std::cout << "\n[Step 2] FC 04 ← addr 0x" << std::hex << ADDR_BLOCK_SIZE
                  << std::dec << "  (block size, 1 reg)\n";

        auto sizeRegs = master.readInputRegisters(ADDR_BLOCK_SIZE, 1);
        const uint16_t availableBytes = sizeRegs.at(0);

        std::cout << "  → available bytes = " << availableBytes
                  << "  (max 244; <244 means EOF)\n";

        if (availableBytes == 0) {
            std::cout << "\n[EVT-Test] No data available — event recorder may be empty,\n"
                      << "          or the filename was not accepted.\n";
            master.disconnect();
            return 0;
        }

        // ── Step 3: FC 04 — read the data block at 0x3203 ─────────────────
        const uint16_t regsToRead = static_cast<uint16_t>((availableBytes + 1) / 2);
        std::cout << "\n[Step 3] FC 04 ← addr 0x" << std::hex << ADDR_DATA_BLOCK
                  << std::dec << "  (data block, " << regsToRead << " regs)\n";

        auto dataRegs  = master.readInputRegisters(ADDR_DATA_BLOCK, regsToRead);
        auto dataBytes = regsToBytes(dataRegs);

        std::cout << "\n──────── First " << availableBytes
                  << " bytes of EVT.TXT (hex+ASCII) ────────\n";
        hexDump(dataBytes, availableBytes);

        std::cout << "\n──────── ASCII rendering (\\r and \\n visible) ────────\n";
        for (std::size_t i = 0; i < availableBytes && i < dataBytes.size(); ++i) {
            const uint8_t c = dataBytes[i];
            if (c == '\r')      std::cout << "\\r";
            else if (c == '\n') std::cout << "\\n\n";
            else if (c == '\0') std::cout << "\\0";
            else if (std::isprint(c)) std::cout << static_cast<char>(c);
            else                 std::cout << '.';
        }
        std::cout << "\n──────── End of dump ────────\n";

    } catch (const std::exception& e) {
        std::cerr << "[EVT-Test] ERROR: " << e.what() << '\n';
        master.disconnect();
        return 1;
    }

    master.disconnect();
    return 0;
}
