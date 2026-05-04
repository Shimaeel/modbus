/**
 * @file main_ge_discover.cpp
 * @brief Read-only Modbus discovery probe for GE Multilin UR-series relays.
 *
 * @details
 * **Purpose:** without a Communications Guide PDF, empirically discover
 * what the GE UR at `192.168.0.21:502` exposes via Modbus TCP. The probe
 * issues only **READ** function codes — it never writes — so it is safe
 * to run against an in-service relay.
 *
 * ### What gets probed
 * - **FC 03** — Read Holding Registers, addresses 0..49 (typical UR
 *   identity area: Product Device Code, hardware/firmware version,
 *   serial, order code).
 * - **FC 04** — Read Input Registers at the same range; verifies the
 *   GE quirk that "UR treats FC 03 ≡ FC 04".
 * - **FC 01** — Read Coils 0..15 (virtual outputs, status bits).
 * - **FC 02** — Read Discrete Inputs 0..15 (contact inputs).
 *
 * ### What we hope to learn
 * - Connection works → FC 03 baseline confirmed for GE.
 * - The model string (`B30`, `T35`, `G60`, …) appears in the ASCII decode
 *   so we know which UR variant this is.
 * - FCs that return Exception 01 are unsupported on this firmware.
 * - Approximate location of identity registers, so we can target them
 *   precisely in a follow-up read.
 *
 * @warning **No writes.** The UR allows setpoint editing via Modbus —
 *          a wrong write can change relay behaviour. Discovery is
 *          read-only by design.
 *
 * ### Usage
 * @code
 *   $ ./modbus_ge_discover
 * @endcode
 * Defaults: host `192.168.0.21`, port `502`, unitId `1`. Edit the
 * constants at the top of `main()` if you need different values.
 */

#include "modbus_master.hpp"
#include "transport.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/**
 * @brief Decode 16-bit registers into a printable ASCII string.
 *
 * Each register holds two characters (high byte first per Modbus
 * convention). Stops at the first NUL byte; non-printable bytes show
 * as `.` so the operator can still see structural patterns.
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

/**
 * @brief Print a register block in a side-by-side decimal/hex/ASCII table.
 */
void dumpRegs(uint16_t startAddr, const std::vector<uint16_t>& regs)
{
    std::cout << "  Addr  | Dec    | Hex    | ASCII\n"
              << "  ------+--------+--------+------\n";
    for (size_t i = 0; i < regs.size(); ++i) {
        uint16_t v = regs[i];
        char hi = static_cast<char>((v >> 8) & 0xFF);
        char lo = static_cast<char>(v & 0xFF);
        char hiP = (hi >= 0x20 && hi < 0x7F) ? hi : '.';
        char loP = (lo >= 0x20 && lo < 0x7F) ? lo : '.';
        std::cout << "  " << std::setw(5) << (startAddr + i)
                  << " | " << std::setw(6) << v
                  << " | 0x" << std::hex << std::setw(4) << std::setfill('0')
                  << v << std::dec << std::setfill(' ')
                  << " | '" << hiP << loP << "'\n";
    }
}

/**
 * @brief Print a boolean vector as a comma-separated 0/1 list.
 */
void dumpBits(const std::vector<bool>& bits)
{
    std::cout << "  [";
    for (size_t i = 0; i < bits.size(); ++i)
        std::cout << (i ? "," : "") << (bits[i] ? "1" : "0");
    std::cout << "]\n";
}

} // anonymous namespace

/**
 * @brief Entry point — runs the read-only discovery probe.
 * @return `0` if connection succeeded; `1` on connect failure.
 */
int main(int argc, char* argv[])
{
    // GE UR L90 default Modbus Slave Address is 254 (manual §5.3.5.8).
    // The relay will silently close the TCP session if the request's
    // Unit ID does not match the configured slave address.
    const char* HOST    = (argc > 1) ? argv[1] : "192.168.0.21";
    uint16_t    PORT    = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) : Modbus::DEFAULT_PORT;
    uint8_t     UNIT_ID = (argc > 3) ? static_cast<uint8_t> (std::stoi(argv[3])) : 254;
    constexpr int TIMEOUT = 5000;                   // ms

    auto logger = [](const std::string& msg) { std::cout << msg << '\n'; };

    std::cout << "================================================\n"
              << "  GE Multilin UR — Modbus Discovery Probe\n"
              << "  Target: " << HOST << ":" << PORT
              << "  unitId=" << static_cast<int>(UNIT_ID) << "\n"
              << "  Mode:   READ-ONLY (no writes)\n"
              << "================================================\n";

    Modbus::TcpTransport transport(HOST, PORT, TIMEOUT, logger);
    Modbus::Master       master(transport, UNIT_ID, logger);

    if (!master.connect()) {
        std::cerr << "[GE-Discover] Cannot connect to " << HOST << ":" << PORT << "\n";
        return 1;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Probe 1: FC 03 — Read Holding Registers, addresses 0..49
    // ─────────────────────────────────────────────────────────────────────
    std::vector<uint16_t> fc03Regs;
    try {
        std::cout << "\n[Probe 1] FC 03 — Read Holding Registers (addr 0..49)\n";
        fc03Regs = master.readHoldingRegisters(0, 50);
        dumpRegs(0, fc03Regs);
        std::cout << "\n  Decoded ASCII (first 40 bytes): \""
                  << regsToAscii(fc03Regs) << "\"\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 03 failed: " << e.what() << '\n';
    }

    // ─────────────────────────────────────────────────────────────────────
    // Probe 2: FC 04 — Read Input Registers at the same range
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Probe 2] FC 04 — Read Input Registers (addr 0..49)\n";
        std::cout << "  (testing whether UR treats FC 04 == FC 03)\n";
        auto fc04Regs = master.readInputRegisters(0, 50);
        bool same = (fc04Regs == fc03Regs && !fc03Regs.empty());
        std::cout << "  Decoded ASCII: \"" << regsToAscii(fc04Regs) << "\"\n";
        std::cout << "  FC 04 == FC 03 ? " << (same ? "YES (UR quirk confirmed)"
                                                    : "NO (different data)") << "\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 04 failed: " << e.what() << '\n';
    }

    // ─────────────────────────────────────────────────────────────────────
    // Probe 3: FC 01 — Read Coils (0..15)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Probe 3] FC 01 — Read Coils (addr 0..15)\n";
        auto coils = master.readCoils(0, 16);
        dumpBits(coils);
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 01 failed: " << e.what() << '\n';
    }

    // ─────────────────────────────────────────────────────────────────────
    // Probe 4: FC 02 — Read Discrete Inputs (0..15)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Probe 4] FC 02 — Read Discrete Inputs (addr 0..15)\n";
        auto inputs = master.readDiscreteInputs(0, 16);
        dumpBits(inputs);
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 02 failed: " << e.what() << '\n';
    }

    // ─────────────────────────────────────────────────────────────────────
    // Probe 5: FC 03 at higher address — try common UR identity area
    // (UR families sometimes put product info around 0x0040 or 0x0080)
    // ─────────────────────────────────────────────────────────────────────
    try {
        std::cout << "\n[Probe 5] FC 03 — Higher-address scan (addr 64..113)\n";
        auto regs = master.readHoldingRegisters(64, 50);
        dumpRegs(64, regs);
        std::cout << "\n  Decoded ASCII: \"" << regsToAscii(regs) << "\"\n";
    } catch (const std::exception& e) {
        std::cerr << "  -> FC 03 (addr 64) failed: " << e.what() << '\n';
    }

    std::cout << "\n================================================\n"
              << "  Discovery complete. Look for ASCII strings\n"
              << "  containing the UR model name (B30, T35, G60, ...)\n"
              << "  and version markers in the dumps above.\n"
              << "================================================\n";

    master.disconnect();
    return 0;
}
