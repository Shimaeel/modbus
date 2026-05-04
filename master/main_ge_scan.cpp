/**
 * @file main_ge_scan.cpp
 * @brief Wide-range read-only Modbus scan for GE Multilin UR / 8-Series.
 *
 * @details
 * Complements `main_ge_discover.cpp` (which probes a few small ranges).
 * This tool sweeps a configurable address range with `scanRange()` —
 * auto-chunking, partial-failure tolerant — and dumps the result to CSV
 * so the operator can spot identity strings, valid address bands, and
 * unsupported regions in one pass.
 *
 * **Read-only.** No writes. Safe to point at an in-service GE relay
 * (the GE UR allows setpoint writes, so we deliberately stay away from
 * any FC 06 / FC 16 here).
 *
 * @note Defaults assume a GE UR at `192.168.0.21:502` unitId=1. If the
 *       network exposes the relay through a Modbus TCP gateway, set
 *       UNIT_ID to the gateway-side address instead of 1.
 */

#include "modbus_master.hpp"
#include "transport.hpp"
#include "scan.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using Modbus::Master;
using Modbus::TcpTransport;
using Modbus::FC;

namespace {

// Defaults — overridable from argv (see main()). Editing these still works,
// but argv is the friction-free path on site where rebuilding is awkward.
constexpr const char* DEFAULT_HOST       = "192.168.0.21";
constexpr uint16_t    DEFAULT_PORT       = Modbus::DEFAULT_PORT;
constexpr uint8_t     DEFAULT_UNIT_ID    = 1;
constexpr int         DEFAULT_TIMEOUT    = 5000;
constexpr uint16_t    DEFAULT_SCAN_START = 0;
constexpr uint16_t    DEFAULT_SCAN_COUNT = 2000;

void printUsage(const char* argv0)
{
    std::cout
        << "Usage: " << argv0
        << " [host] [port] [unitId] [scanStart] [scanCount]\n"
        << "Defaults: host="  << DEFAULT_HOST
        << " port="            << DEFAULT_PORT
        << " unitId="          << static_cast<int>(DEFAULT_UNIT_ID)
        << " start="           << DEFAULT_SCAN_START
        << " count="           << DEFAULT_SCAN_COUNT << "\n"
        << "Examples:\n"
        << "  " << argv0 << "                              # use defaults\n"
        << "  " << argv0 << " 192.168.1.50                 # IP only\n"
        << "  " << argv0 << " 192.168.1.50 502 1           # IP + port + unitId\n"
        << "  " << argv0 << " 192.168.1.50 502 1 0 5000    # also widen scan to 5000 regs\n";
}

void printSummary(const Modbus::ScanResult& r, const char* label)
{
    int ok = 0, bad = 0;
    for (const auto& it : r.items) (it.ok ? ok : bad) += 1;
    std::cout << "  " << label
              << " rows="     << r.items.size()
              << " ok="       << ok
              << " bad="      << bad
              << " chunks="   << r.chunksTotal
              << " (ok="      << r.chunksOk
              << " failed="   << r.chunksFailed << ")\n";
}

/** @brief Best-effort ASCII decode of contiguous OK rows; helps the eye
 *         spot model strings like "B30", "T35", "8615" in the dump. */
void scanForAsciiStrings(const Modbus::ScanResult& r)
{
    std::string run;
    uint16_t    runStart = 0;
    auto flush = [&](uint16_t end) {
        if (run.size() >= 4) {
            std::cout << "    addr " << std::setw(5) << runStart
                      << "..." << std::setw(5) << end << "  \""
                      << run << "\"\n";
        }
        run.clear();
    };
    for (size_t i = 0; i < r.items.size(); ++i) {
        const auto& it = r.items[i];
        if (!it.ok) { flush(it.address); continue; }
        char hi = static_cast<char>((it.value16 >> 8) & 0xFF);
        char lo = static_cast<char>( it.value16       & 0xFF);
        auto printable = [](char c) { return c >= 0x20 && c < 0x7F; };
        if (printable(hi) && printable(lo)) {
            if (run.empty()) runStart = it.address;
            run.push_back(hi);
            run.push_back(lo);
        } else {
            flush(it.address);
        }
    }
    if (!r.items.empty()) flush(r.items.back().address);
}

} // namespace

int main(int argc, char* argv[])
{
    // -h / --help short-circuit.
    if (argc >= 2) {
        std::string a1 = argv[1];
        if (a1 == "-h" || a1 == "--help") { printUsage(argv[0]); return 0; }
    }

    // Parse positional args; missing slots fall back to defaults.
    std::string host    = (argc > 1) ? argv[1] : DEFAULT_HOST;
    uint16_t    port    = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) : DEFAULT_PORT;
    uint8_t     unitId  = (argc > 3) ? static_cast<uint8_t> (std::stoi(argv[3])) : DEFAULT_UNIT_ID;
    uint16_t    start   = (argc > 4) ? static_cast<uint16_t>(std::stoi(argv[4])) : DEFAULT_SCAN_START;
    uint16_t    count   = (argc > 5) ? static_cast<uint16_t>(std::stoi(argv[5])) : DEFAULT_SCAN_COUNT;

    std::cout << "================================================\n"
              << "  GE Multilin — Wide Modbus TCP scan (read-only)\n"
              << "  Target: " << host << ":" << port
              << "  unitId=" << static_cast<int>(unitId) << "\n"
              << "  Range:  " << start << ".."
              << (start + count - 1)
              << "  (" << count << " regs)\n"
              << "================================================\n";

    TcpTransport transport(host, port, DEFAULT_TIMEOUT, nullptr);
    Master       master(transport, unitId, nullptr);

    if (!master.connect()) {
        std::cerr << "[GE-Scan] Cannot connect to " << host << ":" << port
                  << " — check IP/network/port 502 reachability.\n";
        return 1;
    }
    std::cout << "[GE-Scan] Connected.\n";

    // ── FC 03 wide sweep ────────────────────────────────────────────────
    std::cout << "\n[FC 03] sweeping holding registers...\n";
    auto r03 = Modbus::scanRange(master, FC::READ_HOLDING_REGISTERS,
                                 start, count);
    printSummary(r03, "FC 03");

    {
        std::ofstream out("ge_scan_fc03.csv");
        out << Modbus::scanResultToCsv(r03);
        std::cout << "  -> ge_scan_fc03.csv\n";
    }

    std::cout << "\n  ASCII-printable runs (length >= 4) found in FC 03 dump:\n";
    scanForAsciiStrings(r03);

    // ── FC 02 small sweep — discrete inputs ─────────────────────────────
    std::cout << "\n[FC 02] discrete inputs 0..63 ...\n";
    try {
        auto r02 = Modbus::scanRange(master, FC::READ_DISCRETE_INPUTS, 0, 64);
        printSummary(r02, "FC 02");
        std::ofstream out("ge_scan_fc02.csv");
        out << Modbus::scanResultToCsv(r02);
        std::cout << "  -> ge_scan_fc02.csv\n";
    } catch (const std::exception& e) {
        std::cerr << "  FC 02 failed: " << e.what() << '\n';
    }

    // ── FC 01 small sweep — coils ───────────────────────────────────────
    std::cout << "\n[FC 01] coils 0..63 ...\n";
    try {
        auto r01 = Modbus::scanRange(master, FC::READ_COILS, 0, 64);
        printSummary(r01, "FC 01");
        std::ofstream out("ge_scan_fc01.csv");
        out << Modbus::scanResultToCsv(r01);
        std::cout << "  -> ge_scan_fc01.csv\n";
    } catch (const std::exception& e) {
        std::cerr << "  FC 01 failed: " << e.what() << '\n';
    }

    master.disconnect();
    std::cout << "\n[GE-Scan] done. Open the CSVs in Excel / a text editor;\n"
              << "the ASCII run list above usually contains the UR model code.\n";
    return 0;
}
