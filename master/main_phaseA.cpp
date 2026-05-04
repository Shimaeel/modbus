/**
 * @file main_phaseA.cpp
 * @brief Phase A demo — exercises the new pieces end-to-end against SEL-735.
 *
 * @details
 * Walks through the manager's three asks in order:
 *
 *   1. **Function snippets → shared memory**
 *      Calls `readDeviceIdentity`, `readContactInputs`, `readContactOutputs`,
 *      `readDeviceWordBitmap`, `readCommCounters`. Each writes into a
 *      `SharedMemory` instance the way the future state machine will. The
 *      tag dump at the end is what the UI/HMI will eventually render.
 *
 *   2. **Round-trip + counter verification**
 *      `verifyReadRoundTrip()` proves the read pipeline is sound (FID
 *      stable + counters advanced).
 *      `verifyWriteRoundTrip()` does a safe write to addr 79 (Reset
 *      Max/Min) and confirms the counter advanced — i.e., the write
 *      genuinely reached the relay.
 *
 *   3. **Generic block scan**
 *      `scanRange()` over addr 0..199 (FC 03) and the contact-input
 *      bit range (FC 02). Result rendered as CSV — exactly what the manager
 *      can take to site as a UI-ready dump.
 *
 * Defaults: host `192.168.0.2`, port 502, unitId 1. Edit the constants at
 * the top to retarget at runtime; full IP/port/unitId UI inputs come in
 * the front-end work.
 */

#include "modbus_master.hpp"
#include "transport.hpp"
#include "shared_memory.hpp"
#include "snippets.hpp"
#include "scan.hpp"
#include "verify.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <variant>

using Modbus::Master;
using Modbus::TcpTransport;
using Modbus::SharedMemory;
using Modbus::Point;
using Modbus::Quality;
using Modbus::qualityStr;
using Modbus::FC;

namespace {

constexpr const char* HOST    = "192.168.0.2";
constexpr uint16_t    PORT    = Modbus::DEFAULT_PORT;
constexpr uint8_t     UNIT_ID = 1;
constexpr int         TIMEOUT = 5000;

void printPoint(const std::string& key, const Point& p)
{
    std::cout << "  [" << qualityStr(p.quality) << "] " << key << " = ";
    std::visit([](auto&& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            std::cout << "(empty)";
        } else if constexpr (std::is_same_v<T, bool>) {
            std::cout << (v ? "true" : "false");
        } else if constexpr (std::is_same_v<T, uint16_t>) {
            std::cout << v;
        } else if constexpr (std::is_same_v<T, std::string>) {
            std::cout << '"' << v << '"';
        } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
            std::cout << "bits[" << v.size() << "]={";
            for (size_t i = 0; i < v.size(); ++i)
                std::cout << (i ? "," : "") << (v[i] ? "1" : "0");
            std::cout << '}';
        } else if constexpr (std::is_same_v<T, std::vector<uint16_t>>) {
            std::cout << "regs[" << v.size() << "]={";
            size_t shown = std::min<size_t>(8, v.size());
            for (size_t i = 0; i < shown; ++i)
                std::cout << (i ? "," : "") << "0x" << std::hex << v[i] << std::dec;
            if (v.size() > shown) std::cout << ",...";
            std::cout << '}';
        }
    }, p.value);
    if (p.quality != Quality::GOOD) std::cout << "   err=\"" << p.error << '"';
    std::cout << '\n';
}

} // namespace

int main()
{
    auto logger = [](const std::string& msg) { std::cout << msg << '\n'; };

    std::cout << "[PhaseA] Connecting to SEL-735 at " << HOST << ":" << PORT
              << " unitId=" << static_cast<int>(UNIT_ID) << "\n\n";

    TcpTransport transport(HOST, PORT, TIMEOUT, nullptr);   // quiet transport
    Master       master(transport, UNIT_ID, nullptr);
    SharedMemory sm;

    if (!master.connect()) {
        std::cerr << "[PhaseA] Cannot connect — check IP/network.\n";
        return 1;
    }

    // ─────────────────────────────────────────────────────────────────────
    // 1) Function snippets -> SharedMemory
    // ─────────────────────────────────────────────────────────────────────
    std::cout << "── 1) Snippets populate SharedMemory ──────────────\n";
    Modbus::readDeviceIdentity   (master, sm);
    Modbus::readContactInputs    (master, sm);
    Modbus::readContactOutputs   (master, sm);
    Modbus::readDeviceWordBitmap (master, sm);
    Modbus::readCommCounters     (master, sm);

    for (const auto& [k, p] : sm.snapshot()) printPoint(k, p);
    std::cout << '\n';

    // ─────────────────────────────────────────────────────────────────────
    // 2) Round-trip + counter verification
    // ─────────────────────────────────────────────────────────────────────
    std::cout << "── 2a) verifyReadRoundTrip ────────────────────────\n";
    {
        auto v = Modbus::verifyReadRoundTrip(master);
        std::cout << (v.ok ? "  PASS  " : "  FAIL  ") << v.note << '\n';
    }

    std::cout << "── 2b) verifyWriteRoundTrip (addr 79 = Reset Max/Min) ─\n";
    {
        auto v = Modbus::verifyWriteRoundTrip(master, /*writeAddr=*/79, /*value=*/0x0001);
        std::cout << (v.ok ? "  PASS  " : "  FAIL  ") << v.note << '\n';
    }
    std::cout << '\n';

    // ─────────────────────────────────────────────────────────────────────
    // 3) Generic block scan + CSV export
    // ─────────────────────────────────────────────────────────────────────
    std::cout << "── 3a) scanRange FC 03 addr 0..199 ─────────────────\n";
    {
        auto r = Modbus::scanRange(master, FC::READ_HOLDING_REGISTERS, 0, 200);
        std::cout << "  rows=" << r.items.size()
                  << " chunks=" << r.chunksTotal
                  << " ok=" << r.chunksOk
                  << " failed=" << r.chunksFailed << '\n';

        std::ofstream out("scan_fc03_0-199.csv");
        out << Modbus::scanResultToCsv(r);
        std::cout << "  -> scan_fc03_0-199.csv\n";
    }

    std::cout << "── 3b) scanRange FC 02 addr 0..15 ──────────────────\n";
    {
        auto r = Modbus::scanRange(master, FC::READ_DISCRETE_INPUTS, 0, 16);
        std::cout << "  rows=" << r.items.size()
                  << " chunks=" << r.chunksTotal
                  << " ok=" << r.chunksOk
                  << " failed=" << r.chunksFailed << '\n';
        for (const auto& it : r.items) {
            std::cout << "   addr " << std::setw(3) << it.address
                      << "  " << (it.ok ? (it.valueBit ? "1" : "0") : "ERR")
                      << "  " << it.error << '\n';
        }
    }

    master.disconnect();
    std::cout << "\n[PhaseA] done.\n";
    return 0;
}
