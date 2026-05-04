/**
 * @file scan.cpp
 * @brief Implementation of the generic block-scan helper.
 */

#include "scan.hpp"
#include "modbus_master.hpp"
#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace Modbus {

namespace {

// Default chunk size per FC. Slightly under spec ceilings for gateway
// headroom; SEL-735 caps FC 03/04 at 125 anyway.
constexpr uint16_t REG_CHUNK_DEFAULT  = 120;   // FC 03/04 — spec max 125
constexpr uint16_t BIT_CHUNK_DEFAULT  = 1900;  // FC 01/02 — spec max 2000

bool isBitFc(FC fc)  { return fc == FC::READ_COILS || fc == FC::READ_DISCRETE_INPUTS; }
bool isRegFc(FC fc)  { return fc == FC::READ_HOLDING_REGISTERS || fc == FC::READ_INPUT_REGISTERS; }

/**
 * @brief Read one chunk and append per-address items. On Exception 02 with
 *        chunkSize > 1, recursively bisect to isolate the bad address(es)
 *        without dropping the good ones.
 */
void scanChunk(Master&                m,
               FC                     fc,
               uint16_t               addr,
               uint16_t               qty,
               std::vector<ScanItem>& out,
               int&                   chunksOk,
               int&                   chunksFailed)
{
    try {
        if (isBitFc(fc)) {
            std::vector<bool> v = (fc == FC::READ_COILS)
                ? m.readCoils(addr, qty)
                : m.readDiscreteInputs(addr, qty);
            // Slave may pad-up to the next byte; trim to the requested qty.
            if (v.size() > qty) v.resize(qty);
            for (uint16_t i = 0; i < qty; ++i) {
                ScanItem it;
                it.address  = addr + i;
                it.ok       = true;
                it.valueBit = (i < v.size()) ? v[i] : false;
                out.push_back(std::move(it));
            }
        } else if (isRegFc(fc)) {
            std::vector<uint16_t> v = (fc == FC::READ_HOLDING_REGISTERS)
                ? m.readHoldingRegisters(addr, qty)
                : m.readInputRegisters(addr, qty);
            for (uint16_t i = 0; i < qty; ++i) {
                ScanItem it;
                it.address = addr + i;
                it.ok      = true;
                it.value16 = (i < v.size()) ? v[i] : 0;
                out.push_back(std::move(it));
            }
        } else {
            throw std::runtime_error("scanRange: unsupported FC for read scan");
        }
        ++chunksOk;
    }
    catch (const std::exception& e) {
        std::string msg = e.what();
        // Exception 02 = Illegal Data Address. Bisect to isolate the bad
        // address(es) instead of failing the whole chunk.
        bool addressErr = msg.find("Illegal Data Address") != std::string::npos;
        if (addressErr && qty > 1) {
            uint16_t half1 = qty / 2;
            uint16_t half2 = static_cast<uint16_t>(qty - half1);
            scanChunk(m, fc, addr,           half1, out, chunksOk, chunksFailed);
            scanChunk(m, fc, addr + half1,   half2, out, chunksOk, chunksFailed);
            return;
        }
        // Single-address failure or non-address error: mark all qty addresses
        // in this chunk as failed but keep the row count consistent.
        for (uint16_t i = 0; i < qty; ++i) {
            ScanItem it;
            it.address = addr + i;
            it.ok      = false;
            it.error   = msg;
            out.push_back(std::move(it));
        }
        ++chunksFailed;
    }
}

} // namespace

ScanResult scanRange(Master&  m,
                     FC       fc,
                     uint16_t startAddr,
                     uint16_t count,
                     uint16_t chunkSize)
{
    if (!isBitFc(fc) && !isRegFc(fc))
        throw std::runtime_error("scanRange: FC must be 01/02/03/04");
    if (count == 0)
        throw std::runtime_error("scanRange: count must be > 0");

    uint16_t maxChunk = isBitFc(fc) ? BIT_CHUNK_DEFAULT : REG_CHUNK_DEFAULT;
    uint16_t chunk    = (chunkSize == 0) ? maxChunk : std::min<uint16_t>(chunkSize, maxChunk);

    ScanResult r;
    r.fc        = static_cast<uint8_t>(fc);
    r.startAddr = startAddr;
    r.count     = count;
    r.items.reserve(count);

    uint32_t end = static_cast<uint32_t>(startAddr) + count;   // 17-bit safe
    uint32_t cur = startAddr;
    while (cur < end) {
        uint16_t qty = static_cast<uint16_t>(std::min<uint32_t>(chunk, end - cur));
        ++r.chunksTotal;
        scanChunk(m, fc, static_cast<uint16_t>(cur), qty, r.items, r.chunksOk, r.chunksFailed);
        cur += qty;
    }
    return r;
}

std::string scanResultToCsv(const ScanResult& r)
{
    std::ostringstream oss;
    oss << "address,ok,value_hex,value_dec,bit,error\n";
    for (const auto& it : r.items) {
        oss << it.address << ',' << (it.ok ? "1" : "0") << ',';
        if (it.ok) {
            if (r.fc == static_cast<uint8_t>(FC::READ_COILS) ||
                r.fc == static_cast<uint8_t>(FC::READ_DISCRETE_INPUTS)) {
                oss << ",,";                        // hex,dec empty for bit FCs
                oss << (it.valueBit ? "1" : "0");
            } else {
                oss << "0x" << std::hex << it.value16 << std::dec
                    << ',' << it.value16 << ',';
            }
        } else {
            oss << ",,,";
        }
        // Quote the error in case it contains commas.
        oss << ',' << '"';
        for (char c : it.error) {
            if (c == '"') oss << "\"\"";
            else oss << c;
        }
        oss << '"' << '\n';
    }
    return oss.str();
}

} // namespace Modbus
