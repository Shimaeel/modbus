#pragma once
/**
 * @file scan.hpp
 * @brief Generic, vendor-agnostic Modbus block-scan helper (Layer 2 / 3 hybrid).
 *
 * @details
 * `scanRange()` is the manager's "scan address 1..N" function. Caller picks
 * a unit ID, function code, start address, and count; the helper handles:
 *   - Auto-chunking against per-FC limits (FC 03/04 → 125 regs, FC 01/02 → 2000 bits).
 *   - Per-chunk timeouts / exceptions: failure of one chunk does not abort
 *     the rest of the scan.
 *   - On Exception 02 (Illegal Data Address), the failed chunk is bisected
 *     so good addresses still come back populated and only the truly invalid
 *     ones are flagged.
 *
 * The result is a flat per-address table the UI can render directly, plus
 * per-chunk summary counters for diagnostics.
 *
 * Plain C++ — no asio.
 */

#include "../modbus_common.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace Modbus {

class Master;  // forward

/** @brief Result row for one address in a scan. */
struct ScanItem {
    uint16_t      address {0};
    bool          ok      {false};
    uint16_t      value16 {0};      ///< populated for FC 03/04
    bool          valueBit{false};  ///< populated for FC 01/02
    std::string   error;            ///< empty when ok; "Illegal Data Address", "timeout", etc.
};

/** @brief Aggregate result of a scanRange() call. */
struct ScanResult {
    uint8_t                 fc          {0};
    uint16_t                startAddr   {0};
    uint16_t                count       {0};
    std::vector<ScanItem>   items;
    int                     chunksTotal {0};
    int                     chunksOk    {0};
    int                     chunksFailed{0};
};

/**
 * @brief Read a contiguous address range using FC 01/02/03/04, auto-chunking.
 *
 * @param m         Open Modbus master (must be connected; will throw if not).
 * @param fc        One of READ_COILS / READ_DISCRETE_INPUTS / READ_HOLDING_REGISTERS / READ_INPUT_REGISTERS.
 * @param startAddr First address of the range (0-based on the wire).
 * @param count     Number of addresses to read.
 * @param chunkSize 0 = use the per-FC default (125 / 2000); otherwise this caps the per-request size.
 *
 * @return One `ScanItem` per requested address, in order. Items with `ok=false`
 *         carry the error reason in `error`.
 *
 * @throws std::runtime_error only on hard failures (invalid FC, master not
 *         connected). Per-chunk transport/protocol errors are captured into
 *         the items and do **not** propagate.
 *
 * @note Default chunk sizes intentionally sit slightly under spec maximums
 *       to leave headroom for vendor quirks (some gateways cap below 125).
 */
ScanResult scanRange(Master&  m,
                     FC       fc,
                     uint16_t startAddr,
                     uint16_t count,
                     uint16_t chunkSize = 0);

/** @brief Format a ScanResult as a CSV string (header + rows). For UI export. */
std::string scanResultToCsv(const ScanResult& r);

} // namespace Modbus
