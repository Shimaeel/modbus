/**
 * @file read_coils.cpp
 * @brief FC 01 — Read Coils. Reads N 1-bit ON/OFF coil-area values.
 *
 * @details
 * **Purpose:** ask the slave for the current state (1 or 0) of `quantity`
 * coils starting at `startAddr`. Coils are the read/write 1-bit memory area
 * — on the SEL-735 they map to physical output contacts (`OUT101..OUT404`)
 * and Remote Bits (`RB01..RB16`) per Table E.15 of the manual.
 *
 * @dot
 * digraph fc01 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="readCoils(addr, qty)", fillcolor="#e6f0ff"];
 *   bld  [label="buildReadRequest(FC=01)\n→ PDU", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()\n(MBAP+send/recv+exception check)", fillcolor="#fff2cc"];
 *   prs  [label="parseReadCoilsResponse()\n→ vector<bool>", fillcolor="#cfe2ff"];
 *   trm  [label="resize(qty)\n(trim padding bits)", fillcolor="#fff2cc"];
 *   out  [label="return vector<bool>", fillcolor="#c6efce"];
 *   in -> bld -> tx -> prs -> trm -> out;
 * }
 * @enddot
 *
 * @note On-wire bytes are bit-packed LSB-first within each byte; a read of
 *       23 bits comes back as 3 bytes (24 bits) and the trailing padding
 *       bit is dropped by the `coils.resize(quantity)` call below.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Read @p quantity contiguous coils starting at @p startAddr.
 * @param startAddr 0-based starting coil address.
 * @param quantity  Number of coils to read (1..2000 per Modbus spec).
 * @return Vector of length @p quantity holding each coil's bool state.
 * @throws std::runtime_error if the transport fails or the slave returns
 *         a Modbus exception (e.g. Illegal Data Address).
 */
std::vector<bool> Master::readCoils(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_COILS, startAddr, quantity);
    auto resp = transaction(pdu);
    auto coils = parseReadCoilsResponse(resp);
    coils.resize(quantity);  // trim padding bits from byte-packing
    return coils;
}

} // namespace Modbus
