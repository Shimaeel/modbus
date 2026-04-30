/**
 * @file write_multiple_coils.cpp
 * @brief FC 15 — Force Multiple Coils (block write of N 1-bit coils).
 *
 * @details
 * **Purpose:** write a contiguous block of coil values in a single
 * request. The slave packs the bits LSB-first inside each byte (same
 * scheme as FC 01 responses) and applies them all atomically.
 *
 * @warning **Not supported on SEL-735** (manual Table E.2). Calling
 *          this on the SEL-735 will return Exception 01 (Illegal
 *          Function), surfaced as `std::runtime_error("Modbus
 *          exception: Illegal Function")`. The fallback is to loop
 *          `writeSingleCoil` (FC 05) per coil — slower but universal.
 *
 * @dot
 * digraph fc15 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="writeMultipleCoils(addr,values)", fillcolor="#e6f0ff"];
 *   bld  [label="buildWriteMultipleCoils()\n(bit-packs values into bytes)", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()", fillcolor="#fff2cc"];
 *   echo [label="(slave echoes addr+qty)", fillcolor="#cfe2ff"];
 *   out  [label="return true", fillcolor="#c6efce"];
 *   in -> bld -> tx -> echo -> out;
 * }
 * @enddot
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Write a block of coils starting at @p startAddr.
 * @param startAddr 0-based starting coil address.
 * @param values    Boolean vector — one element per coil to write.
 * @return Always `true` on a non-exception response.
 * @throws std::runtime_error on transport failure or Modbus exception
 *         (Illegal Function on SEL-735).
 */
bool Master::writeMultipleCoils(uint16_t startAddr, const std::vector<bool>& values)
{
    auto pdu = buildWriteMultipleCoils(startAddr, values);
    transaction(pdu);
    return true;
}

} // namespace Modbus
