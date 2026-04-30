/**
 * @file read_write_multiple_registers.cpp
 * @brief FC 23 — Read/Write Multiple Registers (combined transaction).
 *
 * @details
 * **Purpose:** in a single Modbus transaction, write a block of registers
 * **and then** read a (possibly different) block back, returning the
 * read result. The slave applies the write **first** so the subsequent
 * read sees the post-write state. Saves a round-trip compared to issuing
 * separate FC 16 + FC 03 calls.
 *
 * @warning **Not supported on SEL-735** (manual Table E.2). Fall back to
 *          FC 16 followed by FC 03 (one extra round-trip).
 *
 * @dot
 * digraph fc23 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="readWriteMultipleRegisters(\nrAddr,rQty,wAddr,wRegs)", fillcolor="#e6f0ff"];
 *   bld  [label="buildReadWriteMultipleRegisters()", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()\n(slave: write first, then read)", fillcolor="#fff2cc"];
 *   prs  [label="parseReadWriteMultipleResponse()", fillcolor="#cfe2ff"];
 *   out  [label="return read-back vector<uint16_t>", fillcolor="#c6efce"];
 *   in -> bld -> tx -> prs -> out;
 * }
 * @enddot
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Combined write-then-read of two register ranges.
 * @param readAddr  0-based starting address of the read range.
 * @param readQty   Number of registers to read.
 * @param writeAddr 0-based starting address of the write range.
 * @param writeRegs Values to write (size = number of registers).
 * @return Vector of @p readQty 16-bit values from the read range
 *         (post-write state).
 * @throws std::runtime_error on transport failure or Modbus exception
 *         (Illegal Function on SEL-735).
 */
std::vector<uint16_t> Master::readWriteMultipleRegisters(
    uint16_t readAddr, uint16_t readQty,
    uint16_t writeAddr, const std::vector<uint16_t>& writeRegs)
{
    auto pdu  = buildReadWriteMultipleRegisters(readAddr, readQty, writeAddr, writeRegs);
    auto resp = transaction(pdu);
    return parseReadWriteMultipleResponse(resp);
}

} // namespace Modbus
