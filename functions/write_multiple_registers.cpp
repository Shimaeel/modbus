/**
 * @file write_multiple_registers.cpp
 * @brief FC 16 — Preset Multiple Registers (block write of N 16-bit registers).
 *
 * @details
 * **Purpose:** write a contiguous block of 16-bit register values in a
 * single transaction. Far more efficient than looping `writeSingleRegister`
 * for large blocks; the slave sees them all applied together (atomic from
 * its perspective).
 *
 * On the SEL-735 (manual section E.8): max **100 registers per request**.
 * Writes targeting **settable parameters** require a password handshake
 * (write Access Level E password to addr 70–74 first). Writes to
 * **command/reset** registers (78–80) do not require the handshake.
 *
 * @dot
 * digraph fc16 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="writeMultipleRegisters(addr,values)", fillcolor="#e6f0ff"];
 *   bld  [label="buildWriteMultipleRegisters()\n→ PDU [10][addr][qty][bc][regs...]", fillcolor="#cfe2ff"];
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
 * @brief Write a block of 16-bit registers starting at @p startAddr.
 * @param startAddr 0-based starting register address.
 * @param values    16-bit vector — one element per register to write.
 * @return Always `true` on a non-exception response.
 * @throws std::runtime_error on transport failure or Modbus exception.
 */
bool Master::writeMultipleRegisters(uint16_t startAddr,
                                    const std::vector<uint16_t>& values)
{
    auto pdu = buildWriteMultipleRegisters(startAddr, values);
    transaction(pdu);
    return true;
}

} // namespace Modbus
