/**
 * @file read_input_registers.cpp
 * @brief FC 04 — Read Input Registers (16-bit, read-only area).
 *
 * @details
 * **Purpose:** read `quantity` 16-bit registers from the slave's
 * **read-only** input register area starting at `startAddr`. Originally
 * intended (in the Modicon model) for read-only sensor data separate from
 * the read/write holding registers, but most modern relays flatten the
 * two areas.
 *
 * On the SEL-735, FC 04 is **functionally identical to FC 03** — both
 * map to the same Modbus Register Map (Table E.26). This was empirically
 * verified during the integration test (FC 04 returns the same FID
 * string as FC 03).
 *
 * @dot
 * digraph fc04 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="readInputRegisters(addr,qty)", fillcolor="#e6f0ff"];
 *   bld  [label="buildReadRequest(FC=04)", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()", fillcolor="#fff2cc"];
 *   prs  [label="parseReadRegistersResponse()\n(shared with FC 03)", fillcolor="#cfe2ff"];
 *   out  [label="return vector<uint16_t>", fillcolor="#c6efce"];
 *   in -> bld -> tx -> prs -> out;
 * }
 * @enddot
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Read @p quantity 16-bit input registers starting at @p startAddr.
 * @param startAddr 0-based starting register address.
 * @param quantity  Number of registers to read (1..125 on SEL-735).
 * @return Vector of @p quantity 16-bit register values.
 * @throws std::runtime_error on transport or Modbus-exception failure.
 *
 * @note On SEL-735 this returns the same data as `readHoldingRegisters`
 *       at the same address.
 */
std::vector<uint16_t> Master::readInputRegisters(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_INPUT_REGISTERS, startAddr, quantity);
    auto resp = transaction(pdu);
    return parseReadRegistersResponse(resp);
}

} // namespace Modbus
