/**
 * @file read_holding_registers.cpp
 * @brief FC 03 — Read Holding Registers (16-bit, read/write area).
 *
 * @details
 * **Purpose:** the most-used Modbus read function. Pulls `quantity`
 * 16-bit registers starting at `startAddr` from the slave's holding
 * register area. Holding registers carry the bulk of process data:
 * measurements, setpoints, configuration, identification strings — all
 * encoded as 16-bit words on the wire (big-endian).
 *
 * On the SEL-735 (Table E.26) this is how you read everything from the
 * firmware identifier (`0..19`) and serial number (`20..39`) to the live
 * voltage/current/power measurements. Maximum **125 registers per
 * request** — caller must chunk longer ranges.
 *
 * @dot
 * digraph fc03 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="readHoldingRegisters(addr,qty)", fillcolor="#e6f0ff"];
 *   bld  [label="buildReadRequest(FC=03)\n→ PDU [03][addr_hi addr_lo][qty_hi qty_lo]", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()", fillcolor="#fff2cc"];
 *   prs  [label="parseReadRegistersResponse()\n→ vector<uint16_t>", fillcolor="#cfe2ff"];
 *   out  [label="return vector<uint16_t>", fillcolor="#c6efce"];
 *   in -> bld -> tx -> prs -> out;
 * }
 * @enddot
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Read @p quantity 16-bit holding registers starting at @p startAddr.
 * @param startAddr 0-based starting register address.
 * @param quantity  Number of registers to read (1..125 on SEL-735).
 * @return Vector of @p quantity 16-bit register values (big-endian on wire,
 *         decoded into native host order).
 * @throws std::runtime_error on transport or Modbus-exception failure.
 *
 * @note 32-bit values (`LONG`, `LONGy`) span two consecutive registers with
 *       the most-significant word in the lower address. The caller is
 *       responsible for combining and applying any scale factors — there is
 *       no built-in 32-bit helper at this layer.
 */
std::vector<uint16_t> Master::readHoldingRegisters(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_HOLDING_REGISTERS, startAddr, quantity);
    auto resp = transaction(pdu);
    return parseReadRegistersResponse(resp);
}

} // namespace Modbus
