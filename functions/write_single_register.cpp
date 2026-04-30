/**
 * @file write_single_register.cpp
 * @brief FC 06 — Preset Single Register (write one 16-bit holding register).
 *
 * @details
 * **Purpose:** write a single 16-bit value to a holding register. Used
 * for writing setpoints, control commands, and configuration words —
 * one register at a time.
 *
 * On the SEL-735 this is how command/reset registers are triggered:
 * - `addr=78` Reset Communication Counters (write `0x0001`).
 * - `addr=79` Reset Max/Min Values.
 * - `addr=80` Reset Peak Demand.
 * - `addr=76` Save Settings.
 *
 * **Settable parameters** (Meter ID, Time, User Map) require a password
 * handshake (manual section E.8–E.9) — without it the relay returns
 * Exception 04 (Device Error / Invalid Access Level). Reset commands at
 * 75–80 do **not** require the handshake (empirically verified).
 *
 * @dot
 * digraph fc06 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="writeSingleRegister(addr,value)", fillcolor="#e6f0ff"];
 *   bld  [label="buildWriteSingleRegister()\n→ PDU [06][addr][value]", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()", fillcolor="#fff2cc"];
 *   echo [label="(slave echoes addr+value)", fillcolor="#cfe2ff"];
 *   out  [label="return true", fillcolor="#c6efce"];
 *   in -> bld -> tx -> echo -> out;
 * }
 * @enddot
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Write @p value to the holding register at @p address.
 * @param address 0-based register address.
 * @param value   New 16-bit value to write.
 * @return Always `true` on a non-exception response.
 * @throws std::runtime_error on transport failure or Modbus exception
 *         (e.g. Exception 04 when access-level password is required).
 */
bool Master::writeSingleRegister(uint16_t address, uint16_t value)
{
    auto pdu = buildWriteSingleRegister(address, value);
    transaction(pdu);
    return true;
}

} // namespace Modbus
