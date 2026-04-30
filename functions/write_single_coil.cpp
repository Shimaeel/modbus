/**
 * @file write_single_coil.cpp
 * @brief FC 05 — Force Single Coil (write a single 1-bit coil ON/OFF).
 *
 * @details
 * **Purpose:** force a single coil to ON or OFF. On the wire the value
 * is encoded as `0xFF00` (set) or `0x0000` (clear) — quirk of the Modbus
 * spec, hidden from the caller by `buildWriteSingleCoil`.
 *
 * On the SEL-735 this is the canonical way to issue **control commands**
 * to the relay:
 * - Write to physical output coil addresses (`OUT101..OUT404`, indices 0–6)
 *   actuates real wiring — handle with care.
 * - Write to Remote Bit addresses (`RB01..RB16`, indices 7–22) flips
 *   internal SELOGIC bits without touching any physical contact — safe
 *   for testing and for soft handshakes between relays.
 * - Write to Pulse RB addresses (indices 23–38) triggers a momentary
 *   pulse that auto-clears.
 *
 * @dot
 * digraph fc05 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="writeSingleCoil(addr,value)", fillcolor="#e6f0ff"];
 *   bld  [label="buildWriteSingleCoil()\nencodes value as 0xFF00/0x0000", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()", fillcolor="#fff2cc"];
 *   echo [label="(slave echoes request — accepted on no-exception)", fillcolor="#cfe2ff"];
 *   out  [label="return true", fillcolor="#c6efce"];
 *   in -> bld -> tx -> echo -> out;
 * }
 * @enddot
 *
 * @note Modbus spec mandates the slave echo the request bytes on success.
 *       This implementation relies on `transaction()` to throw on any
 *       exception response; absence of an exception is treated as success.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Force a single coil at @p address to @p value.
 * @param address 0-based coil address (e.g. 7 = `RB01` on SEL-735).
 * @param value   `true` → coil set; `false` → coil cleared.
 * @return Always `true` on a non-exception response.
 * @throws std::runtime_error on transport failure or Modbus exception.
 */
bool Master::writeSingleCoil(uint16_t address, bool value)
{
    auto pdu = buildWriteSingleCoil(address, value);
    transaction(pdu);
    return true;
}

} // namespace Modbus
