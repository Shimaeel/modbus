/**
 * @file read_discrete_inputs.cpp
 * @brief FC 02 — Read Discrete Inputs (read-only 1-bit input contacts).
 *
 * @details
 * **Purpose:** ask the slave for the current state (1 or 0) of `quantity`
 * discrete inputs starting at `startAddr`. Discrete inputs are the
 * **read-only** 1-bit area — typically wired to external signals coming
 * **into** the relay (breaker auxiliary contacts, status contacts from
 * other equipment, manual override switches).
 *
 * On the SEL-735, the first 6 inputs (Table E.8) are
 * `IN101, IN102, IN401..IN404`. Devices that aren't installed return 0.
 *
 * @dot
 * digraph fc02 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="readDiscreteInputs(addr,qty)", fillcolor="#e6f0ff"];
 *   bld  [label="buildReadRequest(FC=02)\n→ PDU", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()", fillcolor="#fff2cc"];
 *   prs  [label="parseReadCoilsResponse()\n(same bit-packing as FC 01)", fillcolor="#cfe2ff"];
 *   trm  [label="resize(qty)", fillcolor="#fff2cc"];
 *   out  [label="return vector<bool>", fillcolor="#c6efce"];
 *   in -> bld -> tx -> prs -> trm -> out;
 * }
 * @enddot
 *
 * @note FC 02 wire format is identical to FC 01 except for the function
 *       code byte, so the response parser is shared.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Read @p quantity discrete inputs starting at @p startAddr.
 * @param startAddr 0-based starting input address.
 * @param quantity  Number of input contacts to read (1..2000).
 * @return Vector of length @p quantity holding each input's bool state.
 * @throws std::runtime_error on transport or Modbus-exception failure.
 */
std::vector<bool> Master::readDiscreteInputs(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_DISCRETE_INPUTS, startAddr, quantity);
    auto resp = transaction(pdu);
    auto coils = parseReadCoilsResponse(resp);
    coils.resize(quantity);
    return coils;
}

} // namespace Modbus
