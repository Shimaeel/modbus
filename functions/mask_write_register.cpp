/**
 * @file mask_write_register.cpp
 * @brief FC 22 — Mask Write Register (atomic bit-level register modification).
 *
 * @details
 * **Purpose:** modify only selected bits of a single 16-bit register
 * **atomically**, leaving the other bits untouched. The slave performs:
 * @code
 *   new_value = (current_value AND andMask) OR (orMask AND NOT andMask)
 * @endcode
 * which is conceptually equivalent to read-modify-write but happens in a
 * single transaction (no race window if multiple masters share the
 * register).
 *
 * Use cases — packed status/flag registers where only one field needs to
 * change while preserving the rest. Real-world Modbus deployments often
 * skip this in favour of FC 03 + FC 06/16, accepting the race window.
 *
 * @warning **Not supported on SEL-735** (manual Table E.2). Calling on
 *          the SEL-735 will return Exception 01.
 *
 * ### Echo verification
 * Per project Golden Rule #4 — "every write response is checked" — this
 * function decodes the slave's echo and verifies that all three fields
 * (`address`, `andMask`, `orMask`) match the request. A mismatch throws.
 *
 * @dot
 * digraph fc22 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in    [label="maskWriteRegister(addr,and,or)", fillcolor="#e6f0ff"];
 *   bld   [label="buildMaskWriteRegister()", fillcolor="#cfe2ff"];
 *   tx    [label="Master::transaction()", fillcolor="#fff2cc"];
 *   prs   [label="parseMaskWriteResponse()", fillcolor="#cfe2ff"];
 *   chk   [label="echo == request?", shape=diamond, fillcolor="#fff2cc"];
 *   thr   [label="throw runtime_error\n(echo mismatch)", fillcolor="#f4cccc"];
 *   out   [label="return true", fillcolor="#c6efce"];
 *   in -> bld -> tx -> prs -> chk;
 *   chk -> thr [label="no"];
 *   chk -> out [label="yes"];
 * }
 * @enddot
 */

#include "../master/modbus_master.hpp"
#include <stdexcept>

namespace Modbus {

/**
 * @brief Atomically modify selected bits of register at @p address.
 * @param address 0-based register address.
 * @param andMask Bits set to 1 here are kept from the existing value.
 * @param orMask  Bits set to 1 here are forced to 1 in the result.
 * @return `true` on a successful echo-verified round-trip.
 * @throws std::runtime_error on transport failure, Modbus exception, or
 *         echo-mismatch from the slave.
 */
bool Master::maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask)
{
    auto pdu  = buildMaskWriteRegister(address, andMask, orMask);
    auto resp = transaction(pdu);
    auto echo = parseMaskWriteResponse(resp);
    if (echo.address != address || echo.andMask != andMask || echo.orMask != orMask)
        throw std::runtime_error("Modbus Master: FC 22 echo mismatch");
    return true;
}

} // namespace Modbus
