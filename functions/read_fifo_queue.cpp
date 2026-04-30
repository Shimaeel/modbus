/**
 * @file read_fifo_queue.cpp
 * @brief FC 24 — Read FIFO Queue (drain a slave-side FIFO of 16-bit values).
 *
 * @details
 * **Purpose:** read a slave-managed first-in / first-out queue of 16-bit
 * values. The slave returns a count followed by up to **31** queue
 * entries; reading is non-destructive (does not pop). FC 24 was designed
 * for niche use cases like batch event records.
 *
 * @warning **Not supported on SEL-735** (manual Table E.2). Effectively
 *          unused on protection relays in general — most vendors expose
 *          event/log data via dedicated register ranges instead. Will
 *          return Exception 01 if attempted on SEL-735.
 *
 * @dot
 * digraph fc24 {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   in   [label="readFifoQueue(pointerAddr)", fillcolor="#e6f0ff"];
 *   bld  [label="buildReadFifoQueue()", fillcolor="#cfe2ff"];
 *   tx   [label="Master::transaction()", fillcolor="#fff2cc"];
 *   prs  [label="parseReadFifoQueueResponse()", fillcolor="#cfe2ff"];
 *   out  [label="return up to 31\n16-bit values", fillcolor="#c6efce"];
 *   in -> bld -> tx -> prs -> out;
 * }
 * @enddot
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

/**
 * @brief Read the FIFO queue at the given pointer register address.
 * @param pointerAddr Address of the FIFO pointer register.
 * @return Vector of up to 31 16-bit values from the FIFO.
 * @throws std::runtime_error on transport failure or Modbus exception
 *         (Illegal Function on SEL-735 and most other relays).
 */
std::vector<uint16_t> Master::readFifoQueue(uint16_t pointerAddr)
{
    auto pdu  = buildReadFifoQueue(pointerAddr);
    auto resp = transaction(pdu);
    return parseReadFifoQueueResponse(resp);
}

} // namespace Modbus
