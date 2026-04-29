/**
 * @file read_fifo_queue.cpp
 * @brief FC 24 — Read FIFO Queue.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

std::vector<uint16_t> Master::readFifoQueue(uint16_t pointerAddr)
{
    auto pdu  = buildReadFifoQueue(pointerAddr);
    auto resp = transaction(pdu);
    return parseReadFifoQueueResponse(resp);
}

} // namespace Modbus
