/**
 * @file read_write_multiple_registers.cpp
 * @brief FC 23 — Read/Write Multiple Registers.
 *
 * Slave performs the write first, then services the read from the post-write
 * state, all in a single transaction.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

std::vector<uint16_t> Master::readWriteMultipleRegisters(
    uint16_t readAddr, uint16_t readQty,
    uint16_t writeAddr, const std::vector<uint16_t>& writeRegs)
{
    auto pdu  = buildReadWriteMultipleRegisters(readAddr, readQty, writeAddr, writeRegs);
    auto resp = transaction(pdu);
    return parseReadWriteMultipleResponse(resp);
}

} // namespace Modbus
