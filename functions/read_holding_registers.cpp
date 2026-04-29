/**
 * @file read_holding_registers.cpp
 * @brief FC 03 — Read Holding Registers.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

std::vector<uint16_t> Master::readHoldingRegisters(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_HOLDING_REGISTERS, startAddr, quantity);
    auto resp = transaction(pdu);
    return parseReadRegistersResponse(resp);
}

} // namespace Modbus
