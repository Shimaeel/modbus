/**
 * @file read_input_registers.cpp
 * @brief FC 04 — Read Input Registers.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

std::vector<uint16_t> Master::readInputRegisters(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_INPUT_REGISTERS, startAddr, quantity);
    auto resp = transaction(pdu);
    return parseReadRegistersResponse(resp);
}

} // namespace Modbus
