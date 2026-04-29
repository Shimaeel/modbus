/**
 * @file read_discrete_inputs.cpp
 * @brief FC 02 — Read Discrete Inputs.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

std::vector<bool> Master::readDiscreteInputs(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_DISCRETE_INPUTS, startAddr, quantity);
    auto resp = transaction(pdu);
    auto coils = parseReadCoilsResponse(resp);
    coils.resize(quantity);
    return coils;
}

} // namespace Modbus
