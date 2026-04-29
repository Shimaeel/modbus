/**
 * @file read_coils.cpp
 * @brief FC 01 — Read Coils.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

std::vector<bool> Master::readCoils(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_COILS, startAddr, quantity);
    auto resp = transaction(pdu);
    auto coils = parseReadCoilsResponse(resp);
    coils.resize(quantity);  // trim padding bits from byte-packing
    return coils;
}

} // namespace Modbus
