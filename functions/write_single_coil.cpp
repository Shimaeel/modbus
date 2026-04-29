/**
 * @file write_single_coil.cpp
 * @brief FC 05 — Write Single Coil.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

bool Master::writeSingleCoil(uint16_t address, bool value)
{
    auto pdu = buildWriteSingleCoil(address, value);
    transaction(pdu);
    return true;
}

} // namespace Modbus
