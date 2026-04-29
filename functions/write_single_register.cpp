/**
 * @file write_single_register.cpp
 * @brief FC 06 — Write Single Register.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

bool Master::writeSingleRegister(uint16_t address, uint16_t value)
{
    auto pdu = buildWriteSingleRegister(address, value);
    transaction(pdu);
    return true;
}

} // namespace Modbus
