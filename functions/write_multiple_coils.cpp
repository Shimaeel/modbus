/**
 * @file write_multiple_coils.cpp
 * @brief FC 15 — Write Multiple Coils.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

bool Master::writeMultipleCoils(uint16_t startAddr, const std::vector<bool>& values)
{
    auto pdu = buildWriteMultipleCoils(startAddr, values);
    transaction(pdu);
    return true;
}

} // namespace Modbus
