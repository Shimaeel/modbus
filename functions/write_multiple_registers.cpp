/**
 * @file write_multiple_registers.cpp
 * @brief FC 16 — Write Multiple Registers.
 */

#include "../master/modbus_master.hpp"

namespace Modbus {

bool Master::writeMultipleRegisters(uint16_t startAddr,
                                    const std::vector<uint16_t>& values)
{
    auto pdu = buildWriteMultipleRegisters(startAddr, values);
    transaction(pdu);
    return true;
}

} // namespace Modbus
