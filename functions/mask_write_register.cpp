/**
 * @file mask_write_register.cpp
 * @brief FC 22 — Mask Write Register.
 *
 * Verifies the slave's echo against the request per Golden Rule #4
 * ("every write response is checked").
 */

#include "../master/modbus_master.hpp"
#include <stdexcept>

namespace Modbus {

bool Master::maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask)
{
    auto pdu  = buildMaskWriteRegister(address, andMask, orMask);
    auto resp = transaction(pdu);
    auto echo = parseMaskWriteResponse(resp);
    if (echo.address != address || echo.andMask != andMask || echo.orMask != orMask)
        throw std::runtime_error("Modbus Master: FC 22 echo mismatch");
    return true;
}

} // namespace Modbus
