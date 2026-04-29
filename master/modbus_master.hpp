#pragma once
/**
 * @file modbus_master.hpp
 * @brief Modbus Master (Client) — Layer 1 protocol logic.
 *
 * Builds Modbus PDUs, wraps them in MBAP headers, and pushes raw bytes
 * through a Layer 0 @ref Transport. Per CLAUDE.md, this header MUST NOT
 * include `<boost/asio.hpp>` — asio is confined to Layer 0.
 *
 * All public methods are synchronous (blocking until the transport
 * returns the response or throws on timeout).
 */

#include "../modbus_common.hpp"
#include "transport.hpp"
#include <cstdint>
#include <string>

namespace Modbus {

/**
 * @class Master
 * @brief Synchronous Modbus master (client). Pure protocol layer.
 *
 * Holds a non-owning reference to a @ref Transport. Each call builds a
 * Modbus request PDU, frames it with the MBAP header, hands the bytes to
 * the transport, and decodes the reply. ASN.1 TLV round-trip validation
 * is performed on both request and response PDUs.
 */
class Master {
public:
    /**
     * @brief Construct a Modbus master bound to a transport.
     * @param transport Reference to a Layer 0 transport (must outlive the Master).
     * @param unitId    Modbus unit/slave ID (default 1).
     * @param logCb     Optional logging callback.
     */
    explicit Master(Transport& transport,
                    uint8_t    unitId = 1,
                    LogCb      logCb  = nullptr);

    ~Master() = default;

    // Non-copyable
    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;

    /** @brief Open the underlying transport. */
    bool connect()    { return transport_.connect(); }

    /** @brief Close the underlying transport. */
    void disconnect() { transport_.disconnect(); }

    /** @brief Whether the underlying transport is open. */
    bool isConnected() const { return transport_.isConnected(); }

    /** @brief Set or replace the logging callback. */
    void setLogCallback(LogCb cb) { logCb_ = std::move(cb); }

    /** @brief Change the Modbus unit/slave ID for subsequent requests. */
    void setUnitId(uint8_t id) { unitId_ = id; }

    /** @name Modbus function code methods
     *  @{
     */
    std::vector<bool>     readCoils(uint16_t startAddr, uint16_t quantity);
    std::vector<bool>     readDiscreteInputs(uint16_t startAddr, uint16_t quantity);
    std::vector<uint16_t> readHoldingRegisters(uint16_t startAddr, uint16_t quantity);
    std::vector<uint16_t> readInputRegisters(uint16_t startAddr, uint16_t quantity);
    bool writeSingleCoil(uint16_t address, bool value);
    bool writeSingleRegister(uint16_t address, uint16_t value);
    bool writeMultipleCoils(uint16_t startAddr, const std::vector<bool>& values);
    bool writeMultipleRegisters(uint16_t startAddr, const std::vector<uint16_t>& values);
    bool maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask);
    std::vector<uint16_t> readWriteMultipleRegisters(uint16_t readAddr,
                                                     uint16_t readQty,
                                                     uint16_t writeAddr,
                                                     const std::vector<uint16_t>& writeRegs);
    std::vector<uint16_t> readFifoQueue(uint16_t pointerAddr);
    /** @} */

private:
    Bytes transaction(const Bytes& requestPdu);

    void log(const std::string& msg) const { if (logCb_) logCb_(msg); }

    Transport& transport_;
    uint8_t    unitId_;
    LogCb      logCb_;
    uint16_t   transactionId_{0};
};

} // namespace Modbus
