#pragma once
/**
 * @file modbus_asn1_master.hpp
 * @brief Modbus Master (Client) using Boost.Asio for TCP transport.
 *
 * Connects to a Modbus slave, sends ASN.1-encoded requests, and decodes
 * ASN.1-encoded responses over TCP.
 *
 * All public methods are synchronous (blocking until response received or
 * timeout).
 */

#include "modbus_asn1_common.hpp"
#include "modbus_asn1_tlv.hpp"
#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <string>

namespace Modbus {

/** @brief Transport protocol selection. */
enum class Transport { TCP, UDP };

/**
 * @class Master
 * @brief Synchronous Modbus master (client) over TCP/UDP via Boost.Asio.
 *
 * Establishes a connection to a Modbus slave and provides methods for
 * every supported function code.  Each call blocks until a response is
 * received or the configured timeout expires.
 *
 * Wire format is always standard Modbus TCP/UDP (raw PDU).
 * ASN.1 TLV is used internally for encode/decode validation on both
 * request and response PDUs.
 */
class Master {
public:
    /** @brief Logging callback type. */
    using LogCb = std::function<void(const std::string&)>;

    /**
     * @brief Construct a Modbus master.
     * @param host      Hostname or IP address of the slave (default "127.0.0.1").
     * @param port      Port number (default 502).
     * @param unitId    Modbus unit/slave ID (default 1).
     * @param timeoutMs Send/receive timeout in milliseconds (default 3000).
     * @param logCb     Optional logging callback.
     */
    explicit Master(std::string host      = "127.0.0.1",
                    uint16_t    port      = DEFAULT_PORT,
                    uint8_t     unitId    = 1,
                    int         timeoutMs = 3000,
                    LogCb       logCb     = nullptr);

    /** @brief Destructor. Disconnects if still connected. */
    ~Master();

    // Non-copyable
    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;

    /**
     * @brief Open a connection to the slave.
     * @return True on success, false on failure.
     */
    bool connect();

    /** @brief Close the connection. */
    void disconnect();

    /** @brief Check whether the socket is currently connected/open. */
    bool isConnected() const;

    /**
     * @brief Set or replace the logging callback.
     * @param cb New logging callback (may be nullptr to disable logging).
     */
    void setLogCallback(LogCb cb) { logCb_ = std::move(cb); }
    void setTransport(Transport t) { transport_ = t; }
    void setUnitId(uint8_t id)     { unitId_ = id; }

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
    /** @} */

private:
    ASN1::Bytes transaction(const ASN1::Bytes& requestPdu);

    /** @brief Forward a message to the log callback (if set). */
    void log(const std::string& msg) const { if (logCb_) logCb_(msg); }

    std::string host_;
    uint16_t    port_;
    uint8_t     unitId_;
    int         timeoutMs_;
    LogCb       logCb_;
    uint16_t    transactionId_{0};
    Transport   transport_{Transport::TCP};

    boost::asio::io_context                              ioc_;
    std::unique_ptr<boost::asio::ip::tcp::socket>        tcpSocket_;
    std::unique_ptr<boost::asio::ip::udp::socket>        udpSocket_;
    boost::asio::ip::udp::endpoint                       udpEndpoint_;
};

} // namespace Modbus
