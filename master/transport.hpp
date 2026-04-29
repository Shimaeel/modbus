#pragma once
/**
 * @file transport.hpp
 * @brief Layer 0 transport abstraction for the Modbus client.
 *
 * The Transport interface is the byte-level boundary between Layer 0 (I/O)
 * and Layer 1 (Modbus protocol). Per CLAUDE.md, asio lives only in Layer 0;
 * the protocol layer never sees a socket, only this interface.
 */

#include <boost/asio.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Modbus {

/** @brief Logging callback type (shared by transport + protocol layers). */
using LogCb = std::function<void(const std::string&)>;

/**
 * @class Transport
 * @brief Abstract byte-level transport (Layer 0).
 *
 * Implementations own the actual I/O resource (TCP socket, serial port, ...)
 * and provide a synchronous send/recv pair the protocol layer can drive.
 */
class Transport {
public:
    virtual ~Transport() = default;

    /** @brief Open the underlying I/O resource. */
    virtual bool connect() = 0;

    /** @brief Close the underlying I/O resource. */
    virtual void disconnect() = 0;

    /** @brief Whether the transport currently holds an open resource. */
    virtual bool isConnected() const = 0;

    /** @brief Blocking send of an entire frame. Throws on I/O failure. */
    virtual void send(const std::vector<uint8_t>& frame) = 0;

    /** @brief Blocking receive of exactly @p numBytes. Throws on I/O failure. */
    virtual std::vector<uint8_t> recv(size_t numBytes) = 0;

    /** @brief Set or replace the logging callback. */
    void setLogCallback(LogCb cb) { logCb_ = std::move(cb); }

protected:
    LogCb logCb_;
    void log(const std::string& msg) const { if (logCb_) logCb_(msg); }
};

/**
 * @class TcpTransport
 * @brief Modbus TCP transport over `boost::asio::ip::tcp::socket`.
 */
class TcpTransport : public Transport {
public:
    explicit TcpTransport(std::string host      = "127.0.0.1",
                          uint16_t    port      = 502,
                          int         timeoutMs = 3000,
                          LogCb       logCb     = nullptr);

    ~TcpTransport() override;

    TcpTransport(const TcpTransport&) = delete;
    TcpTransport& operator=(const TcpTransport&) = delete;

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;
    void send(const std::vector<uint8_t>& frame) override;
    std::vector<uint8_t> recv(size_t numBytes) override;

private:
    std::string host_;
    uint16_t    port_;
    int         timeoutMs_;

    boost::asio::io_context                          ioc_;
    std::unique_ptr<boost::asio::ip::tcp::socket>    socket_;
};

} // namespace Modbus
