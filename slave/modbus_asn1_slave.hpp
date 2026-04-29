#pragma once
/**
 * @file modbus_asn1_slave.hpp
 * @brief Modbus Slave (Server) using Boost.Asio for TCP transport.
 *
 * Listens on TCP, receives standard Modbus PDUs, dispatches to
 * FC handlers (which use ASN.1 internally for validation), and
 * sends back standard Modbus responses.
 *
 * TCP supports multiple concurrent clients via async accept.
 *
 * Data model (each range 0-9999):
 *   - Coils            (bool, read/write)
 *   - Discrete inputs  (bool, read-only from master)
 *   - Holding registers (uint16, read/write)
 *   - Input registers   (uint16, read-only from master)
 */

#include "../modbus_common.hpp"
#include "../asn/modbus_asn1_tlv.hpp"
#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace Modbus {

/** @brief Size of each data-model array (coils, registers, etc.). */
static constexpr size_t DATA_MODEL_SIZE = 10000;

/**
 * @class TcpSession
 * @brief Represents a single TCP client connection (async read/write loop).
 */
class TcpSession : public std::enable_shared_from_this<TcpSession> {
public:
    using ProcessFn = std::function<ASN1::Bytes(const ASN1::Bytes&)>;
    using LogFn     = std::function<void(const std::string&)>;

    TcpSession(boost::asio::ip::tcp::socket socket,
               ProcessFn processFn, LogFn logFn);

    /** @brief Start reading frames from this client. */
    void start();

private:
    void readHeader();
    void readPdu(MBAPHeader mbap, size_t pduLen);
    void sendResponse(const ASN1::Bytes& frame);

    boost::asio::ip::tcp::socket socket_;
    ProcessFn processFn_;
    LogFn     logFn_;
    ASN1::Bytes headerBuf_;
};

/**
 * @class Slave
 * @brief Modbus slave (server) with TCP support via Boost.Asio.
 *
 * The server runs in a background thread after start() is called.
 * TCP accepts multiple concurrent clients.
 */
class Slave {
public:
    /** @brief Logging callback type. */
    using LogCb = std::function<void(const std::string&)>;

    /**
     * @brief Construct a Modbus slave.
     * @param unitId   Modbus unit/slave ID (default 1).
     * @param port     Port to listen on (default 502).
     * @param logCb    Optional logging callback.
     */
    explicit Slave(uint8_t  unitId    = 1,
                   uint16_t port      = DEFAULT_PORT,
                   LogCb    logCb     = nullptr);

    /** @brief Destructor. Stops the server if still running. */
    ~Slave();

    // Non-copyable
    Slave(const Slave&) = delete;
    Slave& operator=(const Slave&) = delete;

    /**
     * @brief Start the server in a background thread.
     * @return True on success, false if socket setup fails.
     */
    bool start();

    /** @brief Stop the server and close all sockets. */
    void stop();

    /** @brief Check whether the server is currently running. */
    bool isRunning() const { return running_; }

    /** @name Data model accessors
     *  @{
     */
    void     setCoil(uint16_t addr, bool val);
    bool     getCoil(uint16_t addr) const;
    void     setDiscreteInput(uint16_t addr, bool val);
    bool     getDiscreteInput(uint16_t addr) const;
    void     setHoldingRegister(uint16_t addr, uint16_t val);
    uint16_t getHoldingRegister(uint16_t addr) const;
    void     setInputRegister(uint16_t addr, uint16_t val);
    uint16_t getInputRegister(uint16_t addr) const;
    /** @} */

    /**
     * @brief Set or replace the logging callback.
     * @param cb New logging callback (may be nullptr to disable).
     */
    void setLogCallback(LogCb cb) { logCb_ = std::move(cb); }

private:
    void startTcpAccept();
    void setupHandlers();

    ASN1::Bytes processRequest(const ASN1::Bytes& pdu);

    void log(const std::string& msg) const { if (logCb_) logCb_(msg); }

    uint8_t  unitId_;
    uint16_t port_;
    LogCb    logCb_;

    boost::asio::io_context                                ioc_;
    std::unique_ptr<boost::asio::ip::tcp::acceptor>        tcpAcceptor_;

    std::atomic<bool> running_{false};
    std::thread       serverThread_;

    mutable std::mutex dataMutex_;  ///< Protects data model arrays across clients.

    std::array<bool,     DATA_MODEL_SIZE> coils_          {};
    std::array<bool,     DATA_MODEL_SIZE> discreteInputs_ {};
    std::array<uint16_t, DATA_MODEL_SIZE> holdingRegs_    {};
    std::array<uint16_t, DATA_MODEL_SIZE> inputRegs_      {};

    FcDispatcher dispatcher_;   ///< FC dispatcher (ASN.1 used inside handlers).
};

} // namespace Modbus
