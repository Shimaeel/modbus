#pragma once
/**
 * @file transport.hpp
 * @brief Layer 0 transport abstraction (byte-level I/O for the Modbus client).
 *
 * @details
 * This header defines the **boundary between Layer 0 (I/O) and Layer 1
 * (Modbus protocol)**. The protocol layer above never touches a socket — it
 * only ever sees the abstract `Modbus::Transport` interface declared here,
 * which exposes a simple synchronous send/recv pair operating on raw byte
 * buffers.
 *
 * ### Purpose
 * - Hide all `boost::asio` types from the protocol layer.
 * - Make Layer 1 unit-testable by allowing a `MockTransport` to be plugged
 *   in without spinning up an `io_context` or opening a real socket.
 * - Allow the on-wire transport (TCP today, RS-485 tomorrow) to be swapped
 *   out without changing a single line of protocol code.
 *
 * ### Architecture position
 * @dot
 * digraph layers {
 *   rankdir=TB;
 *   node [shape=box, style="rounded,filled", fillcolor="#e6f0ff"];
 *   App     [label="Layer 4: Application\n(main_master.cpp)"];
 *   FC      [label="Layer 3: Per-FC dispatch\n(functions/*.cpp)"];
 *   Master  [label="Layer 2: Modbus::Master\n(modbus_master.cpp)"];
 *   Trans   [label="Layer 1: Transport interface\n(THIS FILE — transport.hpp)", fillcolor="#ffe6b3"];
 *   Tcp     [label="Layer 0: TcpTransport impl\n(transport.cpp — owns asio socket)", fillcolor="#ffd699"];
 *   Net     [label="TCP/IP network", shape=ellipse, fillcolor="#d9d9d9"];
 *   Relay   [label="SEL-735 Relay", shape=ellipse, fillcolor="#c6efce"];
 *   App -> FC -> Master -> Trans -> Tcp -> Net -> Relay;
 * }
 * @enddot
 *
 * ### Asio Boundary Rule
 * Per project rules, **`<boost/asio.hpp>` is included only here and in
 * `transport.cpp`** — no header above this layer may include it. Concrete
 * `Transport` implementations own the asio resources and expose only
 * byte-level primitives upward.
 *
 * @see Modbus::Master  Layer 1 protocol class that consumes a Transport.
 */

#include <boost/asio.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Modbus {

/**
 * @typedef LogCb
 * @brief Logging callback type used by both transport and protocol layers.
 *
 * @details
 * A `std::function` taking a single `const std::string&` and returning void.
 * The same signature is reused by `Modbus::Master` so a single user-supplied
 * lambda can capture log lines from both layers with consistent formatting.
 *
 * Typical usage:
 * @code
 *   auto logger = [](const std::string& msg) { std::cout << msg << "\n"; };
 *   Modbus::TcpTransport t("192.168.0.2", 502, 5000, logger);
 *   Modbus::Master       m(t, 1, logger);
 * @endcode
 */
using LogCb = std::function<void(const std::string&)>;

/**
 * @class Transport
 * @brief Abstract byte-level transport (Layer 0 boundary).
 *
 * @details
 * Concrete implementations own the actual I/O resource (a TCP socket, a
 * serial port, a pipe, a mock) and provide a synchronous `send`/`recv`
 * pair that the protocol layer drives directly.
 *
 * The interface is deliberately tiny — five virtual methods — because
 * everything Modbus-specific (PDUs, MBAP headers, function codes, exception
 * detection) belongs above this layer. A Transport pushes and pulls byte
 * arrays; nothing more.
 *
 * ### Lifecycle
 * @dot
 * digraph lifecycle {
 *   node [shape=box, style="rounded,filled", fillcolor="#e6f0ff"];
 *   ctor    [label="construct\n(args = host, port, ...)"];
 *   conn    [label="connect()", fillcolor="#fff2cc"];
 *   io      [label="send() / recv()", fillcolor="#c6efce"];
 *   disc    [label="disconnect()", fillcolor="#fff2cc"];
 *   dtor    [label="destruct"];
 *   ctor -> conn -> io -> disc -> dtor;
 *   io   -> io   [label="N transactions"];
 *   conn -> dtor [label="never used", style=dashed];
 * }
 * @enddot
 *
 * ### Error model
 * All I/O methods throw `std::runtime_error` on failure. Implementations
 * are expected to clean up the underlying resource (e.g. close the socket
 * with `disconnect()`) before throwing, so the next `connect()` starts
 * from a known state.
 *
 * @see TcpTransport  Concrete TCP/IP implementation used by `main_master.cpp`.
 */
class Transport {
public:
    /** @brief Virtual destructor — required for safe deletion through a base pointer. */
    virtual ~Transport() = default;

    /**
     * @brief Open the underlying I/O resource (TCP connect, serial port open, ...).
     * @return `true` on success, `false` on failure (no exception thrown for
     *         a failed `connect()` so callers can decide policy).
     */
    virtual bool connect() = 0;

    /**
     * @brief Close the underlying I/O resource.
     * @details
     * Idempotent — calling on an already-closed transport is a no-op.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Query whether the transport currently holds an open resource.
     * @return `true` if `connect()` succeeded and `disconnect()` has not yet been called.
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief Blocking send of an entire frame.
     * @param frame The complete byte sequence to push out the wire (MBAP + PDU
     *              for Modbus TCP, or slave-id + PDU + CRC for RTU).
     * @throws std::runtime_error on socket / I/O failure (after disconnect).
     */
    virtual void send(const std::vector<uint8_t>& frame) = 0;

    /**
     * @brief Blocking receive of exactly @p numBytes.
     * @param numBytes Exact number of bytes the caller expects.
     * @return Vector of length @p numBytes containing the received payload.
     * @throws std::runtime_error on timeout or I/O failure (after disconnect).
     *
     * @details
     * The protocol layer normally calls this twice per transaction: once for
     * the fixed-size header (e.g. 7 bytes of MBAP) to learn the body length,
     * and once for the remaining body bytes.
     */
    virtual std::vector<uint8_t> recv(size_t numBytes) = 0;

    /**
     * @brief Install or replace the logging callback.
     * @param cb New callback (may be `nullptr` to disable logging).
     */
    void setLogCallback(LogCb cb) { logCb_ = std::move(cb); }

protected:
    LogCb logCb_;  ///< Optional sink for human-readable log lines.

    /**
     * @brief Convenience helper — emit @p msg if a callback is installed.
     * @param msg Pre-formatted human-readable line, no trailing newline.
     */
    void log(const std::string& msg) const { if (logCb_) logCb_(msg); }
};

/**
 * @class TcpTransport
 * @brief Modbus TCP transport implemented over `boost::asio::ip::tcp::socket`.
 *
 * @details
 * Owns a private `boost::asio::io_context` and a `tcp::socket`. All I/O is
 * synchronous (blocking) — the asio event loop is **not** run; calls go
 * through asio's blocking helpers (`asio::write`, `asio::read`) which return
 * an error code on failure. Receive/send timeouts are enforced via the
 * socket-level `SO_RCVTIMEO` / `SO_SNDTIMEO` options set in `connect()`.
 *
 * ### Why one io_context per transport?
 * Multiple TcpTransport instances do not share an io_context, so they are
 * safe to use independently from different threads. There is no scheduler
 * to run; the io_context is only present because asio sockets require one.
 *
 * ### What this class does NOT do
 * - **No retry logic** — a failed I/O throws and disconnects the socket;
 *   the caller decides whether to reconnect.
 * - **No framing** — does not know what a Modbus PDU looks like.
 * - **No CRC** — TCP itself provides integrity for Modbus TCP.
 *
 * @note Default timeout is 3000 ms; SEL-735 typically replies in < 100 ms,
 *       so a hit on the timeout is almost always a network or unit-id
 *       configuration problem, not a busy relay.
 *
 * ### Typical use
 * @code
 *   Modbus::TcpTransport t("192.168.0.2", 502, 5000, logger);
 *   Modbus::Master       m(t, /*unitId=* / 1, logger);
 *   m.connect();
 *   auto regs = m.readHoldingRegisters(0, 20);
 *   m.disconnect();
 * @endcode
 */
class TcpTransport : public Transport {
public:
    /**
     * @brief Construct (does not connect — call `connect()` separately).
     * @param host      Host name or dotted-quad IP of the slave.
     * @param port      TCP port (502 by default for Modbus TCP).
     * @param timeoutMs Per-call send/recv timeout in milliseconds.
     * @param logCb     Optional logging sink.
     */
    explicit TcpTransport(std::string host      = "127.0.0.1",
                          uint16_t    port      = 502,
                          int         timeoutMs = 3000,
                          LogCb       logCb     = nullptr);

    /** @brief Closes the socket if still open. */
    ~TcpTransport() override;

    TcpTransport(const TcpTransport&) = delete;            ///< Non-copyable (asio resources).
    TcpTransport& operator=(const TcpTransport&) = delete; ///< Non-copyable.

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;
    void send(const std::vector<uint8_t>& frame) override;
    std::vector<uint8_t> recv(size_t numBytes) override;

private:
    std::string host_;       ///< Slave host or IP.
    uint16_t    port_;       ///< Slave TCP port.
    int         timeoutMs_;  ///< Send/recv timeout (milliseconds).

    boost::asio::io_context                          ioc_;     ///< Required by asio sockets even in blocking mode.
    std::unique_ptr<boost::asio::ip::tcp::socket>    socket_;  ///< Owned socket; null when disconnected.
};

} // namespace Modbus
