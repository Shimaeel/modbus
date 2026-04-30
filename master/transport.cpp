/**
 * @file transport.cpp
 * @brief `TcpTransport` implementation — concrete Layer 0 over `boost::asio`.
 *
 * @details
 * This translation unit contains the **only** code in the master that
 * touches `<boost/asio.hpp>`. Everything in Layers 1+ is plain C++ and
 * goes through the `Modbus::Transport` interface declared in
 * @ref transport.hpp.
 *
 * ### Per-call I/O flow
 * @dot
 * digraph io {
 *   rankdir=LR;
 *   node [shape=box, style="rounded,filled"];
 *   master   [label="Modbus::Master::transaction()", fillcolor="#e6f0ff"];
 *   send     [label="TcpTransport::send()", fillcolor="#fff2cc"];
 *   write    [label="asio::write()\n(blocking)", fillcolor="#ffe6b3"];
 *   sock     [label="tcp::socket", fillcolor="#ffd699"];
 *   net      [label="network", shape=ellipse, fillcolor="#d9d9d9"];
 *   master -> send -> write -> sock -> net;
 *   net -> sock -> read -> recv -> master [label="reply"];
 *   recv     [label="TcpTransport::recv()", fillcolor="#fff2cc"];
 *   read     [label="asio::read()\n(blocking)", fillcolor="#ffe6b3"];
 * }
 * @enddot
 *
 * ### Error policy
 * Any I/O error closes the socket and rethrows as `std::runtime_error` with
 * a descriptive message. There is **no automatic retry** at this layer —
 * retry policy belongs in higher layers.
 */

#include "transport.hpp"
#include <stdexcept>

/** @brief Short alias for `boost::asio` (file-local). */
namespace asio = boost::asio;
/** @brief Short alias for `boost::asio::ip::tcp` (file-local). */
using tcp = asio::ip::tcp;

namespace Modbus {

/**
 * @brief Constructor — stores configuration; does not open any I/O.
 * @param host      Host name or dotted-quad IPv4 of the slave.
 * @param port      Modbus TCP port (typically 502).
 * @param timeoutMs Per-call send/recv timeout in milliseconds.
 * @param logCb     Optional logging sink (forwarded to base class).
 */
TcpTransport::TcpTransport(std::string host, uint16_t port,
                           int timeoutMs, LogCb logCb)
    : host_(std::move(host)), port_(port), timeoutMs_(timeoutMs)
{
    logCb_ = std::move(logCb);
}

/**
 * @brief Destructor — ensures the socket is closed.
 */
TcpTransport::~TcpTransport()
{
    disconnect();
}

/**
 * @brief Resolve the host, open a TCP connection, and configure timeouts.
 *
 * @details
 * 1. If the socket is already open, return `true` (idempotent).
 * 2. DNS-resolve `host_:port_`.
 * 3. Allocate a fresh `tcp::socket` and `asio::connect` to the first reachable endpoint.
 * 4. Apply `SO_RCVTIMEO` / `SO_SNDTIMEO` socket options so subsequent blocking
 *    `read`/`write` honour `timeoutMs_`.
 *
 * @return `true` if the socket is open and ready, `false` on resolve or
 *         connect failure (with a log line emitted via the callback).
 */
bool TcpTransport::connect()
{
    if (socket_ && socket_->is_open()) return true;

    boost::system::error_code ec;
    tcp::resolver resolver(ioc_);
    auto results = resolver.resolve(host_, std::to_string(port_), ec);
    if (ec) {
        log("[TcpTransport] resolve failed: " + ec.message());
        return false;
    }

    socket_ = std::make_unique<tcp::socket>(ioc_);
    asio::connect(*socket_, results, ec);
    if (ec) {
        log("[TcpTransport] connect() failed to " + host_ + ":" +
            std::to_string(port_) + " - " + ec.message());
        socket_.reset();
        return false;
    }

#ifdef _WIN32
    DWORD tvMs = static_cast<DWORD>(timeoutMs_);
    setsockopt(socket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char*>(&tvMs), sizeof(tvMs));
    setsockopt(socket_->native_handle(), SOL_SOCKET, SO_SNDTIMEO,
               reinterpret_cast<const char*>(&tvMs), sizeof(tvMs));
#else
    struct timeval tv;
    tv.tv_sec  = timeoutMs_ / 1000;
    tv.tv_usec = (timeoutMs_ % 1000) * 1000;
    setsockopt(socket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
               &tv, sizeof(tv));
    setsockopt(socket_->native_handle(), SOL_SOCKET, SO_SNDTIMEO,
               &tv, sizeof(tv));
#endif

    log("[TcpTransport] Connected to " + host_ + ":" + std::to_string(port_));
    return true;
}

/**
 * @brief Gracefully close the socket if it is still open.
 *
 * @details
 * Performs a half-close shutdown followed by `close()` and resets the
 * `unique_ptr`. Safe to call on a closed transport (no-op) and from the
 * destructor.
 */
void TcpTransport::disconnect()
{
    if (socket_ && socket_->is_open()) {
        boost::system::error_code ec;
        socket_->shutdown(tcp::socket::shutdown_both, ec);
        socket_->close(ec);
        socket_.reset();
        log("[TcpTransport] Disconnected");
    }
}

/**
 * @brief Cheap check for whether the socket is open.
 * @return `true` if a socket exists and `is_open()` returns true.
 */
bool TcpTransport::isConnected() const
{
    return socket_ && socket_->is_open();
}

/**
 * @brief Push a complete frame out the socket via `asio::write`.
 * @param frame Bytes to send (caller-owned buffer).
 * @throws std::runtime_error if the transport is not connected, or if the
 *         underlying write fails (the socket is closed before throwing).
 */
void TcpTransport::send(const std::vector<uint8_t>& frame)
{
    if (!isConnected())
        throw std::runtime_error("TcpTransport: not connected");

    boost::system::error_code ec;
    asio::write(*socket_, asio::buffer(frame), ec);
    if (ec) {
        disconnect();
        throw std::runtime_error("TcpTransport: send failed - " + ec.message());
    }
}

/**
 * @brief Block until exactly @p numBytes have been received.
 * @param numBytes Number of bytes the caller expects (e.g. 7 for an MBAP
 *                 header, or `length - 1` for the PDU body).
 * @return A `vector<uint8_t>` of length @p numBytes.
 * @throws std::runtime_error on timeout, EOF, or any other I/O error
 *         (the socket is closed before throwing).
 */
std::vector<uint8_t> TcpTransport::recv(size_t numBytes)
{
    if (!isConnected())
        throw std::runtime_error("TcpTransport: not connected");

    std::vector<uint8_t> buf(numBytes);
    boost::system::error_code ec;
    asio::read(*socket_, asio::buffer(buf), ec);
    if (ec) {
        disconnect();
        throw std::runtime_error("TcpTransport: recv failed - " + ec.message());
    }
    return buf;
}

} // namespace Modbus
