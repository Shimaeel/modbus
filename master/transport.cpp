/**
 * @file transport.cpp
 * @brief TcpTransport implementation (Layer 0). All asio usage lives here.
 */

#include "transport.hpp"
#include <stdexcept>

namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace Modbus {

TcpTransport::TcpTransport(std::string host, uint16_t port,
                           int timeoutMs, LogCb logCb)
    : host_(std::move(host)), port_(port), timeoutMs_(timeoutMs)
{
    logCb_ = std::move(logCb);
}

TcpTransport::~TcpTransport()
{
    disconnect();
}

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

bool TcpTransport::isConnected() const
{
    return socket_ && socket_->is_open();
}

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
