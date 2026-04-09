/**
 * @file modbus_asn1_master.cpp
 * @brief Modbus Master implementation using Boost.Asio (TCP).
 *
 * Implements connection management and the transaction cycle for every
 * supported Modbus function code over TCP transport.
 */

#include "modbus_asn1_master.hpp"
#include <stdexcept>
#include <sstream>

namespace asio = boost::asio;
using tcp = asio::ip::tcp;
using udp = asio::ip::udp;

namespace Modbus {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
Master::Master(std::string host, uint16_t port, uint8_t unitId,
               int timeoutMs, LogCb logCb)
    : host_(std::move(host)), port_(port), unitId_(unitId),
      timeoutMs_(timeoutMs), logCb_(std::move(logCb))
{}

Master::~Master()
{
    disconnect();
}

// ---------------------------------------------------------------------------
// connect / disconnect
// ---------------------------------------------------------------------------
bool Master::connect()
{
    boost::system::error_code ec;

    if (transport_ == Transport::TCP) {
        if (tcpSocket_ && tcpSocket_->is_open()) return true;

        tcp::resolver resolver(ioc_);
        auto results = resolver.resolve(host_, std::to_string(port_), ec);
        if (ec) {
            log("[Master] resolve failed: " + ec.message());
            return false;
        }

        tcpSocket_ = std::make_unique<tcp::socket>(ioc_);
        asio::connect(*tcpSocket_, results, ec);
        if (ec) {
            log("[Master] connect() failed to " + host_ + ":" +
                std::to_string(port_) + " – " + ec.message());
            tcpSocket_.reset();
            return false;
        }

        // Set socket timeouts via native handle
#ifdef _WIN32
        DWORD tvMs = static_cast<DWORD>(timeoutMs_);
        setsockopt(tcpSocket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tvMs), sizeof(tvMs));
        setsockopt(tcpSocket_->native_handle(), SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&tvMs), sizeof(tvMs));
#else
        struct timeval tv;
        tv.tv_sec  = timeoutMs_ / 1000;
        tv.tv_usec = (timeoutMs_ % 1000) * 1000;
        setsockopt(tcpSocket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                   &tv, sizeof(tv));
        setsockopt(tcpSocket_->native_handle(), SOL_SOCKET, SO_SNDTIMEO,
                   &tv, sizeof(tv));
#endif

        log("[Master] Connected (TCP) to " + host_ + ":" + std::to_string(port_));
        return true;

    } else { // UDP
        if (udpSocket_ && udpSocket_->is_open()) return true;

        udp::resolver resolver(ioc_);
        auto results = resolver.resolve(host_, std::to_string(port_), ec);
        if (ec) {
            log("[Master] resolve failed: " + ec.message());
            return false;
        }
        udpEndpoint_ = *results.begin();

        udpSocket_ = std::make_unique<udp::socket>(ioc_, udp::v4());

#ifdef _WIN32
        DWORD tvMs = static_cast<DWORD>(timeoutMs_);
        setsockopt(udpSocket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tvMs), sizeof(tvMs));
#else
        struct timeval tv;
        tv.tv_sec  = timeoutMs_ / 1000;
        tv.tv_usec = (timeoutMs_ % 1000) * 1000;
        setsockopt(udpSocket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                   &tv, sizeof(tv));
#endif

        log("[Master] Ready (UDP) to " + host_ + ":" + std::to_string(port_));
        return true;
    }
}

void Master::disconnect()
{
    boost::system::error_code ec;
    if (tcpSocket_ && tcpSocket_->is_open()) {
        tcpSocket_->shutdown(tcp::socket::shutdown_both, ec);
        tcpSocket_->close(ec);
        tcpSocket_.reset();
        log("[Master] Disconnected (TCP)");
    }
}

bool Master::isConnected() const
{
    return tcpSocket_ && tcpSocket_->is_open();
}

// ---------------------------------------------------------------------------
// transaction  --  ASN.1 TLV encode → send → receive → TLV decode
// ---------------------------------------------------------------------------
ASN1::Bytes Master::transaction(const ASN1::Bytes& requestPdu)
{
    if (!isConnected() && !connect())
        throw std::runtime_error("Modbus Master: not connected");

    // ---- ASN.1 TLV encode request (internal validation) ----
    Bytes tlvRequest = pduToTlv(requestPdu);
    {
        std::ostringstream oss;
        oss << "[Master] PDU\u2192TLV encode: FC=0x" << std::hex
            << static_cast<int>(requestPdu[0])
            << "  TLV bytes=" << std::dec << tlvRequest.size();
        log(oss.str());
    }

    // Decode TLV back to raw PDU (round-trip validation via ASN1::Decoder)
    Bytes validatedPdu = tlvToPdu(tlvRequest);
    {
        std::ostringstream oss;
        oss << "[Master] TLV\u2192PDU decode: validated PDU bytes="
            << std::dec << validatedPdu.size();
        log(oss.str());
    }

    // ---- Build standard Modbus TCP frame ----
    MBAPHeader hdr;
    hdr.transactionId = ++transactionId_;
    hdr.protocolId    = PROTOCOL_ID;
    hdr.length        = static_cast<uint16_t>(1 + validatedPdu.size());
    hdr.unitId        = unitId_;

    ASN1::Bytes frame = hdr.toBytes();
    frame.insert(frame.end(), validatedPdu.begin(), validatedPdu.end());

    {
        std::ostringstream oss;
        oss << "[Master] TX(TCP) FC=0x" << std::hex
            << static_cast<int>(validatedPdu[0])
            << "  PDU bytes=" << std::dec << validatedPdu.size()
            << "  frame bytes=" << frame.size();
        log(oss.str());
    }

    // Send
    boost::system::error_code ec;
    asio::write(*tcpSocket_, asio::buffer(frame), ec);
    if (ec) {
        disconnect();
        throw std::runtime_error("Modbus Master: send failed – " + ec.message());
    }

    // Read MBAP header (7 bytes)
    ASN1::Bytes respHeader(MBAP_SIZE);
    asio::read(*tcpSocket_, asio::buffer(respHeader), ec);
    if (ec) {
        disconnect();
        throw std::runtime_error("Modbus Master: recv MBAP failed – " + ec.message());
    }

    MBAPHeader respHdr = MBAPHeader::fromBytes(respHeader);

    {
        std::ostringstream oss;
        oss << "[Master] RX MBAP: txId=" << respHdr.transactionId
            << " proto=" << respHdr.protocolId
            << " length=" << respHdr.length
            << " unitId=" << static_cast<int>(respHdr.unitId)
            << "  raw=[";
        for (size_t i = 0; i < respHeader.size(); ++i)
            oss << (i ? " " : "") << "0x" << std::hex
                << static_cast<int>(respHeader[i]);
        oss << "]";
        log(oss.str());
    }

    if (respHdr.transactionId != hdr.transactionId)
        throw std::runtime_error("Modbus Master: transaction ID mismatch");

    int pduLen = static_cast<int>(respHdr.length) - 1;
    if (pduLen <= 0 || pduLen > static_cast<int>(MAX_PDU_SIZE))
        throw std::runtime_error("Modbus Master: invalid response PDU length (length field=" +
            std::to_string(respHdr.length) + ", pduLen=" + std::to_string(pduLen) + ")");

    // Read response PDU
    ASN1::Bytes respPdu(static_cast<size_t>(pduLen));
    asio::read(*tcpSocket_, asio::buffer(respPdu), ec);
    if (ec) {
        disconnect();
        throw std::runtime_error("Modbus Master: recv PDU failed – " + ec.message());
    }

    {
        std::ostringstream oss;
        oss << "[Master] RX(TCP) FC=0x" << std::hex
            << static_cast<int>(respPdu[0])
            << "  PDU bytes=" << std::dec << respPdu.size();
        log(oss.str());
    }

    // Check for exception response
    if (respPdu[0] & static_cast<uint8_t>(FC::ERROR_FLAG)) {
        ExCode exCode = (respPdu.size() >= 2)
                    ? static_cast<ExCode>(respPdu[1])
                    : ExCode::SERVER_DEVICE_FAILURE;
        throw std::runtime_error("Modbus exception: " + exCodeStr(exCode));
    }

    // ---- ASN.1 TLV encode response (internal validation) ----
    Bytes tlvResponse = pduToTlv(respPdu);
    {
        std::ostringstream oss;
        oss << "[Master] Response PDU→TLV encode: FC=0x" << std::hex
            << static_cast<int>(respPdu[0])
            << "  TLV bytes=" << std::dec << tlvResponse.size();
        log(oss.str());
    }

    // Decode TLV back to raw PDU (round-trip validation via ASN1::Decoder)
    Bytes validatedResp = tlvToPdu(tlvResponse);
    {
        std::ostringstream oss;
        oss << "[Master] Response TLV→PDU decode: validated PDU bytes="
            << std::dec << validatedResp.size();
        log(oss.str());
    }

    return validatedResp;
}

// ---------------------------------------------------------------------------
// Public FC methods
// ---------------------------------------------------------------------------
std::vector<bool> Master::readCoils(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_COILS, startAddr, quantity);
    auto resp = transaction(pdu);
    auto coils = parseReadCoilsResponse(resp);
    coils.resize(quantity);  // trim padding bits from byte-packing
    return coils;
}

std::vector<bool> Master::readDiscreteInputs(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_DISCRETE_INPUTS, startAddr, quantity);
    auto resp = transaction(pdu);
    auto coils = parseReadCoilsResponse(resp);
    coils.resize(quantity);
    return coils;
}

std::vector<uint16_t> Master::readHoldingRegisters(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_HOLDING_REGISTERS, startAddr, quantity);
    auto resp = transaction(pdu);
    return parseReadRegistersResponse(resp);
}

std::vector<uint16_t> Master::readInputRegisters(uint16_t startAddr, uint16_t quantity)
{
    auto pdu  = buildReadRequest(FC::READ_INPUT_REGISTERS, startAddr, quantity);
    auto resp = transaction(pdu);
    return parseReadRegistersResponse(resp);
}

bool Master::writeSingleCoil(uint16_t address, bool value)
{
    auto pdu = buildWriteSingleCoil(address, value);
    transaction(pdu);
    return true;
}

bool Master::writeSingleRegister(uint16_t address, uint16_t value)
{
    auto pdu = buildWriteSingleRegister(address, value);
    transaction(pdu);
    return true;
}

bool Master::writeMultipleCoils(uint16_t startAddr, const std::vector<bool>& values)
{
    auto pdu = buildWriteMultipleCoils(startAddr, values);
    transaction(pdu);
    return true;
}

bool Master::writeMultipleRegisters(uint16_t startAddr,
                                    const std::vector<uint16_t>& values)
{
    auto pdu = buildWriteMultipleRegisters(startAddr, values);
    transaction(pdu);
    return true;
}

} // namespace Modbus
