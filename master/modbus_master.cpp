/**
 * @file modbus_master.cpp
 * @brief Modbus Master core — Layer 1 (no asio).
 *
 * Hosts the constructor and the shared @ref Master::transaction primitive.
 * Each Modbus function code lives in its own translation unit under
 * `functions/` (one file per FC) — see e.g. functions/read_coils.cpp.
 *
 * Wire format: standard Modbus TCP (MBAP header + raw PDU). The earlier
 * ASN.1 TLV round-trip on request/response was decorative — `tlvToPdu`
 * after `pduToTlv` returns the original PDU by construction, so it caught
 * no real bugs and added per-transaction overhead. Removed.
 */

#include "modbus_master.hpp"
#include <sstream>
#include <stdexcept>

namespace Modbus {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Master::Master(Transport& transport, uint8_t unitId, LogCb logCb)
    : transport_(transport), unitId_(unitId), logCb_(std::move(logCb))
{}

// ---------------------------------------------------------------------------
// transaction  --  ASN.1 TLV encode -> send -> receive -> TLV decode
// ---------------------------------------------------------------------------
Bytes Master::transaction(const Bytes& requestPdu)
{
    if (!isConnected() && !connect())
        throw std::runtime_error("Modbus Master: not connected");

    // ---- Build standard Modbus TCP frame ----
    MBAPHeader hdr;
    hdr.transactionId = ++transactionId_;
    hdr.protocolId    = PROTOCOL_ID;
    hdr.length        = static_cast<uint16_t>(1 + requestPdu.size());
    hdr.unitId        = unitId_;

    Bytes frame = hdr.toBytes();
    frame.insert(frame.end(), requestPdu.begin(), requestPdu.end());

    {
        std::ostringstream oss;
        oss << "[Master] TX FC=0x" << std::hex
            << static_cast<int>(requestPdu[0])
            << "  PDU bytes=" << std::dec << requestPdu.size()
            << "  frame bytes=" << frame.size();
        log(oss.str());
    }

    // Send through Layer 0
    transport_.send(frame);

    // Read MBAP header (7 bytes)
    Bytes respHeader = transport_.recv(MBAP_SIZE);
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
    Bytes respPdu = transport_.recv(static_cast<size_t>(pduLen));

    {
        std::ostringstream oss;
        oss << "[Master] RX FC=0x" << std::hex
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

    return respPdu;
}

} // namespace Modbus
