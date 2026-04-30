/**
 * @file modbus_master.cpp
 * @brief `Modbus::Master` core — constructor + shared `transaction()` primitive.
 *
 * @details
 * This translation unit hosts the heart of the Layer 1 protocol code.
 * Each Modbus function code has its own translation unit under
 * `functions/` (one file per FC, e.g. `functions/read_coils.cpp`); they
 * all funnel through `Master::transaction()` defined here.
 *
 * ### Wire format
 * Standard **Modbus TCP**: an MBAP header (7 bytes) followed by a raw PDU.
 * No CRC, no escape sequences — TCP itself handles integrity. An earlier
 * ASN.1 TLV round-trip on request/response was decorative (`tlvToPdu` after
 * `pduToTlv` is identity by construction, so it caught no real bugs while
 * adding per-transaction overhead) and has been removed.
 *
 * ### Transaction flow
 * @dot
 * digraph txn {
 *   rankdir=TB;
 *   node [shape=box, style="rounded,filled"];
 *   in     [label="caller passes PDU\n(FC + payload)", fillcolor="#e6f0ff"];
 *   conn   [label="connect() if not connected", fillcolor="#fff2cc"];
 *   mbap   [label="build MBAP header\n(txId++ / proto / len / unitId)", fillcolor="#cfe2ff"];
 *   frame  [label="frame = MBAP + PDU", fillcolor="#cfe2ff"];
 *   tx     [label="transport.send(frame)", fillcolor="#ffe6b3"];
 *   rx1    [label="transport.recv(7)\n -> MBAP header", fillcolor="#ffe6b3"];
 *   match  [label="check txId match", fillcolor="#fff2cc"];
 *   rx2    [label="transport.recv(length-1)\n -> PDU body", fillcolor="#ffe6b3"];
 *   excep  [label="if FC high bit set\n -> throw exception", fillcolor="#f4cccc"];
 *   ret    [label="return response PDU", fillcolor="#c6efce"];
 *   in -> conn -> mbap -> frame -> tx -> rx1 -> match -> rx2 -> excep -> ret;
 * }
 * @enddot
 */

#include "modbus_master.hpp"
#include <sstream>
#include <stdexcept>

namespace Modbus {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

/**
 * @brief Bind a Master to a Transport.
 * @param transport Layer 0 transport reference (must outlive `*this`).
 * @param unitId    Default Modbus unit/slave ID for outgoing requests.
 * @param logCb     Optional log callback.
 */
Master::Master(Transport& transport, uint8_t unitId, LogCb logCb)
    : transport_(transport), unitId_(unitId), logCb_(std::move(logCb))
{}

// ---------------------------------------------------------------------------
// transaction — single Modbus TCP round-trip
// ---------------------------------------------------------------------------

/**
 * @brief Send one Modbus request PDU and return the response PDU.
 * @param requestPdu Application PDU (FC byte + payload, no MBAP).
 * @return Response PDU (FC byte + payload, MBAP stripped).
 *
 * @throws std::runtime_error
 *   - "not connected" — transport could not be opened.
 *   - "transaction ID mismatch" — slave's txId did not echo ours.
 *   - "invalid response PDU length" — MBAP length field out of range.
 *   - "Modbus exception: <name>" — slave returned an exception response
 *     (FC byte's high bit was set).
 *
 * @details
 * Every public FC method in the master ends up calling this primitive
 * with its already-built request PDU. The bookkeeping (MBAP, txId
 * matching, exception detection) is centralised here so per-FC files
 * stay tiny.
 */
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
