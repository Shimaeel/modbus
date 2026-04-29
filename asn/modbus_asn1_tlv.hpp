#pragma once
/**
 * @file modbus_asn1_tlv.hpp
 * @brief ASN.1 TLV encoding for Modbus function codes + extensible dispatcher.
 *
 * @details
 * Only the function-code portion of the Modbus PDU is encoded in ASN.1 TLV.
 * The MBAP header remains standard Modbus TCP (raw big-endian bytes).
 *
 * TLV wire layout (replaces the raw FC+payload inside the PDU):
 * @code
 *   SEQUENCE {                       -- Tag 0x30
 *       INTEGER      functionCode    -- Tag 0x02 | Len | FC value
 *       OCTET STRING payload         -- Tag 0x04 | Len | FC-specific bytes
 *   }
 * @endcode
 *
 * The FcDispatcher decodes the incoming TLV, looks up a handler by FC,
 * invokes it, and returns a TLV-encoded response.  New function codes
 * are added by simply calling registerHandler().
 */

#include "../modbus_common.hpp"
#include "asn1.hpp"

#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace Modbus {

// ═══════════════════════════════════════════════════════════════════════════════
//  TLV data structure
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @struct FcTlv
 * @brief Decoded TLV holding a function code and its associated payload.
 */
struct FcTlv {
    FC    functionCode;  ///< Modbus function code (Type).
    Bytes payload;       ///< FC-specific data bytes (Value), excluding the FC byte.
};

// ═══════════════════════════════════════════════════════════════════════════════
//  TLV Encoding / Decoding
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Encode a function code + payload into ASN.1 TLV format.
 *
 * Produces: SEQUENCE { INTEGER fc, OCTET_STRING payload }
 *
 * @param fc      Modbus function code (used as the TLV Type value).
 * @param payload FC-specific data bytes (the TLV Value).
 * @return ASN.1 BER-encoded byte sequence.
 */
inline Bytes encodeFcTlv(FC fc, const Bytes& payload)
{
    ASN1::Encoder enc;
    enc.beginSequence();
    enc.encodeInteger(static_cast<int64_t>(static_cast<uint8_t>(fc)));
    enc.encodeOctetString(payload);
    enc.endSequence();
    return enc.getBytes();
}

/**
 * @brief Decode ASN.1 TLV bytes into a function code + payload.
 * @param tlvData BER-encoded bytes: SEQUENCE { INTEGER, OCTET_STRING }.
 * @return Decoded FcTlv.
 * @throws std::runtime_error On malformed TLV data.
 */
inline FcTlv decodeFcTlv(const Bytes& tlvData)
{
    ASN1::Decoder dec(tlvData);
    dec.beginSequence();
    uint8_t fcByte   = static_cast<uint8_t>(dec.decodeInteger());
    Bytes   payload  = dec.decodeOctetString();
    dec.endSequence();
    return FcTlv{ static_cast<FC>(fcByte), std::move(payload) };
}

/**
 * @brief Convert a raw Modbus PDU [FC][payload…] → ASN.1 TLV-encoded bytes.
 *
 * Useful for wrapping existing PDU builders before transmission.
 *
 * @param pdu Standard Modbus PDU (FC byte + data).
 * @return TLV-encoded bytes.
 */
inline Bytes pduToTlv(const Bytes& pdu)
{
    if (pdu.empty())
        throw std::runtime_error("pduToTlv: empty PDU");
    FC    fc = static_cast<FC>(pdu[0]);
    Bytes payload(pdu.begin() + 1, pdu.end());
    return encodeFcTlv(fc, payload);
}

/**
 * @brief Convert TLV-encoded bytes → raw Modbus PDU [FC][payload…].
 *
 * Useful on the receiving side when legacy parsers expect a raw PDU.
 *
 * @param tlvData ASN.1 TLV-encoded bytes.
 * @return Standard Modbus PDU.
 */
inline Bytes tlvToPdu(const Bytes& tlvData)
{
    FcTlv tlv = decodeFcTlv(tlvData);
    Bytes pdu;
    pdu.reserve(1 + tlv.payload.size());
    pdu.push_back(static_cast<uint8_t>(tlv.functionCode));
    pdu.insert(pdu.end(), tlv.payload.begin(), tlv.payload.end());
    return pdu;
}

/**
 * @brief Build a TLV-encoded Modbus exception response.
 * @param fc Original function code from the failed request.
 * @param ec Exception code describing the error.
 * @return TLV bytes: SEQUENCE { INTEGER (fc|0x80), OCTET_STRING {ec} }.
 */
inline Bytes buildExceptionTlv(FC fc, ExCode ec)
{
    FC errorFc = static_cast<FC>(
        static_cast<uint8_t>(fc) | static_cast<uint8_t>(FC::ERROR_FLAG));
    Bytes payload = { static_cast<uint8_t>(ec) };
    return encodeFcTlv(errorFc, payload);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Exception type for FC handlers
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Exception thrown by an FC handler to signal a Modbus error.
 *
 * The dispatcher catches this and automatically builds an exception TLV.
 */
struct ModbusException : std::runtime_error {
    ExCode code;  ///< The Modbus exception code to send back.
    explicit ModbusException(ExCode c)
        : std::runtime_error("Modbus exception: " + exCodeStr(c)), code(c) {}
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Dispatcher
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief FC handler function signature.
 *
 * @param fc       The decoded function code.
 * @param payload  Bytes following the FC (startAddr, quantity, data, etc.).
 * @return Response payload bytes (without the FC byte; the dispatcher adds it).
 * @throws ModbusException to signal a Modbus error response.
 */
using FcHandler = std::function<Bytes(FC fc, const Bytes& payload)>;

/**
 * @class FcDispatcher
 * @brief TLV-aware dispatcher that routes Modbus requests to registered handlers.
 *
 * @par Usage
 * @code
 *   FcDispatcher dispatcher;
 *
 *   dispatcher.registerHandler(FC::READ_COILS,
 *       [](FC fc, const Bytes& payload) -> Bytes {
 *           // parse payload, interact with data model, build response …
 *           return responsePayload;
 *       });
 *
 *   // At runtime (inside processRequest):
 *   Bytes tlvResp = dispatcher.dispatch(tlvRequest);
 * @endcode
 *
 * New function codes are added with a single registerHandler() call —
 * no switch/case to modify.
 */
class FcDispatcher {
public:
    /**
     * @brief Register a handler for a specific function code.
     * @param fc      Function code to handle.
     * @param handler Callback that processes the request payload.
     */
    void registerHandler(FC fc, FcHandler handler)
    {
        handlers_[static_cast<uint8_t>(fc)] = std::move(handler);
    }

    /**
     * @brief Check whether a handler is registered for a given FC.
     */
    bool hasHandler(FC fc) const
    {
        return handlers_.count(static_cast<uint8_t>(fc)) > 0;
    }

    /**
     * @brief Directly invoke a handler by FC (bypasses TLV encode/decode).
     *
     * Used by the standard-wire-format path so that the same handler
     * logic is reused without any ASN.1 TLV wrapping on the wire.
     *
     * @param fc      Function code to dispatch.
     * @param payload Raw payload bytes (without the FC byte).
     * @return Response payload bytes (without the FC byte).
     * @throws ModbusException on Modbus-level errors.
     */
    Bytes invokeHandler(FC fc, const Bytes& payload)
    {
        auto it = handlers_.find(static_cast<uint8_t>(fc));
        if (it == handlers_.end())
            throw ModbusException(ExCode::ILLEGAL_FUNCTION);
        return it->second(fc, payload);
    }

    /**
     * @brief Dispatch a TLV-encoded request to the appropriate handler.
     *
     * 1. Decode TLV → FC + payload
     * 2. Look up registered handler for FC
     * 3. Invoke handler(fc, payload) → response payload
     * 4. Encode response as TLV: SEQUENCE { INTEGER fc, OCTET_STRING resp }
     *
     * Error handling:
     * - No handler registered → ILLEGAL_FUNCTION exception TLV.
     * - Handler throws ModbusException → corresponding exception TLV.
     * - Any other exception → SERVER_DEVICE_FAILURE exception TLV.
     *
     * @param tlvRequest ASN.1 TLV-encoded request bytes.
     * @return ASN.1 TLV-encoded response bytes.
     */
    Bytes dispatch(const Bytes& tlvRequest)
    {
        FcTlv req = decodeFcTlv(tlvRequest);

        auto it = handlers_.find(static_cast<uint8_t>(req.functionCode));
        if (it == handlers_.end())
            return buildExceptionTlv(req.functionCode, ExCode::ILLEGAL_FUNCTION);

        try {
            Bytes responsePayload = it->second(req.functionCode, req.payload);
            return encodeFcTlv(req.functionCode, responsePayload);
        } catch (const ModbusException& e) {
            return buildExceptionTlv(req.functionCode, e.code);
        } catch (...) {
            return buildExceptionTlv(req.functionCode, ExCode::SERVER_DEVICE_FAILURE);
        }
    }

private:
    std::unordered_map<uint8_t, FcHandler> handlers_;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Payload-level parsers  (work WITHOUT the leading FC byte)
// ═══════════════════════════════════════════════════════════════════════════════

/** @brief Parse read-request payload: [StartAddr(2)][Qty(2)]. */
inline ReadRequest parseReadPayload(const Bytes& payload)
{
    ReadRequest req;
    req.startAddress = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    req.quantity     = (static_cast<uint16_t>(payload[2]) << 8) | payload[3];
    return req;
}

/** @brief Build read-coils/discrete-inputs response payload: [ByteCount][CoilData…]. */
inline Bytes buildCoilsResponsePayload(const std::vector<bool>& coils)
{
    uint8_t byteCount = static_cast<uint8_t>((coils.size() + 7) / 8);
    Bytes payload;
    payload.reserve(1 + byteCount);
    payload.push_back(byteCount);
    for (uint8_t i = 0; i < byteCount; ++i) {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; ++bit) {
            size_t idx = static_cast<size_t>(i) * 8 + bit;
            if (idx < coils.size() && coils[idx])
                byte |= (1 << bit);
        }
        payload.push_back(byte);
    }
    return payload;
}

/** @brief Build read-registers response payload: [ByteCount][RegData(N×2)]. */
inline Bytes buildRegistersResponsePayload(const std::vector<uint16_t>& regs)
{
    uint8_t byteCount = static_cast<uint8_t>(regs.size() * 2);
    Bytes payload;
    payload.reserve(1 + byteCount);
    payload.push_back(byteCount);
    for (uint16_t r : regs) {
        payload.push_back(static_cast<uint8_t>(r >> 8));
        payload.push_back(static_cast<uint8_t>(r & 0xFF));
    }
    return payload;
}

/** @brief Parse write-single-coil payload: [Addr(2)][Value(2)]. */
inline WriteSingleCoilReq parseWriteSingleCoilPayload(const Bytes& payload)
{
    WriteSingleCoilReq r;
    r.address = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    r.value   = (payload[2] == 0xFF);
    return r;
}

/** @brief Parse write-single-register payload: [Addr(2)][Value(2)]. */
inline WriteSingleRegReq parseWriteSingleRegPayload(const Bytes& payload)
{
    WriteSingleRegReq r;
    r.address = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    r.value   = (static_cast<uint16_t>(payload[2]) << 8) | payload[3];
    return r;
}

/** @brief Parse write-multiple-coils payload: [StartAddr(2)][Qty(2)][ByteCount(1)][Data…]. */
inline WriteMultipleCoilsReq parseWriteMultipleCoilsPayload(const Bytes& payload)
{
    WriteMultipleCoilsReq r;
    r.startAddress = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    uint16_t qty   = (static_cast<uint16_t>(payload[2]) << 8) | payload[3];
    for (uint16_t i = 0; i < qty; ++i) {
        uint8_t byteIdx = static_cast<uint8_t>(i / 8);
        uint8_t bitIdx  = static_cast<uint8_t>(i % 8);
        r.coils.push_back((payload[5 + byteIdx] >> bitIdx) & 0x01);
    }
    return r;
}

/** @brief Parse write-multiple-registers payload: [StartAddr(2)][Qty(2)][ByteCount(1)][Data…]. */
inline WriteMultipleRegsReq parseWriteMultipleRegsPayload(const Bytes& payload)
{
    WriteMultipleRegsReq r;
    r.startAddress = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    uint16_t qty   = (static_cast<uint16_t>(payload[2]) << 8) | payload[3];
    for (uint16_t i = 0; i < qty; ++i) {
        size_t offset = 5 + static_cast<size_t>(i) * 2;
        uint16_t val = (static_cast<uint16_t>(payload[offset]) << 8) | payload[offset + 1];
        r.registers.push_back(val);
    }
    return r;
}

/** @brief Build write-multiple response payload: [StartAddr(2)][Qty(2)]. */
inline Bytes buildWriteMultipleResponsePayload(uint16_t startAddr, uint16_t quantity)
{
    Bytes payload;
    payload.push_back(static_cast<uint8_t>(startAddr >> 8));
    payload.push_back(static_cast<uint8_t>(startAddr & 0xFF));
    payload.push_back(static_cast<uint8_t>(quantity >> 8));
    payload.push_back(static_cast<uint8_t>(quantity & 0xFF));
    return payload;
}

} // namespace Modbus
