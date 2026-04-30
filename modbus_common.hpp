#pragma once
/**
 * @file modbus_common.hpp
 * @brief Shared Modbus TCP types, constants, and PDU helpers (Layer 1).
 *
 * @details
 * This single header is the **Layer 1 protocol toolbox** used by both the
 * master and the slave. It declares everything Modbus-specific that is
 * independent of who is talking — function codes, exception codes, the
 * MBAP header struct, request builders, and response parsers — so neither
 * side has to repeat itself.
 *
 * - Transport : Modbus TCP (MBAP header, port 502).
 * - Encoding  : Standard Modbus wire format (raw big-endian bytes).
 *
 * ### What lives here
 * - `enum class FC`     — Modbus function-code values (`READ_COILS = 0x01`, …).
 * - `enum class ExCode` — Modbus exception codes returned in error responses.
 * - `struct MBAPHeader` — 7-byte TCP application-protocol header + (de)serializer.
 * - `struct Frame`      — convenience: MBAP header + raw PDU.
 * - `buildXxxRequest()` — master-side PDU builders (one per FC).
 * - `parseXxxRequest()` — slave-side PDU parsers (one per FC).
 * - `buildXxxResponse()` — slave-side response builders.
 * - `parseXxxResponse()` — master-side response parsers.
 *
 * ### Wire formats
 * @code
 *   PDU (function-code + payload, no transport framing):
 *     [FC 1 byte][payload bytes ...]
 *
 *   Modbus TCP frame (this is what hits the network):
 *     [MBAP 7 bytes][FC 1 byte][payload bytes ...]
 *
 *   MBAP header layout (big-endian):
 *     | Transaction ID (2) | Protocol ID (2) | Length (2) | Unit ID (1) |
 * @endcode
 *
 * ### Where this fits in the stack
 * @dot
 * digraph common {
 *   rankdir=TB;
 *   node [shape=box, style="rounded,filled"];
 *   App      [label="Application\n(main_master.cpp)", fillcolor="#e6f0ff"];
 *   FCFiles  [label="Per-FC files\n(functions/*.cpp)", fillcolor="#cfe2ff"];
 *   Master   [label="Modbus::Master\n(modbus_master.cpp)", fillcolor="#cfe2ff"];
 *   Common   [label="modbus_common.hpp\n(THIS FILE)", fillcolor="#fff2cc"];
 *   Trans    [label="Transport (TCP)", fillcolor="#ffe6b3"];
 *   App -> Master -> FCFiles -> Common;
 *   Master -> Common [label="MBAP build/parse"];
 *   Common -> Trans  [label="bytes"];
 * }
 * @enddot
 *
 * @note No `<boost/asio.hpp>` include here — this header is **plain C++**
 *       so unit tests can feed PDU byte vectors directly without spinning
 *       up a socket.
 */

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace Modbus {

/** @brief Convenience alias for a raw byte buffer. */
using Bytes = std::vector<uint8_t>;

/** @name Protocol constants
 *  @{
 */
static constexpr uint16_t DEFAULT_PORT     = 502;     ///< Standard Modbus TCP port.
static constexpr uint16_t PROTOCOL_ID      = 0x0000;  ///< Modbus protocol identifier (always 0).
static constexpr size_t   MBAP_SIZE        = 7;       ///< MBAP header size: 6 header + 1 unit-id.
static constexpr size_t   MAX_PDU_SIZE     = 253;     ///< Maximum PDU size per Modbus spec.
/** @} */

/**
 * @brief Modbus function codes.
 */
enum class FC : uint8_t {
    READ_COILS               = 0x01,  ///< FC 01 - Read coils.
    READ_DISCRETE_INPUTS     = 0x02,  ///< FC 02 - Read discrete inputs.
    READ_HOLDING_REGISTERS   = 0x03,  ///< FC 03 - Read holding registers.
    READ_INPUT_REGISTERS     = 0x04,  ///< FC 04 - Read input registers.
    WRITE_SINGLE_COIL        = 0x05,  ///< FC 05 - Write single coil.
    WRITE_SINGLE_REGISTER    = 0x06,  ///< FC 06 - Write single register.
    WRITE_MULTIPLE_COILS     = 0x0F,  ///< FC 15 - Write multiple coils.
    WRITE_MULTIPLE_REGISTERS = 0x10,  ///< FC 16 - Write multiple registers.
    MASK_WRITE_REGISTER      = 0x16,  ///< FC 22 - Mask write register.
    READ_WRITE_MULTIPLE_REGS = 0x17,  ///< FC 23 - Read/write multiple registers.
    READ_FIFO_QUEUE          = 0x18,  ///< FC 24 - Read FIFO queue.
    ERROR_FLAG               = 0x80,  ///< OR'd with FC in exception responses.
};

/**
 * @brief Modbus exception codes returned in error responses.
 */
enum class ExCode : uint8_t {
    ILLEGAL_FUNCTION         = 0x01,  ///< Function code not supported.
    ILLEGAL_DATA_ADDRESS     = 0x02,  ///< Address out of valid range.
    ILLEGAL_DATA_VALUE       = 0x03,  ///< Value out of valid range.
    SERVER_DEVICE_FAILURE    = 0x04,  ///< Unrecoverable server error.
};

/**
 * @brief Convert an exception code to a human-readable string.
 * @param c The exception code.
 * @return Descriptive string for the code.
 */
inline std::string exCodeStr(ExCode c)
{
    switch (c) {
    case ExCode::ILLEGAL_FUNCTION:      return "Illegal Function";
    case ExCode::ILLEGAL_DATA_ADDRESS:  return "Illegal Data Address";
    case ExCode::ILLEGAL_DATA_VALUE:    return "Illegal Data Value";
    case ExCode::SERVER_DEVICE_FAILURE: return "Server Device Failure";
    default: return "Unknown";
    }
}

/**
 * @struct MBAPHeader
 * @brief Modbus Application Protocol (MBAP) header for TCP transport.
 */
struct MBAPHeader {
    uint16_t transactionId {0};   ///< Echoed by the slave to match request/response.
    uint16_t protocolId    {0};   ///< Always 0x0000 for Modbus.
    uint16_t length        {0};   ///< Number of bytes following (unit-id + PDU).
    uint8_t  unitId        {1};   ///< Slave address (1-247).

    /**
     * @brief Serialise the header to a 7-byte buffer.
     * @return Big-endian encoded header bytes.
     */
    Bytes toBytes() const
    {
        Bytes b(7);
        b[0] = (transactionId >> 8) & 0xFF;
        b[1] =  transactionId       & 0xFF;
        b[2] = (protocolId   >> 8) & 0xFF;
        b[3] =  protocolId          & 0xFF;
        b[4] = (length       >> 8) & 0xFF;
        b[5] =  length              & 0xFF;
        b[6] =  unitId;
        return b;
    }

    /**
     * @brief Parse an MBAP header from a byte buffer.
     * @param b Buffer containing at least 7 bytes starting at @p offset.
     * @param offset Starting position within the buffer (default 0).
     * @return Parsed MBAPHeader.
     */
    static MBAPHeader fromBytes(const Bytes& b, size_t offset = 0)
    {
        MBAPHeader h;
        h.transactionId = (static_cast<uint16_t>(b[offset+0]) << 8) | b[offset+1];
        h.protocolId    = (static_cast<uint16_t>(b[offset+2]) << 8) | b[offset+3];
        h.length        = (static_cast<uint16_t>(b[offset+4]) << 8) | b[offset+5];
        h.unitId        = b[offset+6];
        return h;
    }
};

/**
 * @struct Frame
 * @brief Complete Modbus TCP frame (MBAP header + raw PDU bytes).
 */
struct Frame {
    MBAPHeader  header;   ///< MBAP header.
    Bytes       pdu;      ///< PDU: function code byte + standard Modbus payload.

    /**
     * @brief Serialise the entire frame to a byte buffer.
     * @return Concatenation of header bytes and PDU bytes.
     */
    Bytes toBytes() const
    {
        Bytes out = header.toBytes();
        out.insert(out.end(), pdu.begin(), pdu.end());
        return out;
    }
};

/** @name PDU builders (Master -> Slave requests)
 *  @{
 */

/**
 * @brief Build a read request PDU (FC 01/02/03/04).
 * @param fc       Function code (READ_COILS, READ_DISCRETE_INPUTS, etc.).
 * @param startAddr Starting address of the data range.
 * @param quantity  Number of items to read.
 * @return Standard Modbus PDU: [FC][StartAddr_Hi][StartAddr_Lo][Qty_Hi][Qty_Lo].
 */
inline Bytes buildReadRequest(FC fc,
                              uint16_t startAddr,
                              uint16_t quantity)
{
    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(fc));
    pdu.push_back(static_cast<uint8_t>(startAddr >> 8));
    pdu.push_back(static_cast<uint8_t>(startAddr & 0xFF));
    pdu.push_back(static_cast<uint8_t>(quantity >> 8));
    pdu.push_back(static_cast<uint8_t>(quantity & 0xFF));
    return pdu;
}

/**
 * @brief Build a Write Single Coil request PDU (FC 05).
 * @param address Coil address.
 * @param value   Coil state to write.
 * @return Standard Modbus PDU: [FC][Addr_Hi][Addr_Lo][Value_Hi][Value_Lo].
 *         Value = 0xFF00 for ON, 0x0000 for OFF.
 */
inline Bytes buildWriteSingleCoil(uint16_t address, bool value)
{
    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(FC::WRITE_SINGLE_COIL));
    pdu.push_back(static_cast<uint8_t>(address >> 8));
    pdu.push_back(static_cast<uint8_t>(address & 0xFF));
    pdu.push_back(value ? 0xFF : 0x00);
    pdu.push_back(0x00);
    return pdu;
}

/**
 * @brief Build a Write Single Register request PDU (FC 06).
 * @param address Register address.
 * @param value   Register value to write.
 * @return Standard Modbus PDU: [FC][Addr_Hi][Addr_Lo][Value_Hi][Value_Lo].
 */
inline Bytes buildWriteSingleRegister(uint16_t address, uint16_t value)
{
    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(FC::WRITE_SINGLE_REGISTER));
    pdu.push_back(static_cast<uint8_t>(address >> 8));
    pdu.push_back(static_cast<uint8_t>(address & 0xFF));
    pdu.push_back(static_cast<uint8_t>(value >> 8));
    pdu.push_back(static_cast<uint8_t>(value & 0xFF));
    return pdu;
}

/**
 * @brief Build a Write Multiple Coils request PDU (FC 0F).
 * @param startAddr Starting coil address.
 * @param coils     Vector of boolean coil values to write.
 * @return Standard Modbus PDU: [FC][StartAddr(2)][Quantity(2)][ByteCount(1)][CoilData(N)].
 */
inline Bytes buildWriteMultipleCoils(uint16_t startAddr,
                                     const std::vector<bool>& coils)
{
    uint16_t qty = static_cast<uint16_t>(coils.size());
    uint8_t byteCount = static_cast<uint8_t>((qty + 7) / 8);

    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(FC::WRITE_MULTIPLE_COILS));
    pdu.push_back(static_cast<uint8_t>(startAddr >> 8));
    pdu.push_back(static_cast<uint8_t>(startAddr & 0xFF));
    pdu.push_back(static_cast<uint8_t>(qty >> 8));
    pdu.push_back(static_cast<uint8_t>(qty & 0xFF));
    pdu.push_back(byteCount);

    // Pack coils into bytes (LSB of first byte = first coil)
    for (uint8_t i = 0; i < byteCount; ++i) {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; ++bit) {
            size_t idx = static_cast<size_t>(i) * 8 + bit;
            if (idx < coils.size() && coils[idx])
                byte |= (1 << bit);
        }
        pdu.push_back(byte);
    }
    return pdu;
}

/**
 * @brief Build a Write Multiple Registers request PDU (FC 10).
 * @param startAddr Starting register address.
 * @param regs      Vector of register values to write.
 * @return Standard Modbus PDU: [FC][StartAddr(2)][Quantity(2)][ByteCount(1)][RegData(N*2)].
 */
inline Bytes buildWriteMultipleRegisters(uint16_t startAddr,
                                         const std::vector<uint16_t>& regs)
{
    uint16_t qty = static_cast<uint16_t>(regs.size());
    uint8_t byteCount = static_cast<uint8_t>(qty * 2);

    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(FC::WRITE_MULTIPLE_REGISTERS));
    pdu.push_back(static_cast<uint8_t>(startAddr >> 8));
    pdu.push_back(static_cast<uint8_t>(startAddr & 0xFF));
    pdu.push_back(static_cast<uint8_t>(qty >> 8));
    pdu.push_back(static_cast<uint8_t>(qty & 0xFF));
    pdu.push_back(byteCount);

    for (uint16_t r : regs) {
        pdu.push_back(static_cast<uint8_t>(r >> 8));
        pdu.push_back(static_cast<uint8_t>(r & 0xFF));
    }
    return pdu;
}

/**
 * @brief Build a Mask Write Register request PDU (FC 22).
 * @param address  Register address.
 * @param andMask  AND mask applied to current value.
 * @param orMask   OR mask applied after the AND.
 * @return Standard Modbus PDU: [FC][Addr(2)][AndMask(2)][OrMask(2)].
 *
 * Slave computes: result = (current AND andMask) OR (orMask AND (NOT andMask)).
 */
inline Bytes buildMaskWriteRegister(uint16_t address,
                                    uint16_t andMask,
                                    uint16_t orMask)
{
    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(FC::MASK_WRITE_REGISTER));
    pdu.push_back(static_cast<uint8_t>(address >> 8));
    pdu.push_back(static_cast<uint8_t>(address & 0xFF));
    pdu.push_back(static_cast<uint8_t>(andMask >> 8));
    pdu.push_back(static_cast<uint8_t>(andMask & 0xFF));
    pdu.push_back(static_cast<uint8_t>(orMask >> 8));
    pdu.push_back(static_cast<uint8_t>(orMask & 0xFF));
    return pdu;
}

/**
 * @brief Build a Read/Write Multiple Registers request PDU (FC 23).
 * @param readAddr  First register to read.
 * @param readQty   Number of registers to read (1-125).
 * @param writeAddr First register to write.
 * @param writeRegs Register values to write (1-121).
 * @return Standard Modbus PDU:
 *         [FC][ReadAddr(2)][ReadQty(2)][WriteAddr(2)][WriteQty(2)][ByteCount][WriteData(N*2)].
 *
 * Write happens first, then the read uses the post-write state.
 */
inline Bytes buildReadWriteMultipleRegisters(uint16_t readAddr,
                                             uint16_t readQty,
                                             uint16_t writeAddr,
                                             const std::vector<uint16_t>& writeRegs)
{
    uint16_t writeQty   = static_cast<uint16_t>(writeRegs.size());
    uint8_t  byteCount  = static_cast<uint8_t>(writeQty * 2);

    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(FC::READ_WRITE_MULTIPLE_REGS));
    pdu.push_back(static_cast<uint8_t>(readAddr  >> 8));
    pdu.push_back(static_cast<uint8_t>(readAddr  & 0xFF));
    pdu.push_back(static_cast<uint8_t>(readQty   >> 8));
    pdu.push_back(static_cast<uint8_t>(readQty   & 0xFF));
    pdu.push_back(static_cast<uint8_t>(writeAddr >> 8));
    pdu.push_back(static_cast<uint8_t>(writeAddr & 0xFF));
    pdu.push_back(static_cast<uint8_t>(writeQty  >> 8));
    pdu.push_back(static_cast<uint8_t>(writeQty  & 0xFF));
    pdu.push_back(byteCount);

    for (uint16_t r : writeRegs) {
        pdu.push_back(static_cast<uint8_t>(r >> 8));
        pdu.push_back(static_cast<uint8_t>(r & 0xFF));
    }
    return pdu;
}

/**
 * @brief Build a Read FIFO Queue request PDU (FC 24).
 * @param pointerAddr Address of the FIFO pointer register.
 * @return Standard Modbus PDU: [FC][PointerAddr(2)].
 */
inline Bytes buildReadFifoQueue(uint16_t pointerAddr)
{
    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(FC::READ_FIFO_QUEUE));
    pdu.push_back(static_cast<uint8_t>(pointerAddr >> 8));
    pdu.push_back(static_cast<uint8_t>(pointerAddr & 0xFF));
    return pdu;
}

/** @} */

/** @name PDU builders (Slave -> Master responses)
 *  @{
 */

/**
 * @brief Build a Read Coils / Discrete Inputs response PDU (FC 01/02).
 * @param fc    Function code (READ_COILS or READ_DISCRETE_INPUTS).
 * @param coils Vector of boolean values read from the data model.
 * @return Standard Modbus PDU: [FC][ByteCount(1)][CoilData(N)] bit-packed.
 */
inline Bytes buildReadCoilsResponse(FC fc,
                                    const std::vector<bool>& coils)
{
    uint8_t byteCount = static_cast<uint8_t>((coils.size() + 7) / 8);

    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(fc));
    pdu.push_back(byteCount);

    for (uint8_t i = 0; i < byteCount; ++i) {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; ++bit) {
            size_t idx = static_cast<size_t>(i) * 8 + bit;
            if (idx < coils.size() && coils[idx])
                byte |= (1 << bit);
        }
        pdu.push_back(byte);
    }
    return pdu;
}

/**
 * @brief Build a Read Holding/Input Registers response PDU (FC 03/04).
 * @param fc   Function code (READ_HOLDING_REGISTERS or READ_INPUT_REGISTERS).
 * @param regs Vector of register values read from the data model.
 * @return Standard Modbus PDU: [FC][ByteCount(1)][RegData(N*2)] big-endian.
 */
inline Bytes buildReadRegistersResponse(FC fc,
                                        const std::vector<uint16_t>& regs)
{
    uint8_t byteCount = static_cast<uint8_t>(regs.size() * 2);

    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(fc));
    pdu.push_back(byteCount);

    for (uint16_t r : regs) {
        pdu.push_back(static_cast<uint8_t>(r >> 8));
        pdu.push_back(static_cast<uint8_t>(r & 0xFF));
    }
    return pdu;
}

/**
 * @brief Build a Write Multiple response PDU (FC 0F/10).
 *
 * Also usable as echo response for FC 05/06.
 * @param fc        Function code.
 * @param startAddr Starting address that was written.
 * @param quantity  Number of items written.
 * @return Standard Modbus PDU: [FC][StartAddr(2)][Quantity(2)].
 */
inline Bytes buildWriteMultipleResponse(FC fc,
                                        uint16_t startAddr,
                                        uint16_t quantity)
{
    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(fc));
    pdu.push_back(static_cast<uint8_t>(startAddr >> 8));
    pdu.push_back(static_cast<uint8_t>(startAddr & 0xFF));
    pdu.push_back(static_cast<uint8_t>(quantity >> 8));
    pdu.push_back(static_cast<uint8_t>(quantity & 0xFF));
    return pdu;
}

/**
 * @brief Build a Modbus exception (error) response PDU.
 * @param fc Function code of the failed request.
 * @param ec Exception code describing the error.
 * @return 2-byte PDU: (FC | 0x80), exception code.
 */
inline Bytes buildExceptionResponse(FC fc, ExCode ec)
{
    Bytes pdu;
    pdu.push_back(static_cast<uint8_t>(fc) | static_cast<uint8_t>(FC::ERROR_FLAG));
    pdu.push_back(static_cast<uint8_t>(ec));
    return pdu;
}

/** @} */

/** @name PDU parsers (used by the receiver side)
 *  @{
 */

/**
 * @struct ReadRequest
 * @brief Parsed read-request fields (FC 01/02/03/04).
 */
struct ReadRequest {
    uint16_t startAddress;   ///< First address to read.
    uint16_t quantity;       ///< Number of items to read.
};

/**
 * @brief Parse a read-request PDU (FC 01/02/03/04).
 * @param pdu Raw PDU bytes: [FC][StartAddr_Hi][StartAddr_Lo][Qty_Hi][Qty_Lo].
 * @return Parsed ReadRequest.
 */
inline ReadRequest parseReadRequest(const Bytes& pdu)
{
    if (pdu.size() < 5)
        throw std::runtime_error("parseReadRequest: PDU too short");
    ReadRequest req;
    req.startAddress = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    req.quantity     = (static_cast<uint16_t>(pdu[3]) << 8) | pdu[4];
    if (req.quantity == 0 || req.quantity > 2000)
        throw std::runtime_error("parseReadRequest: quantity out of range (1-2000)");
    return req;
}

/**
 * @struct WriteSingleCoilReq
 * @brief Parsed Write Single Coil request fields (FC 05).
 */
struct WriteSingleCoilReq {
    uint16_t address;   ///< Coil address.
    bool     value;     ///< Coil state to set.
};

/**
 * @brief Parse a Write Single Coil request PDU (FC 05).
 * @param pdu Raw PDU bytes: [FC][Addr_Hi][Addr_Lo][Value_Hi][Value_Lo].
 * @return Parsed WriteSingleCoilReq.
 */
inline WriteSingleCoilReq parseWriteSingleCoil(const Bytes& pdu)
{
    if (pdu.size() < 5)
        throw std::runtime_error("parseWriteSingleCoil: PDU too short");
    WriteSingleCoilReq r;
    r.address = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    r.value   = (pdu[3] == 0xFF);  // 0xFF00 = ON, 0x0000 = OFF
    return r;
}

/**
 * @struct WriteSingleRegReq
 * @brief Parsed Write Single Register request fields (FC 06).
 */
struct WriteSingleRegReq {
    uint16_t address;   ///< Register address.
    uint16_t value;     ///< Register value to set.
};

/**
 * @brief Parse a Write Single Register request PDU (FC 06).
 * @param pdu Raw PDU bytes: [FC][Addr_Hi][Addr_Lo][Value_Hi][Value_Lo].
 * @return Parsed WriteSingleRegReq.
 */
inline WriteSingleRegReq parseWriteSingleRegister(const Bytes& pdu)
{
    if (pdu.size() < 5)
        throw std::runtime_error("parseWriteSingleRegister: PDU too short");
    WriteSingleRegReq r;
    r.address = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    r.value   = (static_cast<uint16_t>(pdu[3]) << 8) | pdu[4];
    return r;
}

/**
 * @struct WriteMultipleCoilsReq
 * @brief Parsed Write Multiple Coils request fields (FC 0F).
 */
struct WriteMultipleCoilsReq {
    uint16_t          startAddress;   ///< Starting coil address.
    std::vector<bool> coils;          ///< Coil values to write.
};

/**
 * @brief Parse a Write Multiple Coils request PDU (FC 0F).
 * @param pdu Raw PDU bytes: [FC][StartAddr(2)][Qty(2)][ByteCount(1)][CoilData(N)].
 * @return Parsed WriteMultipleCoilsReq.
 */
inline WriteMultipleCoilsReq parseWriteMultipleCoils(const Bytes& pdu)
{
    if (pdu.size() < 6)
        throw std::runtime_error("parseWriteMultipleCoils: PDU too short");
    WriteMultipleCoilsReq r;
    r.startAddress = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    uint16_t qty   = (static_cast<uint16_t>(pdu[3]) << 8) | pdu[4];
    if (qty == 0 || qty > 1968)
        throw std::runtime_error("parseWriteMultipleCoils: quantity out of range (1-1968)");
    uint8_t byteCount = pdu[5];
    if (pdu.size() < 6 + byteCount)
        throw std::runtime_error("parseWriteMultipleCoils: data truncated");
    // pdu[5] = byteCount, pdu[6..] = coil data bytes
    for (uint16_t i = 0; i < qty; ++i) {
        uint8_t byteIdx = static_cast<uint8_t>(i / 8);
        uint8_t bitIdx  = static_cast<uint8_t>(i % 8);
        r.coils.push_back((pdu[6 + byteIdx] >> bitIdx) & 0x01);
    }
    return r;
}

/**
 * @struct WriteMultipleRegsReq
 * @brief Parsed Write Multiple Registers request fields (FC 10).
 */
struct WriteMultipleRegsReq {
    uint16_t               startAddress;   ///< Starting register address.
    std::vector<uint16_t>  registers;      ///< Register values to write.
};

/**
 * @brief Parse a Write Multiple Registers request PDU (FC 10).
 * @param pdu Raw PDU bytes: [FC][StartAddr(2)][Qty(2)][ByteCount(1)][RegData(N*2)].
 * @return Parsed WriteMultipleRegsReq.
 */
inline WriteMultipleRegsReq parseWriteMultipleRegisters(const Bytes& pdu)
{
    if (pdu.size() < 6)
        throw std::runtime_error("parseWriteMultipleRegisters: PDU too short");
    WriteMultipleRegsReq r;
    r.startAddress = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    uint16_t qty   = (static_cast<uint16_t>(pdu[3]) << 8) | pdu[4];
    if (qty == 0 || qty > 123)
        throw std::runtime_error("parseWriteMultipleRegisters: quantity out of range (1-123)");
    uint8_t byteCount = pdu[5];
    if (pdu.size() < 6 + byteCount)
        throw std::runtime_error("parseWriteMultipleRegisters: data truncated");
    // pdu[5] = byteCount, pdu[6..] = register data (big-endian pairs)
    for (uint16_t i = 0; i < qty; ++i) {
        size_t offset = 6 + static_cast<size_t>(i) * 2;
        uint16_t val = (static_cast<uint16_t>(pdu[offset]) << 8) | pdu[offset + 1];
        r.registers.push_back(val);
    }
    return r;
}

/**
 * @brief Parse a Read Coils/Discrete Inputs response PDU (FC 01/02).
 * @param pdu Raw response PDU bytes: [FC][ByteCount(1)][CoilData(N)].
 * @return Vector of decoded boolean values.
 */
inline std::vector<bool> parseReadCoilsResponse(const Bytes& pdu)
{
    if (pdu.size() < 2)
        throw std::runtime_error("parseReadCoilsResponse: PDU too short");
    uint8_t byteCount = pdu[1];
    if (pdu.size() < 2 + byteCount)
        throw std::runtime_error("parseReadCoilsResponse: data truncated");
    std::vector<bool> out;
    for (uint8_t i = 0; i < byteCount; ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            out.push_back((pdu[2 + i] >> bit) & 0x01);
        }
    }
    return out;
}

/**
 * @brief Parse a Read Registers response PDU (FC 03/04).
 * @param pdu Raw response PDU bytes: [FC][ByteCount(1)][RegData(N*2)].
 * @return Vector of decoded 16-bit register values.
 */
inline std::vector<uint16_t> parseReadRegistersResponse(const Bytes& pdu)
{
    if (pdu.size() < 2)
        throw std::runtime_error("parseReadRegistersResponse: PDU too short");
    uint8_t byteCount = pdu[1];
    if (pdu.size() < 2 + byteCount)
        throw std::runtime_error("parseReadRegistersResponse: data truncated");
    std::vector<uint16_t> out;
    for (uint8_t i = 0; i < byteCount; i += 2) {
        uint16_t val = (static_cast<uint16_t>(pdu[2 + i]) << 8) | pdu[2 + i + 1];
        out.push_back(val);
    }
    return out;
}

/**
 * @struct MaskWriteResp
 * @brief Parsed Mask Write Register echo response (FC 22).
 */
struct MaskWriteResp {
    uint16_t address;   ///< Echoed register address.
    uint16_t andMask;   ///< Echoed AND mask.
    uint16_t orMask;    ///< Echoed OR mask.
};

/**
 * @brief Parse a Mask Write Register response PDU (FC 22).
 * @param pdu Raw response PDU: [FC][Addr(2)][AndMask(2)][OrMask(2)] (echo of request).
 * @return Parsed echo fields. Caller compares against the request to verify.
 */
inline MaskWriteResp parseMaskWriteResponse(const Bytes& pdu)
{
    if (pdu.size() < 7)
        throw std::runtime_error("parseMaskWriteResponse: PDU too short");
    MaskWriteResp r;
    r.address = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    r.andMask = (static_cast<uint16_t>(pdu[3]) << 8) | pdu[4];
    r.orMask  = (static_cast<uint16_t>(pdu[5]) << 8) | pdu[6];
    return r;
}

/**
 * @brief Parse a Read/Write Multiple Registers response PDU (FC 23).
 * @param pdu Raw response PDU: [FC][ByteCount(1)][ReadData(N*2)] big-endian.
 * @return Vector of decoded 16-bit register values from the read portion.
 */
inline std::vector<uint16_t> parseReadWriteMultipleResponse(const Bytes& pdu)
{
    if (pdu.size() < 2)
        throw std::runtime_error("parseReadWriteMultipleResponse: PDU too short");
    uint8_t byteCount = pdu[1];
    if (pdu.size() < 2 + byteCount)
        throw std::runtime_error("parseReadWriteMultipleResponse: data truncated");
    std::vector<uint16_t> out;
    for (uint8_t i = 0; i < byteCount; i += 2) {
        uint16_t val = (static_cast<uint16_t>(pdu[2 + i]) << 8) | pdu[2 + i + 1];
        out.push_back(val);
    }
    return out;
}

/**
 * @brief Parse a Read FIFO Queue response PDU (FC 24).
 * @param pdu Raw response PDU:
 *            [FC][ByteCount(2)][FifoCount(2)][FifoData(FifoCount*2)] big-endian.
 *            Note ByteCount here is **2 bytes** (uint16) — different from FC 03/04.
 * @return Vector of decoded 16-bit FIFO values (up to 31 per Modbus spec).
 */
inline std::vector<uint16_t> parseReadFifoQueueResponse(const Bytes& pdu)
{
    if (pdu.size() < 5)
        throw std::runtime_error("parseReadFifoQueueResponse: PDU too short");
    uint16_t byteCount = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    uint16_t fifoCount = (static_cast<uint16_t>(pdu[3]) << 8) | pdu[4];
    if (fifoCount > 31)
        throw std::runtime_error("parseReadFifoQueueResponse: fifoCount > 31");
    if (pdu.size() < 3u + byteCount)
        throw std::runtime_error("parseReadFifoQueueResponse: data truncated");
    std::vector<uint16_t> out;
    for (uint16_t i = 0; i < fifoCount; ++i) {
        size_t offset = 5 + static_cast<size_t>(i) * 2;
        uint16_t val = (static_cast<uint16_t>(pdu[offset]) << 8) | pdu[offset + 1];
        out.push_back(val);
    }
    return out;
}

/** @} */

} // namespace Modbus
