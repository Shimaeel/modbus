#pragma once
/**
 * @file modbus_master.hpp
 * @brief Modbus Master (Client) — Layer 1 protocol logic for Modbus TCP.
 *
 * @details
 * Declares `Modbus::Master`, the synchronous Modbus TCP client. The class
 * owns no I/O; it holds a reference to a Layer 0 `Transport` and uses it
 * to push raw frames out and pull responses back. Per project rules, this
 * header **MUST NOT** include `<boost/asio.hpp>` — asio is confined to
 * Layer 0 (`master/transport.hpp/.cpp`).
 *
 * ### Where this layer sits
 * @dot
 * digraph layer1 {
 *   rankdir=TB;
 *   node [shape=box, style="rounded,filled"];
 *   App      [label="Application\n(main_master.cpp)", fillcolor="#e6f0ff"];
 *   FCFiles  [label="Per-FC files\n(functions/*.cpp)", fillcolor="#cfe2ff"];
 *   Master   [label="Modbus::Master\n(THIS FILE)", fillcolor="#ffe6b3"];
 *   Common   [label="modbus_common.hpp\n(buildXxx / parseXxx)", fillcolor="#fff2cc"];
 *   Trans    [label="Modbus::Transport", fillcolor="#ffd699"];
 *   App     -> Master;
 *   Master  -> FCFiles [dir=back];
 *   FCFiles -> Common  [label="build/parse"];
 *   FCFiles -> Master  [label="transaction()"];
 *   Master  -> Trans   [label="send/recv"];
 * }
 * @enddot
 *
 * ### Public API style
 * Each Modbus function code has its own one-method-per-FC entry point in
 * the public interface (e.g. `readHoldingRegisters`, `writeSingleCoil`).
 * The implementation of each method is split out into `functions/*.cpp`
 * to keep the FC-specific logic localised; they all funnel through the
 * shared `transaction()` primitive declared private below.
 *
 * @see Modbus::Transport     Layer 0 byte-level interface.
 * @see modbus_common.hpp     Shared PDU builders/parsers used by every FC method.
 */

#include "../modbus_common.hpp"
#include "transport.hpp"
#include <cstdint>
#include <string>

namespace Modbus {

/**
 * @class Master
 * @brief Synchronous Modbus master (client) — pure protocol layer.
 *
 * @details
 * Holds a non-owning reference to a `Transport`. Each public method:
 *   1. Builds a Modbus request PDU using helpers from `modbus_common.hpp`.
 *   2. Calls `transaction()` to attach the MBAP header, send the frame,
 *      and read+validate the response.
 *   3. Parses the response PDU back into typed data and returns it.
 *
 * The class is **non-copyable** because it logically wraps a single
 * conversation channel; copying would be ambiguous (two masters sharing
 * one transaction-id sequence and one socket).
 *
 * ### Threading
 * `Master` is **not thread-safe**. Concurrent calls on the same instance
 * will corrupt the transaction-ID stream and interleave bytes on the
 * socket. Use one `Master` per thread (each with its own `Transport`),
 * or wrap calls in an external mutex.
 *
 * ### Error handling
 * - **Transport failure:** rethrown by `Transport::send`/`recv` as
 *   `std::runtime_error`.
 * - **Modbus exception response** (e.g. Illegal Function): detected in
 *   `transaction()` and rethrown as `std::runtime_error("Modbus exception: <name>")`.
 * - **Transaction-ID mismatch:** rethrown as `std::runtime_error`.
 *
 * Callers must `try`/`catch`. A typed `ModbusException` exists in
 * `asn/modbus_asn1_tlv.hpp:143` for future migration.
 *
 * ### Usage
 * @code
 *   Modbus::TcpTransport t("192.168.0.2", 502, 5000, logger);
 *   Modbus::Master       m(t, /unitId/ 1, logger);
 *   if (!m.connect()) { ... }
 *
 *   auto regs = m.readHoldingRegisters(0, 20);   // FC 03
 *   auto coils = m.readCoils(0, 23);             // FC 01
 *   m.writeSingleCoil(7, true);                   // FC 05 — RB01 ON
 * @endcode
 */
class Master {
public:
    /**
     * @brief Construct a Modbus master bound to a transport.
     * @param transport Reference to a Layer 0 transport (must outlive the Master).
     * @param unitId    Modbus unit/slave ID for outgoing requests (default 1).
     * @param logCb     Optional logging callback (forwarded to log frames).
     *
     * @note The `transport` reference is **non-owning**. The caller is
     *       responsible for keeping the transport alive longer than the
     *       master, and for calling `disconnect()` when done.
     */
    explicit Master(Transport& transport,
                    uint8_t    unitId = 1,
                    LogCb      logCb  = nullptr);

    ~Master() = default;

    Master(const Master&) = delete;             ///< Non-copyable.
    Master& operator=(const Master&) = delete;  ///< Non-copyable.

    /** @brief Open the underlying transport. @return `true` on success. */
    bool connect()    { return transport_.connect(); }

    /** @brief Close the underlying transport. */
    void disconnect() { transport_.disconnect(); }

    /** @brief Whether the underlying transport is currently open. */
    bool isConnected() const { return transport_.isConnected(); }

    /** @brief Replace the logging callback at runtime. */
    void setLogCallback(LogCb cb) { logCb_ = std::move(cb); }

    /** @brief Change the Modbus unit/slave ID for subsequent requests. */
    void setUnitId(uint8_t id) { unitId_ = id; }

    /** @name Modbus function-code methods
     *
     *  Each method maps 1-to-1 to a Modbus function code. The
     *  per-method implementation lives in @c functions/<name>.cpp.
     *  All methods are synchronous: they block until either a complete
     *  reply is received or the underlying transport throws.
     *  @{
     */

    /**
     * @brief FC 01 — Read Coils. Returns 1-bit ON/OFF status of N coils.
     * @param startAddr 0-based starting coil address.
     * @param quantity  Number of coils to read (1..2000).
     * @return Vector of length @p quantity with each coil's state.
     */
    std::vector<bool>     readCoils(uint16_t startAddr, uint16_t quantity);

    /**
     * @brief FC 02 — Read Discrete Inputs (read-only contacts).
     * @param startAddr 0-based starting input address.
     * @param quantity  Number of inputs to read (1..2000).
     * @return Vector of length @p quantity with each input's state.
     */
    std::vector<bool>     readDiscreteInputs(uint16_t startAddr, uint16_t quantity);

    /**
     * @brief FC 03 — Read Holding Registers (16-bit, read/write area).
     * @param startAddr 0-based starting register address.
     * @param quantity  Number of registers to read (1..125 on SEL-735).
     * @return Vector of @p quantity 16-bit values (big-endian on the wire).
     */
    std::vector<uint16_t> readHoldingRegisters(uint16_t startAddr, uint16_t quantity);

    /**
     * @brief FC 04 — Read Input Registers (16-bit, read-only area).
     * @param startAddr 0-based starting register address.
     * @param quantity  Number of registers to read (1..125 on SEL-735).
     * @return Vector of @p quantity 16-bit values.
     * @note SEL relays often treat FC 04 ≡ FC 03 (verified on SEL-735).
     */
    std::vector<uint16_t> readInputRegisters(uint16_t startAddr, uint16_t quantity);

    /**
     * @brief FC 05 — Force a single coil ON or OFF.
     * @param address 0-based coil address.
     * @param value   `true` → coil set to 1; `false` → 0.
     * @return `true` if the slave's echo matches the request.
     */
    bool writeSingleCoil(uint16_t address, bool value);

    /**
     * @brief FC 06 — Preset a single 16-bit register.
     * @param address 0-based register address.
     * @param value   New 16-bit value to write.
     * @return `true` if the slave's echo matches the request.
     */
    bool writeSingleRegister(uint16_t address, uint16_t value);

    /**
     * @brief FC 15 — Write multiple coils.
     * @param startAddr 0-based starting coil address.
     * @param values    Boolean vector, one element per coil to write.
     * @return `true` on a successful (non-exception) response.
     * @note **Not supported on SEL-735** — will throw "Illegal Function".
     */
    bool writeMultipleCoils(uint16_t startAddr, const std::vector<bool>& values);

    /**
     * @brief FC 16 — Write multiple registers (block write).
     * @param startAddr 0-based starting register address.
     * @param values    16-bit vector, one element per register.
     * @return `true` on a successful (non-exception) response.
     */
    bool writeMultipleRegisters(uint16_t startAddr, const std::vector<uint16_t>& values);

    /**
     * @brief FC 22 — Mask write a register (atomic bit modification).
     * @param address 0-based register address.
     * @param andMask Bits to keep from the existing value.
     * @param orMask  Bits to forcibly set to 1.
     * @return `true` on success.
     * @note **Not supported on SEL-735.**
     */
    bool maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask);

    /**
     * @brief FC 23 — Read/Write Multiple Registers (combined transaction).
     * @param readAddr  Read start address.
     * @param readQty   Number of registers to read.
     * @param writeAddr Write start address.
     * @param writeRegs Values to write.
     * @return The read-back register values.
     * @note **Not supported on SEL-735.**
     */
    std::vector<uint16_t> readWriteMultipleRegisters(uint16_t readAddr,
                                                     uint16_t readQty,
                                                     uint16_t writeAddr,
                                                     const std::vector<uint16_t>& writeRegs);

    /**
     * @brief FC 24 — Read FIFO Queue.
     * @param pointerAddr Address of the FIFO pointer register.
     * @return Up to 31 16-bit values from the FIFO.
     * @note **Not supported on SEL-735.**
     */
    std::vector<uint16_t> readFifoQueue(uint16_t pointerAddr);
    /** @} */

private:
    /**
     * @brief Shared transaction primitive — one round-trip on the wire.
     * @param requestPdu PDU to send (FC byte + payload, no MBAP).
     * @return Response PDU (FC byte + payload, MBAP stripped).
     * @throws std::runtime_error on transport failure, transaction-ID
     *         mismatch, or a Modbus exception response.
     *
     * @details
     * 1. Builds a 7-byte MBAP header (transaction-id auto-incremented).
     * 2. Concatenates header + PDU into a single frame.
     * 3. Calls `Transport::send()` to push it onto the wire.
     * 4. Calls `Transport::recv(7)` for the response MBAP.
     * 5. Validates protocol id, length, and transaction id.
     * 6. Calls `Transport::recv(length-1)` for the PDU body.
     * 7. Detects exception responses (FC byte high bit set) and throws.
     */
    Bytes transaction(const Bytes& requestPdu);

    /** @brief Internal log helper — emits @p msg if a callback is installed. */
    void log(const std::string& msg) const { if (logCb_) logCb_(msg); }

    Transport& transport_;        ///< Non-owning reference to the Layer 0 transport.
    uint8_t    unitId_;           ///< Modbus unit/slave ID for outgoing requests.
    LogCb      logCb_;            ///< Optional log sink.
    uint16_t   transactionId_{0}; ///< Monotonic counter — incremented per transaction.
};

} // namespace Modbus
