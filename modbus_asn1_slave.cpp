/**
 * @file modbus_asn1_slave.cpp
 * @brief Modbus Slave implementation using Boost.Asio (TCP).
 *
 * Implements async TCP accept with per-client sessions for every
 * supported Modbus function code.
 */

#include "modbus_asn1_slave.hpp"
#include <mutex>
#include <sstream>

namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace Modbus {

// ===========================================================================
// TcpSession
// ===========================================================================
TcpSession::TcpSession(tcp::socket socket, ProcessFn processFn, LogFn logFn)
    : socket_(std::move(socket)),
      processFn_(std::move(processFn)),
      logFn_(std::move(logFn)),
      headerBuf_(MBAP_SIZE)
{}

void TcpSession::start()
{
    if (logFn_) {
        auto ep = socket_.remote_endpoint();
        logFn_("[Slave] Client connected: " +
               ep.address().to_string() + ":" + std::to_string(ep.port()));
    }
    readHeader();
}

void TcpSession::readHeader()
{
    auto self = shared_from_this();
    asio::async_read(
        socket_, asio::buffer(headerBuf_),
        [this, self](boost::system::error_code ec, std::size_t /*bytes*/) {
            if (ec) {
                if (logFn_)
                    logFn_("[Slave] Client disconnected (" + ec.message() + ")");
                return;
            }

            MBAPHeader mbap = MBAPHeader::fromBytes(headerBuf_);
            int pduLen = static_cast<int>(mbap.length) - 1;
            if (pduLen <= 0 || pduLen > static_cast<int>(MAX_PDU_SIZE)) {
                if (logFn_)
                    logFn_("[Slave] Invalid PDU length: " + std::to_string(pduLen));
                return;
            }
            readPdu(mbap, static_cast<size_t>(pduLen));
        });
}

void TcpSession::readPdu(MBAPHeader mbap, size_t pduLen)
{
    auto pduBuf = std::make_shared<ASN1::Bytes>(pduLen);
    auto self   = shared_from_this();

    asio::async_read(
        socket_, asio::buffer(*pduBuf),
        [this, self, mbap, pduBuf](boost::system::error_code ec, std::size_t /*bytes*/) {
            if (ec) {
                if (logFn_)
                    logFn_("[Slave] recv PDU failed: " + ec.message());
                return;
            }

            {
                std::ostringstream oss;
                oss << "[Slave] RX(TCP) FC=0x" << std::hex
                    << static_cast<int>((*pduBuf)[0])
                    << "  PDU bytes=" << std::dec << pduBuf->size();
                if (logFn_) logFn_(oss.str());
            }

            ASN1::Bytes respPdu = processFn_(*pduBuf);

            MBAPHeader respHdr;
            respHdr.transactionId = mbap.transactionId;
            respHdr.protocolId    = PROTOCOL_ID;
            respHdr.length        = static_cast<uint16_t>(1 + respPdu.size());
            respHdr.unitId        = mbap.unitId;

            ASN1::Bytes respFrame = respHdr.toBytes();
            respFrame.insert(respFrame.end(), respPdu.begin(), respPdu.end());

            sendResponse(respFrame);
        });
}

void TcpSession::sendResponse(const ASN1::Bytes& frame)
{
    auto buf  = std::make_shared<ASN1::Bytes>(frame);
    auto self = shared_from_this();

    asio::async_write(
        socket_, asio::buffer(*buf),
        [this, self, buf](boost::system::error_code ec, std::size_t /*bytes*/) {
            if (ec) {
                if (logFn_)
                    logFn_("[Slave] send failed: " + ec.message());
                return;
            }
            // Continue reading next frame from this client
            readHeader();
        });
}

// ===========================================================================
// Slave
// ===========================================================================

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
Slave::Slave(uint8_t unitId, uint16_t port, LogCb logCb)
    : unitId_(unitId), port_(port), logCb_(std::move(logCb))
{
    coils_.fill(false);
    discreteInputs_.fill(false);
    holdingRegs_.fill(0);
    inputRegs_.fill(0);
    setupHandlers();
}

Slave::~Slave()
{
    stop();
}

// ---------------------------------------------------------------------------
// start / stop
// ---------------------------------------------------------------------------
bool Slave::start()
{
    if (running_) return true;

    boost::system::error_code ec;

    // TCP acceptor
    tcpAcceptor_ = std::make_unique<tcp::acceptor>(
        ioc_, tcp::endpoint(tcp::v4(), port_));
    tcpAcceptor_->set_option(tcp::acceptor::reuse_address(true), ec);
    if (ec) {
        log("[Slave] TCP reuse_address failed: " + ec.message());
        return false;
    }
    startTcpAccept();
    log("[Slave] Listening TCP on port " + std::to_string(port_));

    running_ = true;
    serverThread_ = std::thread([this]() { ioc_.run(); });
    return true;
}

void Slave::stop()
{
    if (!running_) return;
    running_ = false;

    ioc_.stop();

    if (serverThread_.joinable())
        serverThread_.join();

    boost::system::error_code ec;
    if (tcpAcceptor_ && tcpAcceptor_->is_open())
        tcpAcceptor_->close(ec);

    tcpAcceptor_.reset();

    log("[Slave] Stopped");
}

// ---------------------------------------------------------------------------
// TCP async accept
// ---------------------------------------------------------------------------
void Slave::startTcpAccept()
{
    tcpAcceptor_->async_accept(
        [this](boost::system::error_code ec, tcp::socket socket) {
            if (ec) {
                if (running_)
                    log("[Slave] accept() failed: " + ec.message());
                return;
            }

            auto session = std::make_shared<TcpSession>(
                std::move(socket),
                [this](const ASN1::Bytes& pdu) { return processRequest(pdu); },
                [this](const std::string& msg) { log(msg); });
            session->start();

            // Accept next client
            startTcpAccept();
        });
}

// ---------------------------------------------------------------------------
// Data model accessors
// ---------------------------------------------------------------------------
void Slave::setCoil(uint16_t a, bool v) {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Coil address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    coils_[a] = v;
}
bool Slave::getCoil(uint16_t a) const {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Coil address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    return coils_[a];
}
void Slave::setDiscreteInput(uint16_t a, bool v) {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Discrete input address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    discreteInputs_[a] = v;
}
bool Slave::getDiscreteInput(uint16_t a) const {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Discrete input address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    return discreteInputs_[a];
}
void Slave::setHoldingRegister(uint16_t a, uint16_t v) {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Holding register address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    holdingRegs_[a] = v;
}
uint16_t Slave::getHoldingRegister(uint16_t a) const {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Holding register address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    return holdingRegs_[a];
}
void Slave::setInputRegister(uint16_t a, uint16_t v) {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Input register address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    inputRegs_[a] = v;
}
uint16_t Slave::getInputRegister(uint16_t a) const {
    if (a >= DATA_MODEL_SIZE) throw std::out_of_range("Input register address out of range");
    std::lock_guard<std::mutex> lk(dataMutex_);
    return inputRegs_[a];
}

// ---------------------------------------------------------------------------
// setupHandlers  -- register FC handlers with the ASN.1 TLV dispatcher
// ---------------------------------------------------------------------------
void Slave::setupHandlers()
{
    // FC 01 - Read Coils
    dispatcher_.registerHandler(FC::READ_COILS,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseReadPayload(payload);
            if (req.startAddress + req.quantity > DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            std::vector<bool> vals(req.quantity);
            for (uint16_t i = 0; i < req.quantity; ++i)
                vals[i] = coils_[req.startAddress + i];
            return buildCoilsResponsePayload(vals);
        });

    // FC 02 - Read Discrete Inputs
    dispatcher_.registerHandler(FC::READ_DISCRETE_INPUTS,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseReadPayload(payload);
            if (req.startAddress + req.quantity > DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            std::vector<bool> vals(req.quantity);
            for (uint16_t i = 0; i < req.quantity; ++i)
                vals[i] = discreteInputs_[req.startAddress + i];
            return buildCoilsResponsePayload(vals);
        });

    // FC 03 - Read Holding Registers
    dispatcher_.registerHandler(FC::READ_HOLDING_REGISTERS,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseReadPayload(payload);
            if (req.startAddress + req.quantity > DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            std::vector<uint16_t> vals(req.quantity);
            for (uint16_t i = 0; i < req.quantity; ++i)
                vals[i] = holdingRegs_[req.startAddress + i];
            return buildRegistersResponsePayload(vals);
        });

    // FC 04 - Read Input Registers
    dispatcher_.registerHandler(FC::READ_INPUT_REGISTERS,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseReadPayload(payload);
            if (req.startAddress + req.quantity > DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            std::vector<uint16_t> vals(req.quantity);
            for (uint16_t i = 0; i < req.quantity; ++i)
                vals[i] = inputRegs_[req.startAddress + i];
            return buildRegistersResponsePayload(vals);
        });

    // FC 05 - Write Single Coil
    dispatcher_.registerHandler(FC::WRITE_SINGLE_COIL,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseWriteSingleCoilPayload(payload);
            if (req.address >= DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            coils_[req.address] = req.value;
            return payload;  // echo back
        });

    // FC 06 - Write Single Register
    dispatcher_.registerHandler(FC::WRITE_SINGLE_REGISTER,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseWriteSingleRegPayload(payload);
            if (req.address >= DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            holdingRegs_[req.address] = req.value;
            return payload;  // echo back
        });

    // FC 0F - Write Multiple Coils
    dispatcher_.registerHandler(FC::WRITE_MULTIPLE_COILS,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseWriteMultipleCoilsPayload(payload);
            if (req.startAddress + req.coils.size() > DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            for (size_t i = 0; i < req.coils.size(); ++i)
                coils_[req.startAddress + i] = req.coils[i];
            return buildWriteMultipleResponsePayload(
                req.startAddress, static_cast<uint16_t>(req.coils.size()));
        });

    // FC 10 - Write Multiple Registers
    dispatcher_.registerHandler(FC::WRITE_MULTIPLE_REGISTERS,
        [this](FC /*fc*/, const Bytes& payload) -> Bytes {
            auto req = parseWriteMultipleRegsPayload(payload);
            if (req.startAddress + req.registers.size() > DATA_MODEL_SIZE)
                throw ModbusException(ExCode::ILLEGAL_DATA_ADDRESS);
            for (size_t i = 0; i < req.registers.size(); ++i)
                holdingRegs_[req.startAddress + i] = req.registers[i];
            return buildWriteMultipleResponsePayload(
                req.startAddress, static_cast<uint16_t>(req.registers.size()));
        });
}

// ---------------------------------------------------------------------------
// processRequest  -- ASN.1 TLV encode/decode on every request/response
// ---------------------------------------------------------------------------
ASN1::Bytes Slave::processRequest(const ASN1::Bytes& pdu)
{
    if (pdu.empty())
        return buildExceptionResponse(FC::READ_COILS, ExCode::ILLEGAL_FUNCTION);

    try {
        // 1. Convert raw Modbus PDU → ASN.1 TLV  (exercises ASN1::Encoder)
        Bytes tlvRequest = pduToTlv(pdu);

        {
            FC fc = static_cast<FC>(pdu[0]);
            std::ostringstream oss;
            oss << "[Slave] PDU→TLV encode: FC=0x" << std::hex
                << static_cast<int>(static_cast<uint8_t>(fc))
                << "  TLV bytes=" << std::dec << tlvRequest.size();
            log(oss.str());
        }

        // 2. Dispatch via TLV path (ASN1::Decoder → handler → ASN1::Encoder)
        Bytes tlvResponse = dispatcher_.dispatch(tlvRequest);

        // 3. Convert ASN.1 TLV response → raw Modbus PDU (exercises ASN1::Decoder)
        Bytes resp = tlvToPdu(tlvResponse);

        {
            std::ostringstream oss;
            oss << "[Slave] TLV→PDU decode: resp PDU bytes=" << std::dec
                << resp.size();
            log(oss.str());
        }

        return resp;

    } catch (const ModbusException& e) {
        FC fc = static_cast<FC>(pdu[0]);
        return buildExceptionResponse(fc, e.code);
    } catch (const std::exception& e) {
        log(std::string("[Slave] Processing error: ") + e.what());
        FC fc = static_cast<FC>(pdu[0]);
        return buildExceptionResponse(fc, ExCode::SERVER_DEVICE_FAILURE);
    }
}

} // namespace Modbus
