#pragma once
/**
 * @file asn1.hpp
 * @brief ASN.1 BER Encoder / Decoder.
 *
 * Supports the following ASN.1 universal types:
 *   - BOOLEAN
 *   - INTEGER (64-bit signed)
 *   - OCTET STRING
 *   - NULL
 *   - SEQUENCE (constructed)
 *
 * All Modbus data (addresses, register values, coil arrays) is wrapped in
 * these primitives before being placed inside a Modbus TCP PDU.
 */

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace ASN1 {

/** @brief Convenience alias for a byte buffer. */
using Bytes = std::vector<uint8_t>;

/**
 * @brief Universal class tags (primitive unless noted).
 */
enum class Tag : uint8_t {
    BOOLEAN      = 0x01,
    INTEGER      = 0x02,
    OCTET_STRING = 0x04,
    NULL_TYPE    = 0x05,
    SEQUENCE     = 0x30,   // constructed  (0x10 | 0x20)
    SET          = 0x31,   // constructed  (0x11 | 0x20)
};

/**
 * @brief Encode a BER length field.
 * @param len The length value to encode.
 * @return Byte sequence representing the BER-encoded length.
 *
 * Uses short form (1 byte) for lengths < 128, long form otherwise.
 */
inline Bytes encodeLength(size_t len)
{
    Bytes out;
    if (len < 0x80) {
        out.push_back(static_cast<uint8_t>(len));
    } else {
        // Long form: number of subsequent octets, then the length bytes
        Bytes lenBytes;
        size_t tmp = len;
        while (tmp > 0) {
            lenBytes.push_back(static_cast<uint8_t>(tmp & 0xFF));
            tmp >>= 8;
        }
        std::reverse(lenBytes.begin(), lenBytes.end());
        out.push_back(static_cast<uint8_t>(0x80 | lenBytes.size()));
        out.insert(out.end(), lenBytes.begin(), lenBytes.end());
    }
    return out;
}

/**
 * @brief Decode a BER length field from a byte buffer.
 * @param data The byte buffer to read from.
 * @param offset Starting position within the buffer.
 * @return A pair of (decoded_length, bytes_consumed_by_length_field).
 * @throws std::runtime_error If the length field is truncated or uses
 *         unsupported indefinite-length encoding.
 */
inline std::pair<size_t, size_t> decodeLength(const Bytes& data, size_t offset)
{
    if (offset >= data.size())
        throw std::runtime_error("ASN1: truncated length field");

    uint8_t first = data[offset];
    if ((first & 0x80) == 0) {
        return {first, 1};
    }
    size_t numBytes = first & 0x7F;
    if (numBytes == 0)
        throw std::runtime_error("ASN1: indefinite length not supported");
    if (offset + 1 + numBytes > data.size())
        throw std::runtime_error("ASN1: truncated long-form length");

    size_t length = 0;
    for (size_t i = 0; i < numBytes; ++i) {
        length = (length << 8) | data[offset + 1 + i];
    }
    return {length, 1 + numBytes};
}

/**
 * @class Encoder
 * @brief Streaming ASN.1 BER encoder.
 *
 * Builds a byte buffer incrementally.  Nested SEQUENCE elements are
 * supported via beginSequence() / endSequence() pairs; the length field
 * is back-patched when endSequence() is called.
 */
class Encoder {
public:
    /**
     * @brief Encode a BOOLEAN value.
     * @param value The boolean to encode (true -> 0xFF, false -> 0x00).
     */
    void encodeBoolean(bool value)
    {
        buffer_.push_back(static_cast<uint8_t>(Tag::BOOLEAN));
        buffer_.push_back(0x01);
        buffer_.push_back(value ? 0xFF : 0x00);
    }

    /**
     * @brief Encode a signed INTEGER using minimal two's-complement form.
     * @param value The 64-bit signed integer to encode.
     */
    void encodeInteger(int64_t value)
    {
        // Minimal two's-complement encoding (build reversed, then flip)
        Bytes content;
        int64_t tmp = value;
        do {
            content.push_back(static_cast<uint8_t>(tmp & 0xFF));
            tmp >>= 8;
        } while (tmp != 0 && tmp != -1);
        std::reverse(content.begin(), content.end());

        // Ensure sign bit is not misleading
        if (value >= 0 && (content[0] & 0x80))
            content.insert(content.begin(), 0x00);
        else if (value < 0 && !(content[0] & 0x80))
            content.insert(content.begin(), 0xFF);

        buffer_.push_back(static_cast<uint8_t>(Tag::INTEGER));
        Bytes lenBytes = encodeLength(content.size());
        buffer_.insert(buffer_.end(), lenBytes.begin(), lenBytes.end());
        buffer_.insert(buffer_.end(), content.begin(), content.end());
    }

    /**
     * @brief Encode a 16-bit unsigned integer as an ASN.1 INTEGER.
     * @param value The uint16 value to encode.
     */
    void encodeUInt16(uint16_t value)
    {
        encodeInteger(static_cast<int64_t>(value));
    }

    /**
     * @brief Encode an OCTET STRING.
     * @param data Raw bytes to encode as the string content.
     */
    void encodeOctetString(const Bytes& data)
    {
        buffer_.push_back(static_cast<uint8_t>(Tag::OCTET_STRING));
        Bytes lenBytes = encodeLength(data.size());
        buffer_.insert(buffer_.end(), lenBytes.begin(), lenBytes.end());
        buffer_.insert(buffer_.end(), data.begin(), data.end());
    }

    /**
     * @brief Encode a NULL value.
     */
    void encodeNull()
    {
        buffer_.push_back(static_cast<uint8_t>(Tag::NULL_TYPE));
        buffer_.push_back(0x00);
    }

    /**
     * @brief Begin a constructed SEQUENCE.
     *
     * Must be paired with a matching endSequence() call. Nested sequences
     * are supported.
     *
     * @code
     *   enc.beginSequence();
     *   enc.encodeInteger(42);
     *   enc.endSequence();
     * @endcode
     */
    void beginSequence()
    {
        // Push a placeholder tag; remember position to fill in length later
        buffer_.push_back(static_cast<uint8_t>(Tag::SEQUENCE));
        seqStack_.push_back(buffer_.size()); // position AFTER the tag byte
    }

    /**
     * @brief End a constructed SEQUENCE and back-patch the length field.
     * @throws std::logic_error If no matching beginSequence() was called.
     */
    void endSequence()
    {
        if (seqStack_.empty())
            throw std::logic_error("ASN1::Encoder: endSequence without beginSequence");

        size_t contentStart = seqStack_.back();
        seqStack_.pop_back();

        // The content of the SEQUENCE sits from contentStart to buffer_.end(),
        // but we haven't written a length yet — we need to insert it.
        size_t contentLen = buffer_.size() - contentStart;
        Bytes lenBytes = encodeLength(contentLen);
        buffer_.insert(buffer_.begin() + contentStart,
                        lenBytes.begin(), lenBytes.end());
    }

    /** @brief Return a const reference to the accumulated byte buffer. */
    const Bytes& getBytes() const { return buffer_; }
    /** @brief Reset the encoder, discarding all buffered data. */
    void clear() { buffer_.clear(); seqStack_.clear(); }

private:
    Bytes              buffer_;          ///< Accumulated encoded bytes.
    std::vector<size_t> seqStack_;       ///< Stack of content-start positions for open sequences.
};

/**
 * @class Decoder
 * @brief Streaming ASN.1 BER decoder.
 *
 * Reads from an immutable byte buffer.  The caller advances the internal
 * cursor by calling decode*() methods in the order the elements appear.
 */
class Decoder {
public:
    /**
     * @brief Construct a decoder over the given byte buffer.
     * @param data The BER-encoded data to decode (must outlive the Decoder).
     */
    explicit Decoder(const Bytes& data) : data_(data), pos_(0) {}

    /** @brief Return true if there is more data to read. */
    bool hasMore() const { return pos_ < data_.size(); }
    /** @brief Return the current read position. */
    size_t pos()  const { return pos_; }

    /**
     * @brief Peek at the next tag byte without consuming it.
     * @return The raw tag byte value.
     * @throws std::runtime_error If no data remains.
     */
    uint8_t peekTag() const
    {
        if (pos_ >= data_.size())
            throw std::runtime_error("ASN1: no data to peek");
        return data_[pos_];
    }

    /**
     * @brief Decode a BOOLEAN value.
     * @return The decoded boolean.
     * @throws std::runtime_error If the tag or length is invalid.
     */
    bool decodeBoolean()
    {
        expectTag(Tag::BOOLEAN);
        size_t len = readLength();
        if (len != 1)
            throw std::runtime_error("ASN1: BOOLEAN must have length 1");
        return data_[pos_++] != 0x00;
    }

    /**
     * @brief Decode a signed INTEGER (up to 64 bits).
     * @return The decoded 64-bit signed integer.
     * @throws std::runtime_error If the tag or length is invalid.
     */
    int64_t decodeInteger()
    {
        expectTag(Tag::INTEGER);
        size_t len = readLength();
        if (len == 0 || len > 8)
            throw std::runtime_error("ASN1: INTEGER length out of range");

        // Sign-extend from first byte
        int64_t value = (data_[pos_] & 0x80) ? -1 : 0;
        for (size_t i = 0; i < len; ++i)
            value = (value << 8) | data_[pos_++];
        return value;
    }

    /**
     * @brief Decode an INTEGER and return it as a uint16_t.
     * @return The decoded value clamped to [0, 0xFFFF].
     * @throws std::runtime_error If the value is out of uint16 range.
     */
    uint16_t decodeUInt16()
    {
        int64_t v = decodeInteger();
        if (v < 0 || v > 0xFFFF)
            throw std::runtime_error("ASN1: INTEGER out of uint16 range");
        return static_cast<uint16_t>(v);
    }

    /**
     * @brief Decode an OCTET STRING.
     * @return The raw bytes of the string content.
     * @throws std::runtime_error If the data is truncated.
     */
    Bytes decodeOctetString()
    {
        expectTag(Tag::OCTET_STRING);
        size_t len = readLength();
        if (pos_ + len > data_.size())
            throw std::runtime_error("ASN1: OCTET STRING truncated");
        Bytes out(data_.begin() + pos_, data_.begin() + pos_ + len);
        pos_ += len;
        return out;
    }

    /**
     * @brief Decode a NULL value.
     * @throws std::runtime_error If the length is not zero.
     */
    void decodeNull()
    {
        expectTag(Tag::NULL_TYPE);
        size_t len = readLength();
        if (len != 0)
            throw std::runtime_error("ASN1: NULL must have length 0");
    }

    /**
     * @brief Begin reading a constructed SEQUENCE.
     * @return The content length in bytes.
     * @throws std::runtime_error If the data is truncated.
     */
    size_t beginSequence()
    {
        expectTag(Tag::SEQUENCE);
        size_t len = readLength();
        if (pos_ + len > data_.size())
            throw std::runtime_error("ASN1: SEQUENCE content truncated");
        seqEndStack_.push_back(pos_ + len);
        return len;
    }

    /**
     * @brief Finish reading a SEQUENCE, skipping any unconsumed trailing bytes.
     * @throws std::logic_error If no matching beginSequence() was called.
     */
    void endSequence()
    {
        if (seqEndStack_.empty())
            throw std::logic_error("ASN1::Decoder: endSequence without beginSequence");
        size_t end = seqEndStack_.back();
        seqEndStack_.pop_back();
        pos_ = end;  // skip any unconsumed tail bytes inside the sequence
    }

    /**
     * @brief Check whether the cursor has reached the end of the current SEQUENCE.
     * @return True if all content of the innermost SEQUENCE has been consumed.
     */
    bool atSequenceEnd() const
    {
        if (seqEndStack_.empty()) return !hasMore();
        return pos_ >= seqEndStack_.back();
    }

private:
    const Bytes&        data_;           ///< Reference to the source byte buffer.
    size_t              pos_;            ///< Current read position.
    std::vector<size_t> seqEndStack_;    ///< Stack of expected end-positions for open sequences.

    void expectTag(Tag tag)
    {
        if (pos_ >= data_.size())
            throw std::runtime_error("ASN1: unexpected end of data");
        uint8_t got = data_[pos_++];
        if (got != static_cast<uint8_t>(tag)) {
            char buf[80];
            std::snprintf(buf, sizeof(buf),
                "ASN1: unexpected tag 0x%02X, expected 0x%02X",
                got, static_cast<uint8_t>(tag));
            throw std::runtime_error(buf);
        }
    }

    size_t readLength()
    {
        auto [len, consumed] = decodeLength(data_, pos_);
        pos_ += consumed;
        if (pos_ + len > data_.size())
            throw std::runtime_error("ASN1: value truncated");
        return len;
    }
};

} // namespace ASN1
