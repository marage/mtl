#include "mtl/network/in_stream.hpp"
#include "mtl/network/detail/byte_order.hpp"
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {

InStream::InStream()
    : size_(0), begin_(0), cursor_(0)
{
}

InStream::InStream(const SharedBuffer& buffer, uint16_t size,
                   uint16_t begin)
    : buffer_(buffer), size_(size), begin_(begin)
    , cursor_(begin)
{
}

uint16_t InStream::skip(uint16_t offset, int origin)
{
    uint16_t cursor = cursor_;
    uint16_t data_size = size_ - begin_;
    if (origin == kSkipBegin) {
        // begin
        if (offset > data_size) {
            offset = data_size;
        }
        cursor_ = begin_ + offset;
    } else if (origin == kSkipCurrent) {
        // cur
        if (offset > left()) {
            offset = left();
        }
        cursor_ += offset;
    } else {	// end
        if (offset > data_size) {
            offset = data_size;
        }
        cursor_ = size_ - offset;
    }
    return cursor;
}

void InStream::clear()
{
    buffer_.reset();
    size_ = 0;
    cursor_ = 0;
}

uint8_t InStream::readInt8()
{
    uint8_t val = 0;
    if (left() >= 1) {
        val = *reinterpret_cast<const uint8_t*>(data());
        cursor_ += 1;
    }
    return val;
}

uint16_t InStream::readInt16()
{
    uint16_t val = 0;
    if (left() >= 2) {
        uint16_t v = 0;
        readBinary(reinterpret_cast<char*>(&v), 2);
        val = NetworkToHost16(v);
    }
    return val;
}

uint32_t InStream::readInt32()
{
    uint32_t val = 0;
    if (left() >= 4) {
        uint32_t v = 0;
        readBinary(reinterpret_cast<char*>(&v), 4);
        val = NetworkToHost32(v);
    }
    return val;
}

uint64_t InStream::readInt64()
{
    uint64_t val = 0;
    if (left() >= 8) {
        uint64_t v = 0;
        readBinary(reinterpret_cast<char*>(&v), 8);
        val = NetworkToHost64(v);
    }
    return val;
}

std::string InStream::readString(uint16_t max_length)
{
    std::string s;
    uint16_t len = readInt16();
    if (len > 0) {
        uint16_t bytes_to_read = left();
        uint16_t bytes_discarded = 0;
        if (len < bytes_to_read)
            bytes_to_read = len;
        if (max_length > 0 && max_length < bytes_to_read) {
            bytes_discarded = bytes_to_read - max_length;
            bytes_to_read = max_length;
        }
        if (bytes_to_read <= 1024) {
            char tmp[1024];
            readBinary(tmp, bytes_to_read);
            s.assign(tmp, bytes_to_read);
        } else {
            char* p = new char[bytes_to_read];
            readBinary(p, bytes_to_read);
            s.assign(p, bytes_to_read);
            delete []p;
        }
        cursor_ += bytes_discarded;
    }
    return s;
}

void InStream::readBinary(char* buf, uint16_t size)
{
    if (size > left()) {
        size = left();
    }
    memcpy(buf, data(), size);
    cursor_ += size;
}
} // namespace network
} // namespace mtl
