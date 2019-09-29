#include "mtl/network/out_stream.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/detail/byte_order.hpp"
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {

OutStream::OutStream(uint16_t begin)
{
    buffer_ = SingletonBufferPool::getSingleton().allocBuffer(1024);
    begin_ = begin;
    size_ = 1024;
    cursor_ = buffer_.get() + begin;
}

OutStream::OutStream(const SharedBuffer& buffer, uint16_t size,
                     uint16_t begin)
    : buffer_(buffer), size_(size), begin_(begin)
{
    cursor_ = buffer_.get() + begin;
}

uint16_t OutStream::skip(uint16_t offset, int origin)
{
    uint16_t old = size();
    if (origin == kSkipBegin) {
        uint16_t data_size = size_ - begin_;
        if (offset > data_size) {
            offset = data_size;
        }
        cursor_ = reinterpret_cast<char*>(buffer_.get()) + begin_ + offset;
    } else if (origin == kSkipCurrent) {
        if (offset > left()) {
            offset = left();
        }
        cursor_ += offset;
    } else {
        uint16_t data_size = size_ - begin_;
        if (offset > data_size) {
            offset = data_size;
        }
        cursor_ = buffer_.get() + size_ - offset;
    }
    return old;
}

void OutStream::resize(uint16_t sz)
{
    if (sz > size_) {
        uint16_t new_size = sz + 1024 - sz % 1024;
        SharedBuffer buf = SingletonBufferPool::getSingleton().allocBuffer(new_size);
        memcpy(buf.get(), buffer_.get(), size_);
        cursor_ = buf.get() + size();
        buffer_ = buf;
        size_ = new_size;
    }
}

void OutStream::clear()
{
    buffer_.reset();
    size_ = 0;
    cursor_ = nullptr;
}

OutStream& OutStream::writeInt8(uint8_t val)
{
    writeBinary(reinterpret_cast<const char*>(&val), 1);
    return (*this);
}

OutStream& OutStream::writeInt16(uint16_t val)
{
    uint16_t v = HostToNetwork16(val);
    writeBinary(reinterpret_cast<const char*>(&v), 2);
    return (*this);
}

OutStream& OutStream::writeInt32(uint32_t val)
{
    uint32_t v = HostToNetwork32(val);
    writeBinary(reinterpret_cast<const char*>(&v), 4);
    return (*this);
}

OutStream& OutStream::writeInt64(uint64_t val)
{
    uint64_t v = HostToNetwork64(val);
    writeBinary(reinterpret_cast<const char*>(&v), 8);
    return (*this);
}

OutStream& OutStream::writeString(const std::string& val)
{
    uint16_t len = static_cast<uint16_t>(val.length());
    writeInt16(uint16_t(len));
    writeBinary(val.c_str(), len);
    return (*this);
}

OutStream& OutStream::writeBinary(const char* data, uint16_t data_size)
{
    if (left() < data_size) {
        resize(size_ + data_size);
    }
    memcpy(cursor_, data, data_size);
    cursor_ += data_size;
    return (*this);
}
} // namespace network
} // namespace mtl
