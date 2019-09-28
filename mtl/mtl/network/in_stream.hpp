#ifndef MTL_NETWORK_IN_STREAM_HPP
#define MTL_NETWORK_IN_STREAM_HPP
#include "mtl/mtl.hpp"
#include "shared_buffer.hpp"

namespace mtl {
namespace network {

class MTL_EXPORT InStream {
public:
    InStream();
    InStream(const SharedBuffer& buffer, uint16_t size, uint16_t begin = 0);

    inline const SharedBuffer& buffer() const { return buffer_; }
    inline uint16_t size() const { return size_; }
    inline uint16_t begin() const { return begin_; }
    inline uint16_t cursor() const { return cursor_; }

    inline const char* data() const { return (buffer_.get() + cursor_); }
    inline uint16_t left() const { return (size_ - cursor_); }
    inline bool eof() const { return cursor_ >= size_; }

    uint16_t skip(uint16_t offset, int origin);
    void clear();

    uint8_t readInt8();
    uint16_t readInt16();
    uint32_t readInt32();
    uint64_t readInt64();
    std::string readString(uint16_t max_length = 0);
    void readBinary(char* buf, uint16_t size);

private:
    SharedBuffer buffer_;
    uint16_t size_;
    uint16_t begin_;
    uint16_t cursor_;
};

} // network
} // mtl

#endif // MTL_NETWORK_IN_STREAM_HPP
