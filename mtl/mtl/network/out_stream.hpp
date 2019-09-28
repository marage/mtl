#ifndef MTL_NETWORK_OUT_STREAM_HPP
#define MTL_NETWORK_OUT_STREAM_HPP
#include <boost/cstdint.hpp>
#include <string>
#include "mtl/mtl.hpp"
#include "shared_buffer.hpp"

namespace mtl {
namespace network {

class MTL_EXPORT OutStream {
public:
    explicit OutStream(uint16_t begin = 0);
    OutStream(const SharedBuffer& buffer, uint16_t size, uint16_t begin);

    inline const SharedBuffer& buffer() const;
    inline uint16_t begin() const;
    inline uint16_t size() const;
    inline uint16_t left() const;

    uint16_t skip(uint16_t offset, int origin);
    void resize(uint16_t sz);
    void clear();

    OutStream& writeInt8(uint8_t val);
    OutStream& writeInt16(uint16_t val);
    OutStream& writeInt32(uint32_t val);
    OutStream& writeInt64(uint64_t val);
    OutStream& writeString(const std::string& val);
    OutStream& writeBinary(const char* data, uint16_t data_size);

private:
    SharedBuffer buffer_;
    uint16_t size_;
    uint16_t begin_;
    char* cursor_;
};

inline const SharedBuffer& OutStream::buffer() const {
    return buffer_;
}

inline uint16_t OutStream::begin() const {
    return begin_;
}

inline uint16_t OutStream::size() const {
    return static_cast<uint16_t>((cursor_ - buffer_.get()));
}

inline uint16_t OutStream::left() const {
    return (size_ - size());
}

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_OUT_REQUEST_HPP
