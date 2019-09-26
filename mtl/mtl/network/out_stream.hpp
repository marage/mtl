#ifndef MTL_NETWORK_OUT_STREAM_HPP
#define MTL_NETWORK_OUT_STREAM_HPP
#include <boost/cstdint.hpp>
#include <string>
#include "mtl/mtl.hpp"
#include "shared_buffer.hpp"

namespace mtl {
namespace network {

class MTL_EXPORT OutStream
{
public:
    explicit OutStream(uint16_t begin = 0);
    OutStream(const SharedBuffer& buffer, uint16_t size,
              uint16_t begin);

    const SharedBuffer& buffer() const { return buffer_; }
    uint16_t size() const
    {
        return static_cast<uint16_t>((cursor_ - buffer_.get()));
    }
    uint16_t begin() const { return begin_; }
    uint16_t left() const { return (size_ - size()); }

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

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_OUT_REQUEST_HPP
