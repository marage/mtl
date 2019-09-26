#ifndef MTL_NETWORK_OUT_REQUEST_HPP
#define MTL_NETWORK_OUT_REQUEST_HPP
#include "out_stream.hpp"

namespace mtl {
namespace network {

class MTL_EXPORT OutRequest : public OutStream
{
public:
    explicit OutRequest(uint32_t cmd = 0, uint32_t seq = 0, uint16_t begin = 0);
    OutRequest(const SharedBuffer& buffer, uint16_t size, uint16_t begin = 0);

    uint16_t length() const;
    uint8_t version() const;
    uint32_t command() const;
    uint8_t option() const;
    uint32_t sequence() const;
    const char* data() const;
    uint16_t data_size() const;

    OutRequest& writeLength(uint16_t len);
    OutRequest& writeVersion(uint8_t ver);
    OutRequest& writeCommand(uint32_t cmd);
    OutRequest& writeOption(uint8_t option);
    OutRequest& writeSequence(uint32_t seq);
};

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_OUT_REQUEST_HPP
