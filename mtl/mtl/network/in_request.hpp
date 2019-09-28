#ifndef MTL_NETWORK_IN_REQUEST_HPP
#define MTL_NETWORK_IN_REQUEST_HPP
#include "protocol.hpp"
#include "in_stream.hpp"

namespace mtl {
namespace network {

class MTL_EXPORT InRequest : public InStream {
public:
    InRequest();
    InRequest(const SharedBuffer& buffer, uint16_t size,
              uint16_t begin = 0);

    uint16_t dataSize() const { return (size() - kHeaderLength); }
    const UdpEndpoint& from() const { return from_; }
    void setFrom(const UdpEndpoint& from) {
        from_ = from;
    }

    bool isHandled() const { return handled_; }
    void setHandled(bool handled) {
        handled_ = handled;
    }

    uint16_t readLength();
    uint8_t readVersion();
    uint32_t readCommand();
    uint8_t readOption();
    uint32_t readSequence();

private:
    UdpEndpoint from_;
    bool handled_;
};

} // network
} // mtl

#endif // MTL_NETWORK_IN_REQUEST_HPP
