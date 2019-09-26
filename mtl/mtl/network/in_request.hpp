#ifndef MTL_NETWORK_IN_REQUEST_HPP
#define MTL_NETWORK_IN_REQUEST_HPP
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/udp.hpp>
#include "in_stream.hpp"

namespace mtl {
namespace network {

class MTL_EXPORT InRequest : public InStream
{
public:
    InRequest();
    InRequest(const SharedBuffer& buffer, uint16_t size,
              uint16_t begin = 0);

    const boost::asio::ip::udp::endpoint& from() const { return from_; }
    void setFrom(const boost::asio::ip::udp::endpoint& from)
    {
        from_ = from;
    }

    bool isHandled() const { return handled_; }
    void setHandled(bool handled)
    {
        handled_ = handled;
    }

    uint16_t readLength();
    uint8_t readVersion();
    uint32_t readCommand();
    uint8_t readOption();
    uint32_t readSequence();

private:
    boost::asio::ip::udp::endpoint from_;
    bool handled_;
};

} // network
} // mtl

#endif // MTL_NETWORK_IN_REQUEST_HPP
