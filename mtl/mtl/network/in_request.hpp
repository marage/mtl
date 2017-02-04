#ifndef MTL_NETWORK_IN_REQUEST_HPP
#define MTL_NETWORK_IN_REQUEST_HPP
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/udp.hpp>
#include "in_stream.hpp"

namespace mtl {
namespace network {

class _MTL_EXPORT InRequest : public InStream
{
public:
    InRequest();
    InRequest(const SharedBuffer& buffer, uint16_t size,
              uint16_t begin = 0);

    inline const boost::asio::ip::udp::endpoint& from() const { return from_; }

    inline bool isHandled() const { return handled_; }

    inline void setFrom(const boost::asio::ip::udp::endpoint& from)
    {
        from_ = from;
    }

    inline void setHandled(bool handled)
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
