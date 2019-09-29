#ifndef MTL_NETWORK_PROTOCOL_HPP
#define MTL_NETWORK_PROTOCOL_HPP
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "mtl/mtl.hpp"

namespace mtl {
namespace network {

// version
constexpr uint8_t kVersion = 0x01;

// format: len | ver | common | option | seq
constexpr uint16_t kLenFieldLength = sizeof(uint16_t);
constexpr uint16_t kVerFieldLength = sizeof(uint8_t);
constexpr uint16_t kCommandFieldLength = sizeof(uint32_t);
constexpr uint16_t kOptionFieldLength = sizeof(uint8_t);
constexpr uint16_t kSeqFieldLength = sizeof(uint32_t);
constexpr uint16_t kHeaderLength = kLenFieldLength + kVerFieldLength
        + kCommandFieldLength + kOptionFieldLength + kSeqFieldLength;

enum SkipPosition
{
    kSkipBegin,
    kSkipCurrent,
    kSkipEnd
};

using UdpEndpoint = boost::asio::ip::udp::endpoint;
using TcpEndpoint = boost::asio::ip::tcp::endpoint;

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_PROTOCOL_HPP
