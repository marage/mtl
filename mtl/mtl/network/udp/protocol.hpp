#ifndef MTL_NETWORK_UDP_PROTOCOL_HPP
#define MTL_NETWORK_UDP_PROTOCOL_HPP
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {
namespace udp {

// version
constexpr uint8_t kUdpVersion = 0x01;

// udp packet size
constexpr uint16_t kUdpMaxPacketSize = 1024;

// format: common header
constexpr uint16_t kUdpHeaderLength = kHeaderLength;

// dgram group packet request header size
// header | groupid(int16_t) | subtype(int8_t) | blockcount(int8_t)
// | blockindex(int8_t)
constexpr uint16_t kUdpGroupHeaderLength = kUdpHeaderLength + sizeof(int16_t)
    + 3 * sizeof(int8_t);

constexpr uint16_t kUdpDataSize = kUdpMaxPacketSize - kUdpHeaderLength;
constexpr uint16_t kUdpGroupDataSize = kUdpMaxPacketSize - kUdpGroupHeaderLength;

constexpr uint16_t kUdpGroupBlockCount = 255;
constexpr uint16_t kUdpBufferSize =
    uint16_t(kUdpGroupBlockCount * kUdpGroupDataSize - kUdpHeaderLength);

constexpr uint32_t kUdpTimeout = 1000; // 1s
constexpr uint32_t kUdpTryoutTimeout = 500;
constexpr uint32_t kUdpTryoutCount = 3;
constexpr uint32_t kUdpAlive = 2000;   // 2s
constexpr uint8_t kUdpWindow = 8;
constexpr uint8_t kUdpMaxWindow = 255;

// packet type
constexpr uint16_t kUdpType = 0x01;
constexpr uint16_t kRudpType = 0x02;
constexpr uint16_t kAckType = 0x03;
constexpr uint16_t kOutGroupType = 0x04;
constexpr uint16_t kInGroupType = 0x05;
constexpr uint16_t kKeepAliveType = 0x06;

// udp group packet type
constexpr uint8_t kGroupBlockType = 0x01;
constexpr uint8_t kGroupBlockActType = 0x02;
constexpr uint8_t kGroupReportType = 0x03;
constexpr uint8_t kGroupCancelType = 0x04;

} //udp
} //network
} //mtl

#endif // MTL_NETWORK_UDP_PROTOCOL_HPP
