#ifndef MTL_NETWORK_UDP_PROTOCOL_HPP
#define MTL_NETWORK_UDP_PROTOCOL_HPP
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {
namespace udp {

// version
const uint8_t UDP_VERSION = 0x01;

// udp packet size
const int UDP_PACKET_SIZE = 1024;

// format: common header
const int UDP_HEADER_LENGTH = HEADER_LENGTH;

// dgram group packet request header size
// header | groupid(int16_t) | subtype(int8_t) | blockcount(int16_t) | blockindex(int16_t)
const int UDP_GROUP_HEADER_LENGTH = UDP_HEADER_LENGTH + sizeof(int8_t) + 3 * sizeof(int16_t);

const int UDP_DATA_SIZE = UDP_PACKET_SIZE - UDP_HEADER_LENGTH;
const int UDP_GROUP_DATA_SIZE = UDP_PACKET_SIZE - UDP_GROUP_HEADER_LENGTH;

const int UDP_GROUP_BLOCK_COUNT = UDP_GROUP_DATA_SIZE / sizeof(int16_t);
const int UDP_BUFFER_SIZE = UDP_GROUP_BLOCK_COUNT * UDP_GROUP_DATA_SIZE - UDP_HEADER_LENGTH;

const int UDP_TIMEOUT = 1000; // 1s
const int UDP_TRYOUT_TIMEOUT = 500;
const int UDP_WINDOW = 5;
const int UDP_ALIVE = 2000;   // 2s

// packet type
const uint16_t PT_UDP = 0x01;
const uint16_t PT_RUDP = 0x02;
const uint16_t PT_ACK = 0x03;
const uint16_t PT_GROUP_OUT = 0x04;
const uint16_t PT_GROUP_IN = 0x05;
const uint16_t PT_KEEPALIVE = 0x06;

// udp group packet type
const uint8_t PT_GROUP_BLOCK = 0x01;
const uint8_t PT_GROUP_LAST_BLOCK = 0x02;
const uint8_t PT_GROUP_REPORT = 0x03;
const uint8_t PT_GROUP_OK = 0x04;
const uint8_t PT_GROUP_CANCEL = 0x05;

} //udp
} //network
} //mtl

#endif // MTL_NETWORK_UDP_PROTOCOL_HPP
