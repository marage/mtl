#ifndef MTL_NETWORK_P2P_PROTOCOL_HPP
#define MTL_NETWORK_P2P_PROTOCOL_HPP
#include <string>
#include "mtl/mtl.hpp"
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {
namespace p2p {

// version
constexpr uint8_t P2P_VERSION = 0x01;

// data packet header: common | seq | timeout
constexpr int SEQ_FIELD_LENGTH = sizeof(uint64_t);
constexpr int TIMEOUT_FIELD_LENGTH = sizeof(uint16_t);
constexpr int P2P_DATA_HEADER_LENGTH = HEADER_LENGTH + SEQ_FIELD_LENGTH + TIMEOUT_FIELD_LENGTH;

// p2p packet type
constexpr uint8_t PT_P2P_JOIN = 0x01;
constexpr uint8_t PT_P2P_JOIN_ACK = 0x02;
constexpr uint8_t PT_P2P_JOIN_NOTIFICATION = 0x03;
constexpr uint8_t PT_P2P_LEAVE = 0x04;
constexpr uint8_t PT_P2P_LEAVE_NOTIFICATION = 0x05;
constexpr uint8_t PT_P2P_ADD_NEIGHBOR = 0x06;
constexpr uint8_t PT_P2P_REPLACE_NEIGHBOR = 0x07;
constexpr uint8_t PT_P2P_BROADCAST_DATA = 0x08;
constexpr uint8_t PT_P2P_DIRECT_DATA = 0x09;
constexpr uint8_t PT_P2P_SEND_FAILED = 0x0A;

constexpr int P2P_NEIGHBOR_COUNT = 3;

struct JoinC2S
{
    std::string token;
    uint8_t role;
};

struct JoinS2C
{
    uint64_t min_seq;
    uint64_t max_seq;
    uint16_t neighbor_count;
};

struct BroadcastRequestHeader
{
    uint64_t seq;
    uint16_t timeout;
};

struct SequenceRange
{
    uint64_t min;
    uint64_t max;

    uint64_t size() const { return max > min ? max - min : 0; }
};

} // p2p
} // network
} // mtl

#endif // MTL_NETWORK_P2P_PROTOCOL_HPP
