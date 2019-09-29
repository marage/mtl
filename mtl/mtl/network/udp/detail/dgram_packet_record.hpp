#ifndef MTL_NETWORK_DGRAM_PACKET_RECORD_HPP
#define MTL_NETWORK_DGRAM_PACKET_RECORD_HPP
#include <boost/cstdint.hpp>
#include <unordered_set>
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {
namespace udp {

class PacketRecord
{
public:
    enum
    {
        kMaxRecordCount = 500
    };

    PacketRecord();
    bool isPassed(uint32_t seq, const UdpEndpoint& from);

private:
    std::unordered_set<uint32_t> seq_set_;
    uint32_t seqs_[kMaxRecordCount];
    UdpEndpoint addresses_[kMaxRecordCount];
    int pos_;
};

class GroupRecord
{
public:
    enum
    {
        kMaxRecordCount = 500
    };

    GroupRecord();
    bool exists(uint16_t id, uint8_t count,
                const UdpEndpoint& from) const;
    void append(uint16_t id, uint8_t count,
                const UdpEndpoint& from);

private:
    uint16_t ids_[kMaxRecordCount];
    uint8_t counts_[kMaxRecordCount];
    UdpEndpoint addresses_[kMaxRecordCount];
    int pos_;
};

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_PACKET_RECORD_HPP
