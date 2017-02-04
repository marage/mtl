#ifndef MTL_NETWORK_DGRAM_PACKET_RECORD_HPP
#define MTL_NETWORK_DGRAM_PACKET_RECORD_HPP
#include <boost/cstdint.hpp>
#include <boost/asio/ip/udp.hpp>
#include <unordered_set>

namespace mtl {
namespace network {
namespace udp {

class PacketRecord
{
public:
    enum
    {
        MAX_RECORD_COUNT = 500
    };

    PacketRecord();
    bool passed(uint32_t seq, const boost::asio::ip::udp::endpoint& from);

private:
    std::unordered_set<uint32_t> seq_set_;
    uint32_t seqs_[MAX_RECORD_COUNT];
    boost::asio::ip::udp::endpoint addresses_[MAX_RECORD_COUNT];
    int pos_;
};

class GroupRecord
{
public:
    enum
    {
        MAX_RECORD_COUNT = 500
    };

    GroupRecord();
    bool exists(uint16_t id, uint16_t count,
                const boost::asio::ip::udp::endpoint& from) const;
    void append(uint16_t id, uint16_t count,
                const boost::asio::ip::udp::endpoint& from);

private:
    uint16_t ids_[MAX_RECORD_COUNT];
    uint16_t counts_[MAX_RECORD_COUNT];
    boost::asio::ip::udp::endpoint addresses_[MAX_RECORD_COUNT];
    int pos_;
};

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_PACKET_RECORD_HPP
