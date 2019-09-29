#ifndef MTL_NETWORK_P2P_GROUP_PACKET_FILTER_HPP
#define MTL_NETWORK_P2P_GROUP_PACKET_FILTER_HPP
#include <boost/cstdint.hpp>
#include <unordered_set>
#include <list>

namespace mtl {
namespace network {
namespace p2p {

// a packet filter strategy
class GroupPacketFilter
{
public:
    GroupPacketFilter() = default;

    bool isPassed(uint64_t seq);
    void remove(uint64_t min_seq, uint64_t max_seq);

private:
    std::unordered_set<uint64_t> seq_set_;
    std::list<uint64_t> seq_list_;
};

} // p2p
} // network
} // mtl

#endif // MTL_NETWORK_P2P_GROUP_PACKET_FILTER_HPP
