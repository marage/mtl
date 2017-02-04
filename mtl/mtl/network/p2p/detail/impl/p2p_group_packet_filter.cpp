#include "mtl/network/p2p/detail/p2p_group_packet_filter.hpp"

namespace mtl {
namespace network {
namespace p2p {

bool GroupPacketFilter::passed(uint64_t seq)
{
    const size_t max_record_count = 1000;
    if (seq_set_.find(seq) == seq_set_.end()) {
        seq_set_.insert(seq);
        seq_list_.push_back(seq);
        if (seq_list_.size() > max_record_count) {
            seq_set_.erase(*seq_list_.begin());
            seq_list_.erase(seq_list_.begin());
        }
        return true;
    }
    return false;
}

void GroupPacketFilter::remove(uint64_t min_seq, uint64_t max_seq)
{
    for (auto it = seq_list_.begin(), end = seq_list_.end();
         it != end; ++it) {
        if ((*it) >= min_seq && (*it) < max_seq) {
            seq_set_.erase(*it);
        }
    }
}
}
}
}
