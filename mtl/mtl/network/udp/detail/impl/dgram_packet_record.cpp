#include "mtl/network/udp/detail/dgram_packet_record.hpp"

namespace mtl {
namespace network {
namespace udp {

PacketRecord::PacketRecord() : pos_(0)
{
    memset(seqs_, 0, kMaxRecordCount * sizeof(uint32_t));
}

bool PacketRecord::isPassed(uint32_t seq, const UdpEndpoint& from)
{
    bool ok = true;
    for (int i = 0; i < kMaxRecordCount; i++) {
        if (seq == seqs_[i] && from == addresses_[i]) {
            ok = false;
            break;
        }
    }
    if (ok) {
        seqs_[pos_] = seq;
        addresses_[pos_] = from;
        ++pos_;
        pos_ = pos_ % kMaxRecordCount;
    }
    return ok;
}

// GroupRecord
GroupRecord::GroupRecord() : pos_(0)
{
    memset(ids_, 0, kMaxRecordCount * sizeof(uint16_t));
}

bool GroupRecord::exists(uint16_t id, uint16_t count, const UdpEndpoint& from) const
{
    bool ok = true;
    for (int i = 0; i < kMaxRecordCount; i++) {
        if (id == ids_[i] && count == counts_[i] && from == addresses_[i]) {
            ok = false;
            break;
        }
    }
    return ok;
}

void GroupRecord::append(uint16_t id, uint16_t count, const UdpEndpoint& from)
{
    ids_[pos_] = id;
    counts_[pos_] = count;
    addresses_[pos_] = from;
    ++pos_;
    pos_ = pos_ % kMaxRecordCount;
}
} // namespace udp
} // namespace network
} // namespace mtl
