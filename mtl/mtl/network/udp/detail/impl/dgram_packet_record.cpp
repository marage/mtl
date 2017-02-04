#include "mtl/network/udp/detail/dgram_packet_record.hpp"

namespace mtl {
namespace network {
namespace udp {

PacketRecord::PacketRecord() : pos_(0)
{
    memset(seqs_, 0, MAX_RECORD_COUNT * sizeof(uint32_t));
}

bool PacketRecord::passed(uint32_t seq, const boost::asio::ip::udp::endpoint& from)
{
    bool ok = true;
    for (int i = 0; i < MAX_RECORD_COUNT; i++) {
        if (seq == seqs_[i] && from == addresses_[i]) {
            ok = false;
            break;
        }
    }
    if (ok) {
        seqs_[pos_] = seq;
        addresses_[pos_] = from;
        ++pos_;
        pos_ = pos_ % MAX_RECORD_COUNT;
    }
    return ok;
}

// GroupRecord
GroupRecord::GroupRecord() : pos_(0)
{
    memset(ids_, 0, MAX_RECORD_COUNT * sizeof(uint16_t));
}

bool GroupRecord::exists(uint16_t id, uint16_t count,
                         const boost::asio::ip::udp::endpoint& from) const
{
    bool ok = true;
    for (int i = 0; i < MAX_RECORD_COUNT; i++) {
        if (id == ids_[i] && count == counts_[i] && from == addresses_[i]) {
            ok = false;
            break;
        }
    }
    return ok;
}

void GroupRecord::append(uint16_t id, uint16_t count,
                         const boost::asio::ip::udp::endpoint& from)
{
    ids_[pos_] = id;
    counts_[pos_] = count;
    addresses_[pos_] = from;
    ++pos_;
    pos_ = pos_ % MAX_RECORD_COUNT;
}
}
}
}
