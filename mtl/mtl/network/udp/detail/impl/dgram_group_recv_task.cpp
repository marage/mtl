#include "mtl/network/udp/detail/dgram_group_recv_task.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"

namespace mtl {
namespace network {
namespace udp {

GroupRecvTask::GroupRecvTask(Dgram* s, uint16_t id,
                             const boost::asio::ip::udp::endpoint& from)
    : dgram_(s), id_(id), inited_(false), status_(WAITING)
    , size_(0), count_(0), recvd_(0), max_index_(0), from_(from)
{
    create_time_ = boost::posix_time::microsec_clock::local_time();
    end_time_ = create_time_ + boost::posix_time::seconds(90);
    report_time_ = create_time_ + boost::posix_time::milliseconds(50);
}

int32_t GroupRecvTask::cost() const
{
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration d = now - create_time_;
    return (int32_t) d.total_milliseconds();
}

void GroupRecvTask::handleTimeout(const boost::posix_time::ptime& now)
{
    if (now >= end_time_) {
        if (status_ != COMPLETED) {
            status_ = FAILED;
        }
    } else if (report_time_ > now) {
        if (max_index_ >= UDP_WINDOW && double(lostCount())/double(max_index_) > 0.2) {
            // Format: groupid | type | recvd count | block list
            OutRequest oreq(PT_GROUP_IN, Dgram::nextSequence());
            oreq.writeInt16(id_);
            oreq.writeInt8(PT_GROUP_REPORT);
            oreq.writeInt16(recvd_);
            for (uint16_t i = 0; i < recvd_; ++i) {
                if (flags_[i] == 0) {
                    oreq.writeInt16(i);
                }
            }
            dgram_->asyncSendTo(oreq, from_, 0);
        }
        report_time_ = now + boost::posix_time::milliseconds(50);
    }
}

uint16_t GroupRecvTask::lostCount() const
{
    uint16_t lost = 0;
    uint16_t count = recvd_;
    for (uint16_t i = 0; i < count && flags_[i] == 0; i++) {
        ++lost;
    }
    return lost;
}

void GroupRecvTask::handleReceivePacket(InRequest& ireq,
                                        const boost::asio::ip::udp::endpoint& /*from*/)
{
    uint8_t cmd = ireq.readInt8();
    if (cmd == PT_GROUP_BLOCK || cmd == PT_GROUP_LAST_BLOCK) {
        // get the block count
        count_ = ireq.readInt16();
        // get the block data
        uint16_t index = ireq.readInt16();
        // notify first index
        if (index == 0 && !dgram_->group_head_arrival_signal.empty()) {
            InRequest ir(ireq.buffer(), ireq.size());
            // skip 'id'(16), 'type'(8), 'count'(16), 'block'(16)
            ir.skip(3 * sizeof(uint16_t) + sizeof(uint8_t), SKIP_CURRENT);
            bool passed = false;
            dgram_->group_head_arrival_signal(ir, from_, &passed);
            if (!passed) {
                OutRequest oreq(PT_GROUP_IN, Dgram::nextSequence());
                oreq.writeInt16(id_);
                oreq.writeInt8(PT_GROUP_REPORT);
                oreq.writeInt16(count_);
                dgram_->asyncSendTo(oreq, from_, 5000);
                status_ = FAILED;
                return;
            }
        }
        if (!buffer_) {
            // create the recv buffer
            buffer_ = SingletonBufferPool::getSingleton().malloc(UDP_GROUP_DATA_SIZE * count_);
            // init the block flag
            memset(flags_, 0, sizeof(uint8_t) * count_);
        }
        if (flags_[index] == 0) {
            if (max_index_ < index) {
                max_index_ = index;
            }
            flags_[index] = 1;
            char* p = buffer_.get() + index * UDP_GROUP_DATA_SIZE;
            uint16_t size = ireq.left();
            memcpy(p, (void*)ireq.data(), size);
            size_ += size;
            if (++recvd_ >= count_) {
                // finished
                OutRequest oreq(PT_GROUP_IN, Dgram::nextSequence());
                oreq.writeInt16(id_);
                oreq.writeInt8(PT_GROUP_REPORT);
                oreq.writeInt16(recvd_);
                dgram_->asyncSendTo(oreq, from_, 5000);
                status_ = COMPLETED;
                return;
            }
        }
        if (cmd == PT_GROUP_LAST_BLOCK && lostCount() > 0) {
            // Format: groupid | type | recvd count | block list
            OutRequest oreq(PT_GROUP_IN, Dgram::nextSequence());
            oreq.writeInt16(id_);
            oreq.writeInt8(PT_GROUP_REPORT);
            oreq.writeInt16(recvd_);
            for (uint16_t i = 0; i < recvd_; ++i) {
                if (flags_[i] == 0) {
                    oreq.writeInt16(i);
                }
            }
            dgram_->asyncSendTo(oreq, from_, 0);
        }
    } else if (cmd == PT_GROUP_CANCEL) {
        status_ = FAILED;
    }
}

}
}
}
