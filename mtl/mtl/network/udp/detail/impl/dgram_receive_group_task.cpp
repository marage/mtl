#include "mtl/network/udp/detail/dgram_receive_group_task.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"

namespace mtl {
namespace network {
namespace udp {

ReceiveGroupTask::ReceiveGroupTask(Dgram* s, uint16_t id,
                                   const UdpEndpoint& from)
    : dgram_(s), id_(id), inited_(false), status_(kWaitingBlock)
    , size_(0), count_(0), recvd_(0), max_index_(0), from_(from)
{
    create_time_ = std::chrono::system_clock::now();
    end_time_ = create_time_ + std::chrono::seconds(90);
    report_time_ = create_time_ + std::chrono::milliseconds(50);
}

int32_t ReceiveGroupTask::cost() const
{
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::system_clock::duration d = now - create_time_;
    return int32_t(std::chrono::duration_cast<std::chrono::milliseconds>(d).count());
}

void ReceiveGroupTask::handleTimeout(const std::chrono::system_clock::time_point& now)
{
    if (now >= end_time_) {
        if (status_ != kCompletedReceiving) {
            status_ = kFailed;
        }
    } else if (report_time_ > now) {
        if (max_index_ >= kUdpWindow && double(lostCount())/double(max_index_) > 0.2) {
            // Format: groupid | type | recvd count | block list
            OutRequest oreq(kInGroupType, Dgram::nextSequence());
            oreq.writeInt16(id_);
            oreq.writeInt8(kGroupReportType);
            oreq.writeInt16(recvd_);
            for (uint16_t i = 0; i < recvd_; ++i) {
                if (flags_[i] == 0) {
                    oreq.writeInt16(i);
                }
            }
            dgram_->asyncSendTo(oreq, from_, 0);
        }
        report_time_ = now + std::chrono::milliseconds(50);
    }
}

uint16_t ReceiveGroupTask::lostCount() const
{
    uint16_t lost = 0;
    uint16_t count = recvd_;
    for (uint16_t i = 0; i < count && flags_[i] == 0; i++) {
        ++lost;
    }
    return lost;
}

void ReceiveGroupTask::handleReceivePacket(InRequest& ireq, const UdpEndpoint& /*from*/)
{
    uint8_t cmd = ireq.readInt8();
    if (cmd == kGroupBlockType || cmd == kGroupLastBlockType) {
        // get the block count
        count_ = ireq.readInt16();
        // get the block data
        uint16_t index = ireq.readInt16();
        // notify first index
        if (index == 0 && !dgram_->group_head_arrival_signal.empty()) {
            InRequest ir(ireq.buffer(), ireq.size());
            // skip 'id'(16), 'type'(8), 'count'(16), 'block'(16)
            ir.skip(3 * sizeof(uint16_t) + sizeof(uint8_t), kSkipCurrent);
            bool passed = false;
            dgram_->group_head_arrival_signal(ir, from_, &passed);
            if (!passed) {
                OutRequest oreq(kInGroupType, Dgram::nextSequence());
                oreq.writeInt16(id_);
                oreq.writeInt8(kGroupReportType);
                oreq.writeInt16(count_);
                dgram_->asyncSendTo(oreq, from_, 5000);
                status_ = kFailed;
                return;
            }
        }
        if (!buffer_) {
            // create the recv buffer
            buffer_ = SingletonBufferPool::getSingleton().allocBuffer(kUdpGroupDataSize * count_);
            // init the block flag
            memset(flags_, 0, sizeof(uint8_t) * count_);
        }
        if (flags_[index] == 0) {
            if (max_index_ < index) {
                max_index_ = index;
            }
            flags_[index] = 1;
            char* p = buffer_.get() + index * kUdpGroupDataSize;
            uint16_t size = ireq.left();
            memcpy(p, ireq.data(), size);
            size_ += size;
            if (++recvd_ >= count_) {
                // finished
                OutRequest oreq(kInGroupType, Dgram::nextSequence());
                oreq.writeInt16(id_);
                oreq.writeInt8(kGroupReportType);
                oreq.writeInt16(recvd_);
                dgram_->asyncSendTo(oreq, from_, 5000);
                status_ = kCompletedReceiving;
                return;
            }
        }
        if (cmd == kGroupLastBlockType && lostCount() > 0) {
            // Format: groupid | type | recvd count | block list
            OutRequest oreq(kInGroupType, Dgram::nextSequence());
            oreq.writeInt16(id_);
            oreq.writeInt8(kGroupReportType);
            oreq.writeInt16(recvd_);
            for (uint16_t i = 0; i < recvd_; ++i) {
                if (flags_[i] == 0) {
                    oreq.writeInt16(i);
                }
            }
            dgram_->asyncSendTo(oreq, from_, 0);
        }
    } else if (cmd == kGroupCancelType) {
        status_ = kFailed;
    }
}

} // namespace udp
} // namespace network
} // namespace mtl
