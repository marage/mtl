#include "mtl/network/udp/detail/dgram_group_send_task.hpp"
#include <time.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/in_request.hpp"

namespace mtl {
namespace network {
namespace udp {

static uint16_t nextTaskId()
{
    static uint16_t id = (uint16_t) time(0);
    ++id;
    return (id %= 0xFFFF);
}

GroupSendTask::GroupSendTask(Dgram* s, const SharedBuffer& buffer, uint16_t size,
                             const boost::asio::ip::udp::endpoint& to,
                             const boost::posix_time::ptime& end_time)
    : dgram_(s), id_(nextTaskId()), buffer_(buffer), size_(size), to_(to)
    , end_time_(end_time)
{
    count_ = size / UDP_GROUP_DATA_SIZE;
    if ((size % UDP_GROUP_DATA_SIZE) != 0) {
        ++count_;
    }
    create_time_ = boost::posix_time::microsec_clock::local_time();
    window_time_ = create_time_ + boost::posix_time::milliseconds(500);
    send_time_ = create_time_;
    pos_ = 0;
    status_ = SENDING;
    recvd_ = 0;
    window_ = 5;
    window_pos_ = 0;
    memset(flags_, 1, sizeof(uint8_t) * count_);
}

int32_t GroupSendTask::cost() const
{
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration d = now - create_time_;
    int32_t ms = (int32_t) d.total_milliseconds();
    if (recvd_ > 0) {
        ms = int32_t(ms * (1.0/double(recvd_)));
    }
    return ms;
}

void GroupSendTask::handleSendComplete()
{
    if (status_ == WAITING_SENDING) {
        if (boost::posix_time::microsec_clock::local_time() >= send_time_) {
            status_ = SENDING;
        }
    }
    if (status_ == SENDING) {
        if (pos_ >= count_) {
            uint16_t idx = firstLostIndex();
            if (idx >= count_) {
                send_time_ = boost::posix_time::microsec_clock::local_time()
                        + boost::posix_time::milliseconds(UDP_TRYOUT_TIMEOUT);
                status_ = WAITING_REPORT;
            } else {
                window_ = UDP_WINDOW;
                sendBlock(idx);
            }
        } else {
            sendBlock(pos_++);
        }
    }
}

void GroupSendTask::handleReceivePacket(InRequest& ireq,
                                        const boost::asio::ip::udp::endpoint& from)
{
    uint8_t cmd = ireq.readInt8();
    if (cmd == PT_GROUP_REPORT) {
        //DATA FORMAT : seq | blockcount | blockno list
        recvd_ = ireq.readInt16();
        if (recvd_ >= count_) {
            uint32_t seq = ireq.readSequence();
            dgram_->sendPacketAck(seq, from);
            status_ = COMPLETED;
        } else {
            uint16_t idx;
            while (ireq.left() >= sizeof(uint16_t)) {
                idx = ireq.readInt16();
                flags_[idx] = 0;
            }
            if (pos_ < count_) {
                window_time_ = boost::posix_time::microsec_clock::local_time()
                        + boost::posix_time::milliseconds(1000);
                window_ -= int(window_ * 0.2);
                if (window_ < UDP_WINDOW) {
                    window_ = UDP_WINDOW;
                }
            }
        }
    }
}

void GroupSendTask::handleTimeout(const boost::posix_time::ptime& now)
{
    if (now > end_time_) {
        status_ = FAILED;
    } else if (status_ == WAITING_REPORT) {
        if (now >= send_time_) {
            window_pos_ = 0;
            sendBlock(count_ - 1);
            send_time_ = now + boost::posix_time::milliseconds(UDP_TRYOUT_TIMEOUT);
        }
    } else if (status_ == WAITING_SENDING) {
        if (now >= send_time_) {
            status_ = SENDING;
            handleSendComplete();
        }
    } else if (status_ == SENDING) {
        if (pos_ < count_ && now > window_time_) {
            window_time_ = now + boost::posix_time::milliseconds(window_ * 10);
            ++window_;
        }
    }
}

void GroupSendTask::sendBlock(uint16_t index)
{
    // send the block data, data format: M | count | index | data
    uint16_t data_size = UDP_GROUP_DATA_SIZE;
    uint16_t last_index = count_ - 1;
    if (index == last_index) {
        data_size = size_ % UDP_GROUP_DATA_SIZE;
        if (data_size == 0)
            data_size = UDP_GROUP_DATA_SIZE;
    }
    char* data = buffer_.get() + index * UDP_GROUP_DATA_SIZE;
    // Format: Type | BlockCount | Block | Content
    OutRequest oreq(PT_GROUP_OUT, dgram_->nextSequence());
    oreq.writeInt16(id_);
    oreq.writeInt8(index == last_index ? PT_GROUP_LAST_BLOCK : PT_GROUP_BLOCK);
    oreq.writeInt16(count_);
    oreq.writeInt16(index);
    oreq.writeBinary(data, data_size);
    dgram_->asyncSendTo(oreq, to_, 0);
    flags_[index] = 1;
    if (++window_pos_ >= window_) {
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        int64_t ms = 5;
        if (recvd_ > 0) {
            double t = double((now - create_time_).total_milliseconds()) / (double)recvd_;
            ms = int64_t(t * window_);
        }
        send_time_ = now + boost::posix_time::milliseconds(ms);
        window_pos_ = 0;
        status_ = WAITING_SENDING;
    }
}

uint16_t GroupSendTask::firstLostIndex() const
{
    uint16_t idx = count_ + 1;
    for (uint16_t i = 0; i < count_; ++i) {
        if (flags_[i] == 0) {
            idx = i;
            break;
        }
    }
    return idx;
}

}
}
}
