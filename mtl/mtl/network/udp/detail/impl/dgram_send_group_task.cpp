﻿#include "mtl/network/udp/detail/dgram_send_group_task.hpp"
#include <time.h>
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/in_request.hpp"

namespace mtl {
namespace network {
namespace udp {

static uint16_t nextTaskId()
{
    static uint16_t id = uint16_t(time(nullptr));
    ++id;
    return (id %= 0xFFFF);
}

std::map<std::string, uint8_t> SendGroupTask::windows;

SendGroupTask::SendGroupTask(Dgram* s, const SharedBuffer& buffer, uint16_t size,
                             const UdpEndpoint& to, const std::chrono::system_clock::time_point& end_time)
    : dgram_(s), id_(nextTaskId()), buffer_(buffer), size_(size), to_(to)
    , end_time_(end_time) {
    count_ = size / kUdpGroupDataSize;
    if ((size % kUdpGroupDataSize) != 0) {
        ++count_;
    }
    create_time_ = std::chrono::system_clock::now();
    window_time_ = create_time_ + std::chrono::milliseconds{500};
    send_time_ = create_time_;
    pos_ = 0;
    status_ = kSending;
    recvd_ = 0;
    window_ = kUdpWindow;
    window_pos_ = 0;
    memset(flags_, 1, sizeof(uint8_t) * count_);
    // apply history window
    auto it = windows.find(Dgram::toString(to_));
    if (it != windows.end()) {
        window_ = it->second;
    }
}

SendGroupTask::~SendGroupTask()
{
    windows.insert(std::make_pair(Dgram::toString(to_), window_));
}

int32_t SendGroupTask::cost() const
{
    auto now = std::chrono::system_clock::now();
    auto d = now - create_time_;
    int64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
    if (recvd_ > 0) {
        ms = int64_t(ms * (1.0/double(recvd_)));
    }
    return int32_t(ms);
}

void SendGroupTask::handleSendComplete(const std::chrono::system_clock::time_point& now)
{
    if (status_ == kWaitingSending) {
        if (now >= send_time_) {
            status_ = kSending;
        }
    }
    if (status_ == kSending) {
        if (pos_ >= count_) {
            uint8_t idx = firstLostIndex();
            if (idx >= count_) {
                send_time_ = now + std::chrono::milliseconds(kUdpTryoutTimeout);
                status_ = kWaitingReport;
            } else {
                window_ = kUdpWindow;
                sendBlock(idx, now);
            }
        } else {
            sendBlock(pos_++, now);
        }
    }
}

void SendGroupTask::handleReceivePacket(InRequest& ireq, const UdpEndpoint& from)
{
    uint8_t cmd = ireq.readInt8();
    if (cmd == kGroupReportType) {
        //DATA FORMAT : seq | blockcount | blockno list
        recvd_ = ireq.readInt8();
        if (recvd_ >= count_) {
            uint32_t seq = ireq.readSequence();
            dgram_->sendPacketAck(seq, from);
            status_ = kCompleted;
        } else {
            uint8_t idx;
            while (ireq.left() >= sizeof(uint8_t)) {
                idx = ireq.readInt8();
                flags_[idx] = 0;
            }
            // 發送窗口減半(丟失了一半)
            if (pos_ > 0 && (float(recvd_) / float(pos_)) < 0.5f) {
                window_time_ = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
                window_ -= int(window_ * 0.2);
                if (window_ < kUdpWindow) {
                    window_ = kUdpWindow;
                }
            }
            status_ = kSending;
            auto now = std::chrono::system_clock::now();
            if (pos_ < (count_ - 1)) {
                sendBlock(pos_++, now);
            } else {
                sendBlock(firstLostIndex(), now);
            }
        }
    } else if (cmd == kGroupBlockActType) {
        status_ = kSending;
        sendBlock(pos_++, std::chrono::system_clock::now());
    }
}

void SendGroupTask::handleTimeout(const std::chrono::system_clock::time_point& now)
{
    if (now >= end_time_) {
        status_ = kFailed;
    } else if (status_ == kSending) {
        if (pos_ < count_ && now > window_time_) {
            window_time_ = now + std::chrono::milliseconds(window_ * 10);
            ++window_;
        }
    } else if (status_ == kWaitingReport) {
        if (now >= send_time_) {
            window_pos_ = 0;
            sendBlock(count_ - 1, now);
            send_time_ = now + std::chrono::milliseconds(kUdpTryoutTimeout);
        }
    } else if (status_ == kWaitingSending) {
        if (now >= send_time_) {
            status_ = kSending;
            handleSendComplete(now);
        }
    } else if (status_ == kWaitingAck) {
        if (now >= send_time_) {
            auto d = now - create_time_;
            int64_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
            if (msecs >= kUdpTryoutCount * kUdpTryoutTimeout) {
                status_ = kFailed;
            } else {
                pos_ = 0;
                sendBlock(pos_++, now);
            }
        }
    }
}

void SendGroupTask::sendBlock(uint8_t index, const std::chrono::system_clock::time_point& now)
{
    // send the block data, data format: M | count | index | data
    uint16_t data_size = kUdpGroupDataSize;
    uint8_t last_index = count_ - 1;
    if (index == last_index) {
        data_size = size_ % kUdpGroupDataSize;
        if (data_size == 0) {
            data_size = kUdpGroupDataSize;
        }
    }
    char* data = buffer_.get() + index * kUdpGroupDataSize;
    // Format: Type | BlockCount | Block | Content
    OutRequest oreq(kSendGroupType, dgram_->nextSequence());
    oreq.writeInt16(id_);
    oreq.writeInt8(kGroupBlockType);
    oreq.writeInt8(count_);
    oreq.writeInt8(index);
    oreq.writeBinary(data, data_size);
    dgram_->asyncSendTo(oreq, to_, 0);
    flags_[index] = 1;
    if (index == 0) {
        send_time_ = now + std::chrono::milliseconds(kUdpTryoutTimeout);
        status_ = kWaitingAck;
    } else if (++window_pos_ >= window_) {
        int64_t ms = 5;
        if (recvd_ > 0) {
            auto d = now - create_time_;
            int64_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
            double t = double(msecs) / double(recvd_);
            ms = int64_t(t * 0.5 * window_);
        }
        send_time_ = now + std::chrono::milliseconds(ms);
        window_pos_ = 0;
        status_ = kWaitingSending;
    }
}

uint8_t SendGroupTask::firstLostIndex() const
{
    uint8_t idx = count_ + 1;
    for (uint8_t i = 0; i < count_; i++) {
        if (flags_[i] == 0) {
            idx = i;
            break;
        }
    }
    return idx;
}

} // namespace udp
} // namespace network
} // namespace mtl
