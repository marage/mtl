#include "mtl/network/udp/detail/dgram_group_receive_task.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/utility/utility.hpp"
#include <iostream>

namespace mtl {
namespace network {
namespace udp {

GroupReceiveTask::GroupReceiveTask(Dgram* s, uint16_t id, const UDPEndpoint& from)
  : dgram_(s), id_(id), inited_(false), status_(kReceiving)
  , size_(0), count_(0), recvd_(0), max_index_(0), from_(from) {
  create_time_ = SystemClock::now();
  end_time_ = create_time_ + std::chrono::seconds(90);
  report_time_ = create_time_ + std::chrono::milliseconds(50);
}

int32_t GroupReceiveTask::Cost() const {
  auto d = SystemClock::now() - create_time_;
  return (int32_t) (std::chrono::duration_cast<std::chrono::milliseconds>(d)).count();
}

void GroupReceiveTask::HandleReceivePacket(InRequest& ireq,
                                           const UDPEndpoint& /*from*/,
                                           const TimePoint& now) {
  uint8_t cmd = ireq.ReadInt8();
  if (cmd == kGroupBlockType) {
    count_ = ireq.ReadInt8();
    uint8_t index = ireq.ReadInt8();
    if (index == 0) {
      // notify first index
      if (!dgram_->group_head_arrival_signal.empty()) {
        InRequest ir(ireq.buffer(), ireq.size());
        // skip 'id'(16), 'type'(8), 'count'(8), 'block'(8)
        ir.Skip(3 * sizeof(uint8_t) + sizeof(uint16_t), kSkipCurrent);
        bool passed = false;
        dgram_->group_head_arrival_signal(ir, from_, &passed);
        if (!passed) {
          OutRequest oreq(kInGroupType, Dgram::NextSequence());
          oreq.WriteInt16(id_);
          oreq.WriteInt8(kGroupReportType);
          oreq.WriteInt8(count_);
          dgram_->AsyncSendTo(oreq, from_, 5000);
          status_ = kFailed;
          return;
        }
      }
      // notify first block ack
      OutRequest oreq(kInGroupType, Dgram::NextSequence());
      oreq.WriteInt16(id_);
      oreq.WriteInt8(kGroupBlockActType);
      dgram_->AsyncSendTo(oreq, from_, 0);
    }
    if (!buffer_) {
      // create the recv buffer
      buffer_ = SingletonBufferPool::getSingleton().Malloc(kUdpGroupDataSize * count_);
      // init the block flag
      memset(flags_, 0, sizeof(uint8_t) * count_);
    }
    if (flags_[index] == 0) {
      if (max_index_ < index) {
        max_index_ = index;
      }
      flags_[index] = 1;
      char* p = buffer_.get() + index * kUdpGroupDataSize;
      uint16_t size = ireq.Left();
      memcpy(p, (void*)ireq.Data(), size);
      size_ += size;
      if (++recvd_ >= count_) {
        // finished
        OutRequest oreq(kInGroupType, Dgram::NextSequence());
        oreq.WriteInt16(id_);
        oreq.WriteInt8(kGroupReportType);
        oreq.WriteInt8(recvd_);
        dgram_->AsyncSendTo(oreq, from_, 2000);
        status_ = kCompleted;
        return;
      }
    }
    // last block
    if ((index == count_ - 1) && LostCount() > 0) {
      // Format: groupid | type | recvd count | block list
      OutRequest oreq(kInGroupType, Dgram::NextSequence());
      oreq.WriteInt16(id_);
      oreq.WriteInt8(kGroupReportType);
      oreq.WriteInt8(recvd_);
      for (uint8_t i = 0; i < recvd_; ++i) {
        if (flags_[i] == 0) {
          oreq.WriteInt8(i);
        }
      }
      dgram_->AsyncSendTo(oreq, from_, 0);
      report_time_ = now;
      std::cout << LostCount() << std::endl;
    }
  } else if (cmd == kGroupCancelType) {
    status_ = kFailed;
  }
}

void GroupReceiveTask::HandleTimeout(const TimePoint& now) {
  if (now >= end_time_) {
    if (status_ != kCompleted) {
      status_ = kFailed;
    }
  } else if (status_ == kReceiving && report_time_ > now) {
    if (double(LostCount())/double(max_index_) > 0.2) {
      // Format: groupid | type | recvd count | block list
      OutRequest oreq(kInGroupType, Dgram::NextSequence());
      oreq.WriteInt16(id_);
      oreq.WriteInt8(kGroupReportType);
      oreq.WriteInt8(recvd_);
      for (uint8_t i = 0; i < recvd_; i++) {
        if (flags_[i] == 0) {
          oreq.WriteInt8(i);
        }
      }
      dgram_->AsyncSendTo(oreq, from_, 0);
    }
    report_time_ = now + std::chrono::milliseconds(50);
  }
}

uint8_t GroupReceiveTask::LostCount() const {
  uint8_t lost = 0;
  uint8_t count = recvd_;
  for (uint8_t i = 0; i < count && flags_[i] == 0; i++) {
    lost++;
  }
  return lost;
}

}
}
}
