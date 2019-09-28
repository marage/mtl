#ifndef MTL_NETWORK_DGRAM_GROUP_RECEIVE_TASK_HPP
#define MTL_NETWORK_DGRAM_GROUP_RECEIVE_TASK_HPP
#include <boost/noncopyable.hpp>
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/shared_buffer.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;

namespace udp {

class Dgram;

class GroupReceiveTask : public boost::noncopyable {
public:
  GroupReceiveTask(Dgram* s, uint16_t id, const UDPEndpoint& from);

  int32_t Cost() const;
  inline const SharedBuffer& data() const;
  inline uint16_t size() const;
  inline uint8_t count() const;

  inline bool CompareId(uint16_t id,
                        const UDPEndpoint& from) const;
  inline bool IsCompleted() const;
  inline bool IsFailed() const;

  inline const UDPEndpoint& from() const { return from_; }

  void HandleReceivePacket(InRequest& ireq,
                           const UDPEndpoint& from,
                           const TimePoint& now);
  void HandleTimeout(const TimePoint& now);

private:
  uint8_t LostCount() const;

  enum Status {
    kReceiving,
    kCompleted,
    kFailed,
  };

  Dgram* dgram_;
  uint16_t id_;
  TimePoint create_time_;
  TimePoint end_time_;
  TimePoint report_time_;
  bool inited_;
  Status status_;
  SharedBuffer buffer_;
  uint16_t size_;
  uint8_t count_;
  uint8_t recvd_;
  uint8_t max_index_;
  uint8_t flags_[kUdpGroupBlockCount];
  UDPEndpoint from_;
};

inline const SharedBuffer& GroupReceiveTask::data() const {
  return buffer_;
}

inline uint16_t GroupReceiveTask::size() const {
  return size_;
}

uint8_t GroupReceiveTask::count() const {
  return count_;
}

inline bool GroupReceiveTask::CompareId(
    uint16_t id, const UDPEndpoint& from) const {
  return (id_ == id && from == from_);
}

inline bool GroupReceiveTask::IsCompleted() const {
  return (status_ == kCompleted);
}

inline bool GroupReceiveTask::IsFailed() const {
  return status_ == kFailed;
}

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_GROUP_RECEIVE_TASK_HPP
