#ifndef MTL_NETWORK_DGRAM_GROUP_SEND_TASK_HPP
#define MTL_NETWORK_DGRAM_GROUP_SEND_TASK_HPP
#include <boost/noncopyable.hpp>
#include "mtl/mtl.hpp"
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/shared_buffer.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;

namespace udp {

class Dgram;
class GroupSendTask : public boost::noncopyable {
public:
  GroupSendTask(Dgram* s, const SharedBuffer& buffer, uint16_t size,
                const UdpEndpoint& to, const TimePoint& end_time);
  ~GroupSendTask();

  inline uint16_t id() const;
  inline const UdpEndpoint& to() const;
  inline const TimePoint& end_time() const;

  int32_t cost() const;
  inline const SharedBuffer& buffer() const;
  inline uint16_t size() const;

  inline bool IsCompleted() const;
  inline bool IsFailed() const;
  inline bool IsSending() const;
  inline bool IsWaiting() const;

  void HandleSendComplete(const TimePoint& now);
  void HandleReceivePacket(InRequest& ireq,
                           const UdpEndpoint& from);
  void HandleTimeout(const TimePoint& now);

private:
  void SendBlock(uint8_t index, const TimePoint& now);
  uint8_t FirstLostIndex() const;

  enum Status {
    kIdle,
    kWaitingAck,
    kSending,
    kWaitingSending,
    kWaitingReport,
    kCompleted,
    kFailed,
  };

  Dgram* dgram_;
  uint16_t id_;
  SharedBuffer buffer_;
  uint16_t size_;
  uint8_t count_;
  UdpEndpoint to_;
  TimePoint create_time_;
  TimePoint end_time_;
  uint8_t flags_[kUdpGroupBlockCount];
  Status status_;
  uint8_t pos_;
  uint8_t recvd_;
  uint8_t window_;
  uint8_t window_pos_;
  TimePoint window_time_;
  TimePoint send_time_;
  static std::map<std::string, uint8_t> windows;
};

inline uint16_t GroupSendTask::id() const {
  return id_;
}

inline const UdpEndpoint& GroupSendTask::to() const {
  return to_;
}

inline const TimePoint& GroupSendTask::end_time() const {
  return end_time_;
}

inline const SharedBuffer& GroupSendTask::buffer() const {
  return buffer_;
}

inline uint16_t GroupSendTask::size() const {
  return size_;
}

inline bool GroupSendTask::IsCompleted() const {
  return status_ == kCompleted;
}

inline bool GroupSendTask::IsFailed() const {
  return status_ == kFailed;
}

inline bool GroupSendTask::IsSending() const {
  return (status_ == kSending);
}

inline bool GroupSendTask::IsWaiting() const {
  return (status_ == kWaitingAck || status_ == kWaitingSending
          || status_ == kWaitingReport);
}

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_GROUP_SEND_TASK_HPP
