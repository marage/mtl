#ifndef MTL_NETWORK_DGRAM_SEND_GROUP_TASK_HPP
#define MTL_NETWORK_DGRAM_SEND_GROUP_TASK_HPP
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
class SendGroupTask : public boost::noncopyable
{
public:
    SendGroupTask(Dgram* s, const SharedBuffer& buffer, uint16_t size,
                  const UdpEndpoint& to, const std::chrono::system_clock::time_point& endTime);
    ~SendGroupTask();

    uint16_t id() const;
    const UdpEndpoint& to() const;
    const std::chrono::system_clock::time_point& endTime() const;

    int32_t cost() const;
    const SharedBuffer& buffer() const;
    uint16_t size() const;

    bool isCompleted() const;
    bool isFailed() const;
    bool isSending() const;
    bool isWaiting() const;

    void handleSendComplete(const std::chrono::system_clock::time_point& now);
    void handleReceivePacket(InRequest& ireq, const UdpEndpoint& from);
    void handleTimeout(const std::chrono::system_clock::time_point& now);

private:
    void sendBlock(uint8_t index, const std::chrono::system_clock::time_point& now);
    uint8_t firstLostIndex() const;

    enum Status
    {
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
    std::chrono::system_clock::time_point create_time_;
    std::chrono::system_clock::time_point end_time_;
    uint8_t flags_[kUdpGroupBlockCount];
    Status status_;
    uint8_t pos_;
    uint8_t recvd_;
    uint8_t window_;
    uint8_t window_pos_;
    std::chrono::system_clock::time_point window_time_;
    std::chrono::system_clock::time_point send_time_;
    static std::map<std::string, uint8_t> windows;
};

inline uint16_t SendGroupTask::id() const
{
    return id_;
}

inline const UdpEndpoint& SendGroupTask::to() const
{
    return to_;
}

inline const std::chrono::system_clock::time_point& SendGroupTask::endTime() const
{
    return end_time_;
}

inline const SharedBuffer& SendGroupTask::buffer() const
{
    return buffer_;
}

inline uint16_t SendGroupTask::size() const
{
    return size_;
}

inline bool SendGroupTask::isCompleted() const
{
    return status_ == kCompleted;
}

inline bool SendGroupTask::isFailed() const
{
    return status_ == kFailed;
}

inline bool SendGroupTask::isSending() const
{
    return (status_ == kSending);
}

inline bool SendGroupTask::isWaiting() const
{
    return (status_ == kWaitingAck || status_ == kWaitingSending
            || status_ == kWaitingReport);
}

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_SEND_GROUP_TASK_HPP
