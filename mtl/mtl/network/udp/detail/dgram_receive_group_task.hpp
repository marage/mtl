#ifndef MTL_NETWORK_DGRAM_RECEIVE_GROUP_TASK_HPP
#define MTL_NETWORK_DGRAM_RECEIVE_GROUP_TASK_HPP
#include <boost/noncopyable.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/shared_buffer.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;

namespace udp {

class Dgram;

class ReceiveGroupTask : public boost::noncopyable
{
public:
    enum Status
    {
        kWaitingBlock,
        kCompletedReceiving,
        kFailed,
    };

    ReceiveGroupTask(Dgram* s, uint16_t id, const UdpEndpoint& from);
    virtual ~ReceiveGroupTask() {}

    int32_t cost() const;
    const SharedBuffer& data() const { return buffer_; }
    uint16_t size() const { return size_; }
    uint16_t blockCount() const { return count_; }

    bool compareId(uint16_t id, const UdpEndpoint& from) const
    {
        return (id_ == id && from == from_);
    }

    bool isCompleted() const { return status_ == kCompletedReceiving; }
    bool isFailed() const { return status_ == kFailed; }

    const boost::asio::ip::udp::endpoint& from() const { return from_; }

    void handleReceivePacket(InRequest& ireq, const UdpEndpoint& from);
    void handleTimeout(const std::chrono::system_clock::time_point& now);

private:
    uint16_t lostCount() const;
    void report();

    Dgram* dgram_;
    uint16_t id_;
    std::chrono::system_clock::time_point create_time_;
    std::chrono::system_clock::time_point end_time_;
    std::chrono::system_clock::time_point report_time_;
    bool inited_;
    Status status_;
    SharedBuffer buffer_;
    uint16_t size_;
    uint16_t count_;
    uint16_t recvd_;
    uint16_t max_index_;
    uint8_t flags_[kUdpGroupBlockCount];
    UdpEndpoint from_;
};

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_RECEIVE_GROUP_TASK_HPP
