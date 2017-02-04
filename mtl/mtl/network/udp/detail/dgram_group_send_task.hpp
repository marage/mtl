#ifndef MTL_NETWORK_DGRAM_GROUP_SEND_TASK_HPP
#define MTL_NETWORK_DGRAM_GROUP_SEND_TASK_HPP
#include <boost/noncopyable.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/shared_buffer.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;

namespace udp {

class Dgram;
class GroupSendTask : public boost::noncopyable
{
public:
    GroupSendTask(Dgram* s, const SharedBuffer& buffer, uint16_t size,
                  const boost::asio::ip::udp::endpoint& to,
                  const boost::posix_time::ptime& end_time);

    inline uint16_t id() const { return id_; }
    inline const boost::asio::ip::udp::endpoint& to() const { return to_; }
    inline const boost::posix_time::ptime& end_time() const { return end_time_; }

    int32_t cost() const;
    inline const SharedBuffer& buffer() const { return buffer_; }
    inline uint16_t size() const { return size_; }

    inline bool isCompleted() const { return status_ == COMPLETED; }
    inline bool isFailed() const { return status_ == FAILED; }

    void handleSendComplete();
    void handleReceivePacket(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleTimeout(const boost::posix_time::ptime& now);

private:
    void sendBlock(uint16_t index);
    uint16_t firstLostIndex() const;

    enum Status
    {
        IDLE,
        SENDING,
        WAITING_SENDING,
        WAITING_REPORT,
        COMPLETED,
        FAILED,
    };

    Dgram* dgram_;
    uint16_t id_;
    SharedBuffer buffer_;
    uint16_t size_;
    uint16_t count_;
    boost::asio::ip::udp::endpoint to_;
    boost::posix_time::ptime create_time_;
    boost::posix_time::ptime end_time_;
    uint8_t flags_[UDP_GROUP_BLOCK_COUNT];
    Status status_;
    uint16_t pos_;
    uint16_t recvd_;
    uint16_t window_;
    uint16_t window_pos_;
    boost::posix_time::ptime window_time_;
    boost::posix_time::ptime send_time_;
};

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_GROUP_SEND_TASK_HPP
