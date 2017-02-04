#ifndef MTL_NETWORK_DGRAM_GROUP_RECV_TASK_HPP
#define MTL_NETWORK_DGRAM_GROUP_RECV_TASK_HPP
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

class GroupRecvTask : public boost::noncopyable
{
public:
    enum Status
    {
        WAITING,
        COMPLETED,
        FAILED,
    };

    GroupRecvTask(Dgram* s, uint16_t id, const boost::asio::ip::udp::endpoint& from);

    int32_t cost() const;
    inline const SharedBuffer& data() const { return buffer_; }
    inline uint16_t size() const { return size_; }
    inline uint16_t blockCount() const { return count_; }

    inline bool compareId(uint16_t id, const boost::asio::ip::udp::endpoint& from) const
    {
        return (id_ == id && from == from_);
    }

    inline bool isCompleted() const { return status_ == COMPLETED; }
    inline bool isFailed() const { return status_ == FAILED; }

    inline const boost::asio::ip::udp::endpoint& from() const { return from_; }

    void handleReceivePacket(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleTimeout(const boost::posix_time::ptime& now);

private:
    uint16_t lostCount() const;
    void report();

    Dgram* dgram_;
    uint16_t id_;
    boost::posix_time::ptime create_time_;
    boost::posix_time::ptime end_time_;
    boost::posix_time::ptime report_time_;
    bool inited_;
    Status status_;
    SharedBuffer buffer_;
    uint16_t size_;
    uint16_t count_;
    uint16_t recvd_;
    uint16_t max_index_;
    uint8_t flags_[UDP_GROUP_BLOCK_COUNT];
    boost::asio::ip::udp::endpoint from_;
};

} // dgram
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_GROUP_RECV_TASK_HPP
