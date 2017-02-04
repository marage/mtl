#ifndef MTL_NETWORK_DGRAM_HPP
#define MTL_NETWORK_DGRAM_HPP
#include <boost/noncopyable.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <map>
#include "mtl/network/shared_buffer.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/udp/detail/dgram_packet_record.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;

namespace udp {

/// forward classes
class GroupRecvTask;
class GroupSendTask;

/// Not support thread
class _MTL_EXPORT Dgram
    : public boost::enable_shared_from_this<Dgram>, private boost::noncopyable
{
public:
    /// signals
    boost::signals2::signal<void(const OutRequest&, const boost::asio::ip::udp::endpoint&, int32_t)> sent_signal;
    boost::signals2::signal<void(InRequest&, const boost::asio::ip::udp::endpoint&, int32_t)> arrival_signal;
    boost::signals2::signal<void(const OutRequest&, const boost::asio::ip::udp::endpoint&)> timeout_signal;
    boost::signals2::signal<void(const boost::asio::ip::udp::endpoint&)> keep_alive_signal;
    boost::signals2::signal<void(InRequest&, const boost::asio::ip::udp::endpoint&, bool*)> group_head_arrival_signal;
    boost::signals2::signal<void()> timer_tick_signal;

    explicit Dgram(boost::asio::io_service& io_service);
    ~Dgram();

    bool open(const boost::asio::ip::udp::endpoint& endpoint, uint32_t frequency = 1000);
    void close();

    boost::asio::io_service& getIOService();
    boost::asio::ip::udp::endpoint localEndpoint() const;
    bool isBusy();
    bool sendTo(const OutRequest& oreq, const boost::asio::ip::udp::endpoint& to, uint32_t timeout = 5000);
    void keepAlive(const boost::asio::ip::udp::endpoint& to, bool alive = true);
    void clearPackets(const boost::asio::ip::udp::endpoint& to);
    void clearPackets();

    static uint32_t nextSequence();

private:
    void sendNextPacket();
    void sendPacketAck(uint32_t seq, const boost::asio::ip::udp::endpoint& to);

    void asyncReceiveFrom();
    void asyncSendTo(OutRequest& oreq, const boost::asio::ip::udp::endpoint& to, uint32_t timeout);

    void handleReceiveFrom(const boost::system::error_code& error, size_t bytes_transferred);
    void handleReceiveFinished(size_t bytes_recvd);
    void handlePacketAck(InRequest& ireq);
    void handleReceiveGroupPacket(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleReceiveGroupPacketAck(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleSendTo(const SharedBuffer& buffer, const boost::system::error_code& error,
                      size_t bytes_transferred);
    void handleTimeout(const boost::system::error_code& e);
    void handleClose();

    void handleSendComplete();
    void handleReceivePacket(const SharedBuffer& data, size_t size, const boost::asio::ip::udp::endpoint& from);

    bool destIsRecving(const boost::asio::ip::udp::endpoint& dest);

    GroupRecvTask* getRecvTask(uint16_t id, const boost::asio::ip::udp::endpoint& from);

    struct SmallPacket
    {
        OutRequest oreq;
        boost::asio::ip::udp::endpoint to;
        boost::posix_time::ptime end_time;
    };

    struct PendingPacket
    {
        OutRequest oreq;
        boost::asio::ip::udp::endpoint to;
        boost::posix_time::ptime end_time;
        boost::posix_time::ptime next_time;
    };

    boost::asio::ip::udp::socket socket_;
    boost::asio::deadline_timer timer_;

    bool open_;
    bool closing_;
    bool alive_;
    boost::asio::ip::udp::endpoint alive_endpoint_;
    boost::posix_time::ptime last_alive_time_;
    int64_t timer_frequency_;

    std::list<SmallPacket> small_packet_queue_;
    std::map<uint32_t, PendingPacket> pending_packets_;
    std::list<GroupSendTask*> group_send_queue_;
    std::list<GroupRecvTask*> active_group_recv_tasks_;
    std::list<GroupSendTask*> active_group_send_tasks_;

    PacketRecord packet_record_;
    GroupRecord group_record_;

    boost::mutex mutex_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    SharedBuffer buffer_;

    friend class GroupRecvTask;
    friend class GroupSendTask;
};

typedef boost::shared_ptr<Dgram> dgram_ptr;

} // udp
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_HPP
