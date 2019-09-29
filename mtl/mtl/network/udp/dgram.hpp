#ifndef MTL_NETWORK_DGRAM_HPP
#define MTL_NETWORK_DGRAM_HPP
#include <map>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/asio/io_service.hpp>
#include "mtl/network/protocol.hpp"
#include "mtl/network/shared_buffer.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/udp/detail/dgram_packet_record.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;

namespace udp {

/// forward classes
class ReceiveGroupTask;
class SendGroupTask;

/// Not support thread
class MTL_EXPORT Dgram
        : public boost::enable_shared_from_this<Dgram>
        , private boost::noncopyable
{
public:
    /// signals
    boost::signals2::signal<void(const OutRequest&, const UdpEndpoint&, int32_t)> sent_signal;
    boost::signals2::signal<void(InRequest&, const UdpEndpoint&, int32_t)> arrival_signal;
    boost::signals2::signal<void(const OutRequest&, const UdpEndpoint&)> timeout_signal;
    boost::signals2::signal<void(const UdpEndpoint&)> keep_alive_signal;
    boost::signals2::signal<void(InRequest&, const UdpEndpoint&, bool*)> group_head_arrival_signal;
    boost::signals2::signal<void()> tick_signal;

    explicit Dgram(boost::asio::io_context& context);
    ~Dgram();

    bool open(const UdpEndpoint& endpoint);
    void close();

    boost::asio::io_context& context() { return context_; }
    UdpEndpoint localEndpoint() const;
    bool isBusy();
    bool sendTo(const OutRequest& oreq, const UdpEndpoint& to,
                uint32_t timeout = 5000);
    void keepAlive(const UdpEndpoint& to, bool alive = true);
    void clearPackets(const UdpEndpoint& to);
    void clearPackets();

    static uint32_t nextSequence();
    static std::string toString(const UdpEndpoint& endpoint);

private:
    bool destIsReceiving(const UdpEndpoint& dest) const;
    ReceiveGroupTask* getReceiveGroupTask(uint16_t id, const UdpEndpoint& from) const;

    void asyncReceiveFrom();
    void asyncSendTo(OutRequest& oreq, const UdpEndpoint& to, uint32_t timeout);

    void handleReceiveFrom(const boost::system::error_code& error,
                           size_t bytes_transferred);
    void handleSendTo(const SharedBuffer& buffer,
                      const boost::system::error_code& error,
                      size_t bytes_transferred);
    void handleClose();

    void sendPacketAck(uint32_t seq, const UdpEndpoint& to);
    bool sendNextPacket();
    void handleReceiveComplete(const SharedBuffer& buffer, size_t bytes_recvd,
                               const UdpEndpoint& from);
    void handlePacketAck(InRequest& ireq);
    void handleReceiveGroupPacket(InRequest& ireq, const UdpEndpoint& from);
    void handleReceiveGroupPacketAck(InRequest& ireq, const UdpEndpoint& from);

    void tick();
    bool processEvents();
    void mainLoop();

    struct SmallPacket
    {
        OutRequest oreq;
        UdpEndpoint to;
        std::chrono::system_clock::time_point end_time;
    };

    struct PendingPacket
    {
        OutRequest oreq;
        UdpEndpoint to;
        std::chrono::system_clock::time_point end_time;
        std::chrono::system_clock::time_point next_time;
    };

    struct Event
    {
        enum
        {
            kReceiveComplete = 1,
            kSendComplete,
            kSendNextPacket
        };

        explicit Event(int t) : type(t)
        {
        }
        Event(int t, const UdpEndpoint& ep, const SharedBuffer& sb, size_t sz)
            : type(t), endpoint(ep), buffer(sb), size(sz)
        {
        }

        int type;
        UdpEndpoint endpoint;
        SharedBuffer buffer;
        size_t size;
    };

    boost::asio::io_context& context_;
    boost::asio::ip::udp::socket socket_;
    std::thread* thread_;
    bool running_;
    bool open_;
    bool closing_;
    bool alive_;
    UdpEndpoint alive_endpoint_;
    std::chrono::system_clock::time_point last_alive_time_;
    std::chrono::system_clock::time_point now_;

    std::list<SmallPacket> small_packets_;
    std::map<uint32_t, PendingPacket> pending_packets_;
    std::list<SendGroupTask*> inactive_send_group_tasks_;
    std::list<ReceiveGroupTask*> active_receive_group_tasks_;
    std::list<SendGroupTask*> active_send_group_tasks_;

    PacketRecord packet_record_;
    GroupRecord group_record_;

    size_t available_send_buffer_size_;
    UdpEndpoint sender_endpoint_;
    SharedBuffer buffer_;
    std::mutex mutex_;
    std::list<Event> events_;
    std::mutex events_mutex_;
    std::condition_variable events_cv_;

    friend class ReceiveGroupTask;
    friend class SendGroupTask;
};

inline std::string Dgram::toString(const UdpEndpoint& endpoint)
{
    std::string s;
    s.append(endpoint.address().to_string());
    std::stringstream ss;
    ss << endpoint.port();
    s.append(ss.str());
    return s;
}

typedef boost::shared_ptr<Dgram> DgramPtr;

} // udp
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_HPP
