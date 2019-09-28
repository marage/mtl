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
class GroupReceiveTask;
class GroupSendTask;

/// Not support thread
class MTL_EXPORT Dgram
        : public boost::enable_shared_from_this<Dgram>
        , private boost::noncopyable {
public:
    /// signals
    boost::signals2::signal<void(const OutRequest&, const UdpEndpoint&, int32_t)> sent_signal;
    boost::signals2::signal<void(InRequest&, const UdpEndpoint&, int32_t)> arrival_signal;
    boost::signals2::signal<void(const OutRequest&, const UdpEndpoint&)> timeout_signal;
    boost::signals2::signal<void(const UdpEndpoint&)> keep_alive_signal;
    boost::signals2::signal<void(InRequest&, const UdpEndpoint&, bool*)> group_head_arrival_signal;
    boost::signals2::signal<void()> tick_signal;

    explicit Dgram(boost::asio::io_context& io_context);
    ~Dgram();

    bool open(const UdpEndpoint& endpoint);
    void close();

    boost::asio::io_service& GetIOService();
    UdpEndpoint LocalEndpoint() const;
    bool IsBusy();
    bool SendTo(const OutRequest& oreq, const UdpEndpoint& to,
                uint32_t timeout = 5000);
    void KeepAlive(const UdpEndpoint& to, bool alive = true);
    void ClearPackets(const UdpEndpoint& to);
    void ClearPackets();

    static uint32_t NextSequence();
    inline static std::string ToString(const UdpEndpoint& endpoint);

private:
    bool DestIsReceiving(const UdpEndpoint& dest) const;
    GroupReceiveTask* GetReceiveTask(uint16_t id, const UdpEndpoint& from) const;

    void AsyncReceiveFrom();
    void AsyncSendTo(OutRequest& oreq, const UdpEndpoint& to, uint32_t timeout);

    void HandleReceiveFrom(const boost::system::error_code& error,
                           size_t bytes_transferred);
    void HandleSendTo(const SharedBuffer& buffer,
                      const boost::system::error_code& error,
                      size_t bytes_transferred);
    void HandleClose();

    void SendPacketAck(uint32_t seq, const UdpEndpoint& to);
    bool SendNextPacket();
    void HandleReceiveComplete(const SharedBuffer& buffer, size_t bytes_recvd,
                               const UdpEndpoint& from);
    void HandlePacketAck(InRequest& ireq);
    void handleReceiveGroupPacket(InRequest& ireq, const UdpEndpoint& from);
    void handleReceiveGroupPacketAck(InRequest& ireq, const UdpEndpoint& from);

    void Tick();
    bool ProcessEvents();
    void MainLoop();

    struct SmallPacket {
        OutRequest oreq;
        UdpEndpoint to;
        TimePoint end_time;
    };

    struct PendingPacket {
        OutRequest oreq;
        UdpEndpoint to;
        TimePoint end_time;
        TimePoint next_time;
    };

    struct Event {
        enum {
            kReceiveComplete = 1,
            kSendComplete,
            kSendNextPacket
        };

        explicit Event(int t) : type(t) {
        }
        Event(int t, const UdpEndpoint& ep, const SharedBuffer& sb, size_t sz)
            : type(t), endpoint(ep), buffer(sb), size(sz) {
        }

        int type;
        UdpEndpoint endpoint;
        SharedBuffer buffer;
        size_t size;
    };

    UdpSocket socket_;
    std::thread* thread_;
    bool running_;
    bool open_;
    bool closing_;
    bool alive_;
    UdpEndpoint alive_endpoint_;
    TimePoint last_alive_time_;
    TimePoint now_;

    std::list<SmallPacket> small_packets_;
    std::map<uint32_t, PendingPacket> pending_packets_;
    std::list<GroupSendTask*> inactive_group_send_tasks_;
    std::list<GroupReceiveTask*> active_group_receive_tasks_;
    std::list<GroupSendTask*> active_group_send_tasks_;

    PacketRecord packet_record_;
    GroupRecord group_record_;

    size_t available_send_buffer_size_;
    UdpEndpoint sender_endpoint_;
    SharedBuffer buffer_;
    std::mutex mutex_;
    std::list<Event> events_;
    std::mutex events_mutex_;
    std::condition_variable events_cv_;

    friend class GroupReceiveTask;
    friend class GroupSendTask;
};

inline std::string Dgram::ToString(const UdpEndpoint& endpoint) {
    std::string s;
    s.append(endpoint.address().to_string());
    std::stringstream ss;
    ss << endpoint.port();
    s.append(ss.str());
    return std::move(s);
}

typedef boost::shared_ptr<Dgram> DgramPtr;

} // udp
} // network
} // mtl

#endif // MTL_NETWORK_DGRAM_HPP
