#ifndef MTL_NETWORK_P2P_CLIENT_H
#define MTL_NETWORK_P2P_CLIENT_H
#include <list>
#include <sstream>
#include "mtl/task/task_group.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include "mtl/network/p2p/detail/p2p_group_packet_filter.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;

namespace p2p {

class MTL_EXPORT Client
        : public boost::enable_shared_from_this<Client>
        , private boost::noncopyable
{
public:
    /// signals
    boost::signals2::signal<void()> join_finished_signal;
    boost::signals2::signal<void()> join_failed_signal;
    boost::signals2::signal<void()> leave_signal;
    boost::signals2::signal<void(InRequest& ireq, const UdpEndpoint& from)> packet_arrival_signal;
    boost::signals2::signal<void(const OutRequest& oreq, const UdpEndpoint& to)> packet_timeout_signal;

    enum Role
    {
        kServerRole,
        kClientRole
    };

    enum State
    {
        kIdleState,
        kJoiningState,
        kJoined,
    };

    explicit Client(const udp::DgramPtr& dgram, Role role = kClientRole);
    virtual ~Client();

    virtual bool open(const UdpEndpoint& endpoint);
    virtual void close();

    bool isMember() const;
    Role role() const;
    UdpEndpoint localEndpoint() const;
    bool isNeighbor(const UdpEndpoint& endpoint) const;

    void join(const UdpEndpoint& to, const std::string& access_token);
    void broadcast(OutRequest& oreq, int32_t timeout = 5000);
    void sendTo(OutRequest& oreq, const UdpEndpoint& to,
                uint32_t timeout = 5000);
    void leave();

    uint64_t genSeq() const;
    int32_t costMilliseconds(uint16_t size) const;

    static const char* toBytes(const UdpEndpoint& endpoint, char bytes[]);
    static UdpEndpoint toEndpoint(char bytes[16], uint16_t port);

protected:
    void handlePacketArrival(InRequest& ireq, const UdpEndpoint& from,
                             int32_t milliseconds);
    void handleGroupHeadArrival(InRequest& ireq, const UdpEndpoint& from,
                                bool* passed);
    void handlePacketSent(const OutRequest& oreq, const UdpEndpoint& to,
                          int32_t milliseconds);
    void handlePacketTimeout(const OutRequest& oreq, const UdpEndpoint& to);

    void handleJoinAck(InRequest& ireq, const UdpEndpoint& from);
    void handleAddNeighbor(InRequest& ireq, const UdpEndpoint& from);
    void handleReplaceNeighbor(InRequest& ireq, const UdpEndpoint& from);
    void handleBroadcastData(InRequest& ireq, const UdpEndpoint& from,
                             int32_t milliseconds);
    void handleBroadcastDataTimeout(const OutRequest& oreq,
                                    const UdpEndpoint& to);
    void handleTimerTick();

    void removeNeighbor(const UdpEndpoint& address);

    void appendTask(const mtl::TaskPtr& t);
    void activateTempTasks();
    void processTasks();
    void clearTempTasks();
    void statCost(const UdpEndpoint& from, uint16_t size,
                  int32_t milliseconds);

    struct Neighbor
    {
        uint8_t role;
        UdpEndpoint address;
        int32_t sent_count;
        int32_t failure_times;
        int32_t cost_per_k;

        double failureRate() const
        {
            return (failure_times > 5 ? double(failure_times)/double(sent_count)
                                      : 0);
        }

        int32_t cost(uint16_t size) const
        {
            return (size < 1024 ? cost_per_k
                                : int32_t(cost_per_k*(double(size)/1024.0)));
        }

        friend bool operator==(const Neighbor& a, const Neighbor& b)
        {
            return a.address == b.address;
        }
        friend bool operator!=(const Neighbor& a, const Neighbor& b)
        {
            return a.address != b.address;
        }
    };

    mtl::network::udp::DgramPtr dgram_;
    std::string access_token_;
    UdpEndpoint server_endpoint_;
    bool timeout_;
    std::chrono::system_clock::time_point timeout_end_time_;
    SequenceRange seq_range_;
    std::list<Neighbor> neighbors_;
    Role role_;
    State status_;
    GroupPacketFilter packet_filter_;
    std::list<mtl::TaskPtr> temp_tasks_;
    std::mutex temp_tasks_mutex_;
    mtl::TaskGroup task_group_;

    friend class BroadcastNeighborsTask;
};

inline bool Client::isMember() const
{
    return (status_ == kJoined);
}

inline Client::Role Client::role() const
{
    return role_;
}

inline const char* Client::toBytes(const UdpEndpoint& endpoint, char bytes[])
{
    if (endpoint.address().is_v4()) {
        memcpy(bytes, endpoint.address().to_v4().to_bytes().data(), 4);
        bytes[4] = bytes[5] = bytes[6] = bytes[7] = 0;
        bytes[8] = bytes[9] = bytes[10] = bytes[11] = 0;
        bytes[12] = bytes[13] = bytes[14] = bytes[15] = 0;
    } else {
        memcpy(bytes, endpoint.address().to_v6().to_bytes().data(), 16);
    }
    return bytes;
}

inline UdpEndpoint Client::toEndpoint(char bytes[], uint16_t port)
{
    using namespace boost::asio;
    if (bytes[4] == 0 && bytes[5] == 0 && bytes[6] == 0 && bytes[7] == 0
        && bytes[8] == 0 && bytes[9] == 0 && bytes[10] == 0 && bytes[11] == 0) {
        ip::address_v4::bytes_type bytes_v4;
        memcpy(bytes_v4.data(), bytes, 4);
        return ip::udp::endpoint(ip::address_v4(bytes_v4), port);
    } else {
        boost::asio::ip::address_v6::bytes_type bytes_v6;
        memcpy(bytes_v6.data(), bytes, 16);
        return ip::udp::endpoint(ip::address_v6(bytes_v6), port);
    }
}

typedef boost::shared_ptr<Client> ClientPtr;

} // namespace p2p
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_P2P_CLIENT_H
