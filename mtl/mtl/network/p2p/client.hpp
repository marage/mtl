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

class MTL_EXPORT Client : public boost::enable_shared_from_this<Client>, private boost::noncopyable
{
public:
    /// signals
    boost::signals2::signal<void()> join_finished_signal;
    boost::signals2::signal<void()> join_failed_signal;
    boost::signals2::signal<void()> leave_signal;
    boost::signals2::signal<void(InRequest& ireq, const boost::asio::ip::udp::endpoint& from)> packet_arrival_signal;
    boost::signals2::signal<void(const OutRequest& oreq, const boost::asio::ip::udp::endpoint& to)> packet_timeout_signal;

    enum Role
    {
        SERVER,
        CLIENT
    };

    enum Status
    {
        NONE_STATUS,
        JOINING,
        JOINED,
    };

    explicit Client(const udp::dgram_ptr& dgram, Role role = CLIENT);
    virtual ~Client();

    virtual bool open(const boost::asio::ip::udp::endpoint& endpoint, uint32_t frequency);
    virtual void close();

    bool isMember() const { return (status_ == JOINED); }
    Role role() const { return role_; }
    boost::asio::ip::udp::endpoint localEndpoint() const;
    bool isNeighbor(const boost::asio::ip::udp::endpoint& endpoint) const;

    void join(const boost::asio::ip::udp::endpoint& to, const std::string& access_token);
    void broadcast(OutRequest& oreq, int32_t timeout = 5000);
    void sendTo(OutRequest& oreq, const boost::asio::ip::udp::endpoint& to,
                uint32_t timeout = 5000);
    void leave();

    uint64_t getSeq() const;
    int32_t costMilliseconds(uint16_t size) const;

    static inline std::string toString(const boost::asio::ip::udp::endpoint& endpoint);
    inline const char* toBytes(const boost::asio::ip::udp::endpoint& endpoint,
                               char bytes[]);
    static boost::asio::ip::udp::endpoint toEndpoint(char bytes[16], uint16_t port);

protected:
    void handlePacketArrival(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                             int32_t milliseconds);
    void handleGroupHeadArrival(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                                bool* passed);
    void handlePacketSent(const OutRequest& oreq, const boost::asio::ip::udp::endpoint& to,
                          int32_t milliseconds);
    void handlePacketTimeout(const OutRequest& oreq,
                             const boost::asio::ip::udp::endpoint& to);

    void handleJoinAck(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleAddNeighbor(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleReplaceNeighbor(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleBroadcastData(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                             int32_t milliseconds);
    void handleBroadcastDataTimeout(const OutRequest& oreq, const boost::asio::ip::udp::endpoint& to);
    void handleTimerTick();

    void removeNeighbor(const boost::asio::ip::udp::endpoint& address);

    void addTask(const mtl::task_ptr& t);
    void activateTempTasks();
    void processTasks();
    void clearTempTasks();
    void statCost(const boost::asio::ip::udp::endpoint& from, uint16_t size, int32_t milliseconds);

    struct Neighbor
    {
        uint8_t role;
        boost::asio::ip::udp::endpoint address;
        int32_t sent_count;
        int32_t failure_times;
        int32_t cost_per_k;

        double failureRate() const
        {
            return (failure_times > 5 ? double(failure_times)/double(sent_count) : 0);
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

    mtl::network::udp::dgram_ptr dgram_;
    std::string access_token_;
    boost::asio::ip::udp::endpoint server_endpoint_;
    bool timeout_;
    boost::posix_time::ptime timeout_end_time_;
    SequenceRange seq_range_;
    std::list<Neighbor> neighbors_;
    Role role_;
    Status status_;
    GroupPacketFilter packet_filter_;
    std::list<mtl::task_ptr> temp_tasks_;
    boost::mutex temp_tasks_mutex_;
    mtl::TaskGroup task_group_;

    friend class BroadcastNeighborsTask;
};

typedef boost::shared_ptr<Client> client_ptr;

inline std::string Client::toString(const boost::asio::ip::udp::endpoint& endpoint)
{
    std::string s;
    s.append(endpoint.address().to_string());
    std::stringstream ss;
    ss << endpoint.port();
    s.append(ss.str());
    return s;
}

const char* Client::toBytes(const boost::asio::ip::udp::endpoint& endpoint, char bytes[])
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

inline boost::asio::ip::udp::endpoint Client::toEndpoint(char bytes[], uint16_t port)
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

} // namespace p2p
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_P2P_CLIENT_H
