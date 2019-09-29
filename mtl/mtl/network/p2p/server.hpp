#ifndef MTL_NETWORK_P2P_SERVER_H
#define MTL_NETWORK_P2P_SERVER_H
#include <set>
#include <boost/shared_ptr.hpp>
#include <boost/asio/deadline_timer.hpp>
#include "client.hpp"
#include "mtl/network/p2p/detail/graph.hpp"

namespace mtl {
namespace network {
namespace p2p {

class MTL_EXPORT Server : public Client
{
public:
    Server(const udp::DgramPtr& dgram, const std::string& token);

    bool open(const UdpEndpoint& endpoint) override;
    void close() override;

    bool isClient(const UdpEndpoint& endpoint) const;

    void broadcast(OutRequest& oreq, int timeout = 5000);

private:
    void handlePacketArrival(InRequest& ireq, const UdpEndpoint& from,
                             int32_t milliseconds);
    void handleGroupHeadArrival(InRequest& ireq, const UdpEndpoint& from,
                                bool* passed);

    void handleJoinRequest(InRequest& ireq, const UdpEndpoint& from);
    void handleLeaveRequest(InRequest& ireq, const UdpEndpoint& from);
    void handleFailedReportRequest(InRequest& ireq, const UdpEndpoint& from);
    void handleCheckAliveTimeout(const boost::system::error_code& e);
    void handleAlive(const UdpEndpoint& from);
    void handleBroadcastData(InRequest& ireq, const UdpEndpoint& from,
                             int32_t milliseconds);

    SequenceRange usableSequenceRange(uint64_t size);

    struct NearVertex
    {
        GraphVertex* vertex;
        int weight;
    };
    std::list<NearVertex> nearMembers(const std::string& center,
                                      uint32_t max_count);
    void removeMember(const std::string& name);
    void removeMember(const GraphVertex* v);
    void replaceNeighbor(const GraphVertex* v, const UdpEndpoint& neighbor);
    bool isValidNeighbor(const GraphVertex* v);
    void bfsMembers(const UdpEndpoint& start, std::list<UdpEndpoint>& targets);

    std::string token_;
    Graph graph_;

    std::list<SequenceRange> usable_sequences_;
    SequenceRange members_seq_range_;
    uint64_t next_member_seq_;

    boost::asio::deadline_timer check_alive_timer_;

    friend class BroadcastClientsTask;
};

inline bool Server::isClient(const UdpEndpoint& endpoint) const
{
    return graph_.containsVertex(udp::Dgram::toString(endpoint));
}

typedef boost::shared_ptr<Server> ServerPtr;

} // namespace p2p
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_P2P_SERVER_H
