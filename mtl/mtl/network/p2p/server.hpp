#ifndef MTL_NETWORK_P2P_SERVER_H
#define MTL_NETWORK_P2P_SERVER_H
#include <set>
#include <boost/shared_ptr.hpp>
#include "client.hpp"
#include "mtl/network/p2p/detail/graph.hpp"

namespace mtl {
namespace network {
namespace p2p {

class _MTL_EXPORT Server : public Client
{
public:
    Server(const udp::dgram_ptr& dgram, const std::string& token);

    bool open(const boost::asio::ip::udp::endpoint& endpoint, uint32_t frequency) override;
    void close() override;

    inline bool isClient(const boost::asio::ip::udp::endpoint& endpoint) const;

    void broadcast(OutRequest& oreq, int32_t timeout = 5000);

private:
    void handlePacketArrival(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                             int32_t milliseconds);
    void handleGroupHeadArrival(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                                bool* passed);

    void handleJoinRequest(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleLeaveRequest(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleFailedReportRequest(InRequest& ireq, const boost::asio::ip::udp::endpoint& from);
    void handleCheckAliveTimeout(const boost::system::error_code& e);
    void handleAlive(const boost::asio::ip::udp::endpoint& from);
    void handleBroadcastData(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                             int32_t milliseconds);

    SequenceRange getUsableSequenceRange(uint64_t size);

    struct NearVertex
    {
        GraphVertex* vertex;
        int weight;
    };
    std::list<NearVertex> nearMembers(const std::string& center, uint32_t max_count);
    void removeMember(const std::string& name);
    void removeMember(const GraphVertex* v);
    void replaceNeighbor(const GraphVertex* v, const boost::asio::ip::udp::endpoint& neighbor);
    bool isValidNeighbor(const GraphVertex* v);
    void bfsMembers(const boost::asio::ip::udp::endpoint& start,
                    std::list<boost::asio::ip::udp::endpoint>& targets);

    std::string token_;
    Graph graph_;

    std::list<SequenceRange> usable_sequences_;
    SequenceRange members_seq_range_;
    uint64_t next_member_seq_;

    boost::asio::deadline_timer check_alive_timer_;

    friend class BroadcastClientsTask;
};

typedef boost::shared_ptr<Server> server_ptr;

bool Server::isClient(const boost::asio::ip::udp::endpoint& endpoint) const
{
    return graph_.containsVertex(toString(endpoint));
}

} // namespace p2p
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_P2P_SERVER_H
