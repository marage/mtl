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

class MTL_EXPORT Server : public Client {
public:
  Server(const udp::DgramPtr& dgram, const std::string& token);

  bool Open(const UdpEndpoint& endpoint) override;
  void Close() override;

  inline bool IsClient(const UdpEndpoint& endpoint) const;

  void Broadcast(OutRequest& oreq, int32_t timeout = 5000);

private:
  void HandlePacketArrival(InRequest& ireq, const UdpEndpoint& from,
                           int32_t milliseconds);
  void HandleGroupHeadArrival(InRequest& ireq, const UdpEndpoint& from,
                              bool* passed);

  void HandleJoinRequest(InRequest& ireq, const UdpEndpoint& from);
  void HandleLeaveRequest(InRequest& ireq, const UdpEndpoint& from);
  void HandleFailedReportRequest(InRequest& ireq, const UdpEndpoint& from);
  void HandleCheckAliveTimeout(const boost::system::error_code& e);
  void HandleAlive(const UdpEndpoint& from);
  void HandleBroadcastData(InRequest& ireq, const UdpEndpoint& from,
                           int32_t milliseconds);

  SequenceRange GetUsableSequenceRange(uint64_t size);

  struct NearVertex {
    GraphVertex* vertex;
    int weight;
  };
  std::list<NearVertex> NearMembers(const std::string& center,
                                    uint32_t max_count);
  void RemoveMember(const std::string& name);
  void RemoveMember(const GraphVertex* v);
  void ReplaceNeighbor(const GraphVertex* v, const UdpEndpoint& neighbor);
  bool IsValidNeighbor(const GraphVertex* v);
  void BfsMembers(const UdpEndpoint& start, std::list<UdpEndpoint>& targets);

  std::string token_;
  Graph graph_;

  std::list<SequenceRange> usable_sequences_;
  SequenceRange members_seq_range_;
  uint64_t next_member_seq_;

  boost::asio::deadline_timer check_alive_timer_;

  friend class BroadcastClientsTask;
};

inline bool Server::IsClient(const UdpEndpoint& endpoint) const {
  return graph_.ContainsVertex(udp::Dgram::ToString(endpoint));
}

typedef boost::shared_ptr<Server> ServerPtr;

} // namespace p2p
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_P2P_SERVER_H
