#ifndef MTL_NETWORK_P2P_BROADCAST_TASK_HPP
#define MTL_NETWORK_P2P_BROADCAST_TASK_HPP
#include <list>
#include <boost/cstdint.hpp>
#include "mtl/network/out_request.hpp"
#include "mtl/task/task.hpp"
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {
namespace p2p {

class Client;
class Server;

class BroadcastNeighborsTask final : public Task
{
public:
  BroadcastNeighborsTask(Client* c, const OutRequest& oreq, int timeout,
                         const UdpEndpoint& from);

private:
  State processImpl() override;

  Client* client_;
  OutRequest request_;
  UdpEndpoint from_;
  int next_pos_;
};

class BroadcastClientsTask final : public Task
{
public:
  BroadcastClientsTask(Server* s, const OutRequest& oreq, int timeout,
                       const UdpEndpoint& from);

private:
  void activateImpl() override;
  State processImpl() override;

  Server* server_;
  OutRequest request_;
  UdpEndpoint from_;
  std::list<UdpEndpoint> targets_;
  int next_pos_;
};

} // p2p
} // network
} // mtl

#endif // MTL_NETWORK_P2P_BROADCAST_TASK_HPP
