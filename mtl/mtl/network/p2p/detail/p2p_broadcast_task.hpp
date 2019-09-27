#ifndef MTL_NETWORK_P2P_BROADCAST_TASK_HPP
#define MTL_NETWORK_P2P_BROADCAST_TASK_HPP
#include <list>
#include <boost/cstdint.hpp>
#include <boost/asio/ip/udp.hpp>
#include "mtl/network/out_request.hpp"
#include "mtl/task/task.hpp"

namespace mtl {
namespace network {
namespace p2p {

class Client;
class Server;

class BroadcastNeighborsTask final : public Task
{
public:
    BroadcastNeighborsTask(Client* c, const OutRequest& oreq, int timeout,
                           const boost::asio::ip::udp::endpoint& from);

private:
    Status processImpl();

    Client* client_;
    OutRequest request_;
    boost::asio::ip::udp::endpoint from_;
    int next_pos_;
};

class BroadcastClientsTask final : public Task
{
public:
    BroadcastClientsTask(Server* s, const OutRequest& oreq, int timeout,
                         const boost::asio::ip::udp::endpoint& from);

private:
    void activateImpl();
    Status processImpl();

    Server* server_;
    OutRequest request_;
    boost::asio::ip::udp::endpoint from_;
    std::list<boost::asio::ip::udp::endpoint> targets_;
    int next_pos_;
};

} // p2p
} // network
} // mtl

#endif // MTL_NETWORK_P2P_BROADCAST_TASK_HPP
