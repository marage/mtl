#include <boost/asio/io_service.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/bind.hpp>
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/io_service_pool.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/p2p/server.hpp"
#include <iostream>
#include <Windows.h>
#include <boost/lexical_cast.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

using namespace mtl::network;

void on_packet_arrival(mtl::network::InRequest& ireq, const boost::asio::ip::udp::endpoint& from)
{
    std::cout << ireq.size() << std::endl;
}

int main(int argc, char* argv[])
{
    SingletonBufferPool buffer_pool;
    IOServicePool io_services(1);
    boost::shared_ptr<udp::Dgram> dgram(new udp::Dgram(io_services.getIOService()));
    boost::shared_ptr<p2p::Server> server(new p2p::Server(dgram, "123456"));

    boost::asio::ip::address addr = boost::asio::ip::address::from_string("192.168.151.84");
    boost::asio::ip::udp::endpoint endpoint(addr, 6105);
    server->open(endpoint, 250);

    server->packet_arrival_signal.connect(boost::bind(&on_packet_arrival, _1, _2));

    io_services.run();

    DWORD now = ::GetTickCount();
    while (1) {
        if (GetTickCount() >= now + 6000000)
            break;
        ::Sleep(1000);
    }

    dgram->close();

    return 0;
}
