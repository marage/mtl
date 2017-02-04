#include <unordered_set>
#include <boost/asio/io_service.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio/ip/address.hpp>
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/io_service_pool.hpp"
#include "mtl/network/p2p/client.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include <Windows.h>

using namespace mtl::network;

p2p::Client* the_client;
void send_test_data()
{
    static int pts = 10;
    mtl::network::OutRequest oreq(100, 100, p2p::P2P_DATA_HEADER_LENGTH);
    if (pts % 10) {
        char szA[500];
        memset(szA, '+', 498);
        szA[499] = '\0';
        oreq.writeString(szA);
    }
    else {
        char szA[4096];
        memset(szA, '*', 4095);
        szA[4095] = '\0';
        oreq.writeString(szA);
    }
    the_client->broadcast(oreq, (oreq.size() >= 1000 ? 1000 : 250));
}

void on_packet_arrival(mtl::network::InRequest& ireq, const boost::asio::ip::udp::endpoint& from)
{
    std::cout << "received data: " << ireq.readCommand()
        << "\t" << ireq.size() << "\t" << from.port() << std::endl;
}

void on_leave()
{
    std::cout << "leave" << std::endl;
}

int main(int argc, char* argv[])
{
    SingletonBufferPool buffer_pool;
    IOServicePool io_services(1);
    boost::shared_ptr<udp::Dgram> dgram(new udp::Dgram(io_services.getIOService()));
    boost::shared_ptr<p2p::Client> client(new p2p::Client(dgram));
    client->join_finished_signal.connect(boost::bind(&send_test_data));
    client->packet_arrival_signal.connect(boost::bind(&on_packet_arrival, _1, _2));
    client->packet_arrival_signal.connect(boost::bind(&on_leave));
    the_client = client.get();

    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address_v4::any(), 0);
    client->open(endpoint, 500);
    std::cout << client->localEndpoint().port() << std::endl;

    boost::asio::ip::address addr = boost::asio::ip::address::from_string("192.168.151.84");
    boost::asio::ip::udp::endpoint server_endpoint(addr, 6105);
    client->join(server_endpoint, "123456");

    DWORD now;
    try {
        io_services.run();
        now = ::GetTickCount();
        while (1) {
            if (GetTickCount() >= now + 600000)
                break;
            ::Sleep(40);
            send_test_data();
        }
    }
    catch (...) {
        std::cout << "exception" << std::endl;
    }

    dgram->close();

    return 0;
}
