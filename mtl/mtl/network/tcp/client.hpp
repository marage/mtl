#ifndef MTL_CLIENT_HPP
#define MTL_CLIENT_HPP
#include <chrono>
#include "connection.hpp"

namespace mtl {
namespace network {
namespace tcp {

class _MTL_EXPORT Client
    : public boost::enable_shared_from_this<Client>, private boost::noncopyable
{
public:
    enum Status
    {
        UNCONNECTED,
        CONNECTING,
        CONNECTED,
    };

    /// signals
    boost::signals2::signal<void(const boost::system::error_code&)> new_connection_signal;
    boost::signals2::signal<void(InRequest&)> packet_arrival_signal;
    boost::signals2::signal<void(const boost::system::error_code&)> close_signal;

    explicit Client(boost::asio::io_service& io_service);
    ~Client();

    boost::asio::ip::tcp::endpoint localEndpoint() const;
    bool isConnected() const;
    bool isBusy() const;
    int pendingCount() const;
    std::chrono::system_clock::time_point lastRequestTime() const;

    bool open(const std::string& server, const std::string& port);
    void send(OutRequest& oreq);
    void close();

private:
    void handleConnect(const boost::system::error_code& ec);

    Status status_;
    boost::asio::io_service& io_service_;
    connection_ptr connection_;
};

typedef boost::shared_ptr<Client> client_ptr;

} // tcp
} // network
} // mtl

#endif // CLIENT_HPP
