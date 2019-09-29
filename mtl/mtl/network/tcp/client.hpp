#ifndef MTL_CLIENT_HPP
#define MTL_CLIENT_HPP
#include "connection.hpp"

namespace mtl {
namespace network {
namespace tcp {

class MTL_EXPORT Client
        : public boost::enable_shared_from_this<Client>
        , private boost::noncopyable
{
public:
    enum Status
    {
        kUnconnected,
        kConnecting,
        kConnected,
    };

    /// signals
    boost::signals2::signal<void(const boost::system::error_code&)> new_connection_signal;
    boost::signals2::signal<void(InRequest&)> packet_arrival_signal;
    boost::signals2::signal<void(const boost::system::error_code&)> close_signal;

    explicit Client(boost::asio::io_context& context);
    ~Client();

    const std::string& serverHost() const { return server_host_; }
    const std::string& serverPort() const { return server_port_; }
    TcpEndpoint localEndpoint() const;
    bool isConnected() const;
    bool isBusy() const;
    int pendingCount() const;
    std::chrono::system_clock::time_point lastRequestTime() const;

    bool open(const std::string& host, const std::string& port);
    void send(OutRequest& oreq);
    void close();

private:
    void handleConnect(const boost::system::error_code& ec);

    Status status_;
    boost::asio::io_context& context_;
    std::string server_host_;
    std::string server_port_;
    ConnectionPtr connection_;
};

typedef boost::shared_ptr<Client> ClientPtr;

} // tcp
} // network
} // mtl

#endif // CLIENT_HPP
