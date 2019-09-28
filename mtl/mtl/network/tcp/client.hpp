#ifndef MTL_CLIENT_HPP
#define MTL_CLIENT_HPP
#include "connection.hpp"

namespace mtl {
namespace network {
namespace tcp {

class MTL_EXPORT Client
    : public boost::enable_shared_from_this<Client>
    , private boost::noncopyable {
public:
  enum Status {
    kUnconnected,
    kConnecting,
    kConnected,
  };

  /// signals
  boost::signals2::signal<void(const boost::system::error_code&)> new_connection_signal;
  boost::signals2::signal<void(InRequest&)> packet_arrival_signal;
  boost::signals2::signal<void(const boost::system::error_code&)> close_signal;

  explicit Client(boost::asio::io_service& io_service);
  ~Client();

  inline const std::string& server_host() const { return server_host_; }
  inline const std::string& server_port() const { return server_port_; }
  TCPEndpoint LocalEndpoint() const;
  bool IsConnected() const;
  bool IsBusy() const;
  int PendingCount() const;
  std::chrono::system_clock::time_point LastRequestTime() const;

  bool Open(const std::string& host, const std::string& port);
  void Send(OutRequest& oreq);
  void Close();

private:
  void HandleConnect(const boost::system::error_code& ec);

  Status status_;
  boost::asio::io_service& io_service_;
  std::string server_host_;
  std::string server_port_;
  ConnectionPtr connection_;
};

typedef boost::shared_ptr<Client> ClientPtr;

} // tcp
} // network
} // mtl

#endif // CLIENT_HPP
