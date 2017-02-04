//
// server.hpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MTL_NETWORK_SERVER_HPP
#define MTL_NETWORK_SERVER_HPP
#include <boost/asio/ip/tcp.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include "mtl/mtl.hpp"
#include "mtl/network/io_service_pool.hpp"
#include "connection.hpp"

namespace mtl {
namespace network {
namespace tcp {

/// The top-level class of the TCP server.
class _MTL_EXPORT Server : private boost::noncopyable
{
public:
  /// signals
  boost::signals2::signal<bool(connection_ptr)> new_connection_signal;

  /// Construct the server to listen on the specified TCP address and port.
  explicit Server(IOServicePool& isp);
  ~Server();

  /// Run the server's io_service loop.
  virtual bool open(const std::string& address, unsigned short port);
  virtual bool close();
  
  /// Get the socket address
  boost::asio::ip::tcp::endpoint localEndpoint() const;

private:
  /// Initiate an asynchronous accept operation.
  void startAccept();

  /// Handle completion of an asynchronous accept operation.
  void handleAccept(const boost::system::error_code& e);

  /// The pool of io_service objects used to perform asynchronous operations.
  IOServicePool& io_service_pool_;

  /// Acceptor used to listen for incoming connections.
  boost::asio::ip::tcp::acceptor acceptor_;

  /// The next connection to be accepted.
  connection_ptr new_connection_;

  /// The handler for all incoming requests.
};

typedef boost::shared_ptr<Server> server_ptr;

} // namespace tcp
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_SERVER_HPP
