//
// connection.hpp
// ~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MTL_NETWORK_CONNECTION_HPP
#define MTL_NETWORK_CONNECTION_HPP
#include <chrono>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/signals2/signal.hpp>
#include "mtl/mtl.hpp"
#include "mtl/network/detail/byte_stream.hpp"
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;
class OutRequest;

namespace tcp {

/// Represents a single connection from a client.
class MTL_EXPORT Connection
    : public boost::enable_shared_from_this<Connection>
    , private boost::noncopyable {
public:
  /// signals
  boost::signals2::signal<void (InRequest&)> packet_arrival_signal;
  boost::signals2::signal<void (const boost::system::error_code&)> close_signal;

  /// Construct a connection with the given io_service.
  explicit Connection(boost::asio::io_service& io_service);

  /// Get the socket address
  TCPEndpoint LocalEndpoint() const;
  TCPEndpoint RemoteEndpoint() const;

  /// Get the socket associated with the connection.
  inline TCPSocket& socket() { return socket_; }

  /// Return whether the connection is connected
  inline bool IsConnected() const { return connected_; }

  /// Return whether the connection is busy.
  inline bool IsBusy() const { return (send_count_ > (sent_count_ + 5)); }
  inline int PendingCount() const { return (send_count_ - sent_count_); }

  /// Return the last request time
  inline std::chrono::system_clock::time_point last_request_time() const {
    return last_request_time_;
  }

  /// Start the first asynchronous operation for the connection.
  void Start();

  /// Aync send
  void Send(OutRequest& request);

  /// close
  void Close();

private:
  /// Handle completion of a read operation.
  void HandleRead(const boost::system::error_code& ec, std::size_t bytes_transferred);

  /// Handle completion of a write operation.
  void HandleWrite(SharedBuffer buffer, const boost::system::error_code& ec,
                   std::size_t bytes_transferred);

  /// Handle packet arrival
  void HandlePacketArrival(const SharedBuffer& buffer, size_t len);

  /// Is an intact request packet.
  bool IsIntactRequest(boost::int16_t& req_length);

  /// Socket for the connection.
  TCPSocket socket_;

  /// Connected flag
  bool connected_;

  /// Buffer for incoming data.
  SharedBuffer buffer_;

  /// Received byte stream
  ByteStream stream_;

  /// Stat the send & sent count
  int send_count_;
  int sent_count_;
  std::chrono::system_clock::time_point last_request_time_;
};

typedef boost::shared_ptr<Connection> ConnectionPtr;

} // namespace tcp
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_CONNECTION_HPP
