#ifndef MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_H
#define MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_H
#include <chrono>
#include "mtl/network/tcp/connection.hpp"
#include "logic_unit.h"

namespace mtl {
namespace framework {
namespace core {

class MTL_EXPORT ConnectionController
    : public LogicUnit<network::tcp::ConnectionPtr> {
public:
  ConnectionController(uint32_t id, network::tcp::ConnectionPtr c);

  inline uint16_t main_command() const { return main_command_; }
  inline uint64_t access_count() const { return access_count_; }
  inline void set_access_count(uint64_t count) {
    access_count_ = count;
  }
  inline bool IsBusy() const { return connection_->IsBusy(); }
  inline const std::chrono::system_clock::time_point& when() const {
    return when_;
  }
  inline TCPEndpoint RemoteEndpoint() const {
    return connection_->RemoteEndpoint();
  }
  inline void Send(OutRequest& oreq) { connection_->Send(oreq); }
  inline void Close() { connection_->Close(); }

protected:
  inline void set_main_command(uint16_t cmd) { main_command_ = cmd; }

private:
  void HandleArrival(InRequest& ireq);
  void HandleClose(const boost::system::error_code& ec);

  uint16_t main_command_;
  uint64_t access_count_;
  network::tcp::ConnectionPtr connection_;
  std::chrono::system_clock::time_point when_;
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_H
