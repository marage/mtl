#include "mtl/framework/core/connection_controller.h"
#include "mtl/framework/log/log.h"
#include <time.h>

namespace mtl {
namespace framework {
namespace core {

ConnectionController::ConnectionController(uint32_t id,
                                           network::tcp::ConnectionPtr c)
  : LogicUnit<network::tcp::ConnectionPtr>(id, c, kOnce)
  , main_command_(0), access_count_(0), connection_(c) {
  set_active(true);
  when_ = std::chrono::system_clock::now();
  c->packet_arrival_signal.connect(
        boost::bind(&ConnectionController::HandleArrival, this, _1));
  c->close_signal.connect(
        boost::bind(&ConnectionController::HandleClose, this, _1));
}

void ConnectionController::HandleArrival(InRequest& ireq) {
  uint32_t cmd = ireq.ReadCommand();
  if (cmd) {
    PushRequest(ireq);
  }
  when_ = std::chrono::system_clock::now();
}

void ConnectionController::HandleClose(const boost::system::error_code& ec) {
  set_active(false);
  when_ = std::chrono::system_clock::now();
  // log
  BOOST_LOG_FUNCTION()
  auto lg = log::global_logger::get();
  BOOST_LOG_SEV(lg, log::kInfo) << ec.message();
}
}
}
}
