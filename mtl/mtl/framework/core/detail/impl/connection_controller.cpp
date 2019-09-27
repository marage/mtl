#include "mtl/framework/core/connection_controller.h"
#include "mtl/framework/log/log.h"
#include <time.h>

namespace mtl {
namespace framework {
namespace core {

ConnectionController::ConnectionController(uint32_t id, network::tcp::connection_ptr c)
    : LogicUnit<network::tcp::connection_ptr>(id, c, PROCESS_ONCE)
    , main_command_(0), access_count_(0), connection_(c)
{
    setActive(true);
    when_ = std::chrono::system_clock::now();
    c->packet_arrival_signal.connect(
                boost::bind(&ConnectionController::handleArrival, this, _1));
    c->close_signal.connect(
                boost::bind(&ConnectionController::handle_close, this, _1));
}

void ConnectionController::handleArrival(InRequest& ireq)
{
    uint32_t cmd = ireq.readCommand();
    if (cmd) {
        pushRequest(ireq);
    }
    when_ = std::chrono::system_clock::now();
}

void ConnectionController::handle_close(const boost::system::error_code& ec)
{
    setActive(false);
    when_ = std::chrono::system_clock::now();
    // log
    BOOST_LOG_FUNCTION()
    auto lg = log::global_logger::get();
    BOOST_LOG_SEV(lg, log::SL_INFO) << ec.message();
}
}
}
}
