#ifndef MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_H
#define MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_H
#include <chrono>
#include "mtl/network/tcp/connection.hpp"
#include "logic_unit.h"

namespace mtl {
namespace framework {
namespace core {

class MTL_EXPORT ConnectionController : public LogicUnit<network::tcp::connection_ptr>
{
public:
    ConnectionController(uint32_t id, network::tcp::connection_ptr c);

    uint16_t mainCommand() const { return main_command_; }
    uint64_t accessCount() const { return access_count_; }
    void setAccessCount(uint64_t count)
    {
        access_count_ = count;
    }
    bool isBusy() const { return connection_->isBusy(); }
    const std::chrono::system_clock::time_point& when() const { return when_; }
    TCPEndpoint remoteEndpoint() const
    {
        return connection_->remoteEndpoint();
    }
    void send(OutRequest& oreq) { connection_->send(oreq); }
    void close() { connection_->close(); }

protected:
    void setMainCommand(uint16_t cmd) { main_command_ = cmd; }

private:
    void handleArrival(InRequest& ireq);
    void handle_close(const boost::system::error_code& ec);

    uint16_t main_command_;
    uint64_t access_count_;
    network::tcp::connection_ptr connection_;
    std::chrono::system_clock::time_point when_;
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_H
