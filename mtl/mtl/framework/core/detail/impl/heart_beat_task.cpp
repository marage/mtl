#include "mtl/framework/core/heart_beat_task.h"
#include <chrono>
#include "mtl/network/out_request.hpp"
#include "mtl/framework/log/log.h"

namespace mtl {
namespace framework {
namespace core {

HeartBeatTask::HeartBeatTask(network::tcp::ClientPtr client,
                             int timeout)
    : Task(0, 0), client_(client), timeout_(timeout)
{
}

Task::State HeartBeatTask::processImpl() {
    auto d = std::chrono::system_clock::now() - client_->lastRequestTime();
    if ((std::chrono::duration_cast<std::chrono::milliseconds>(d)).count() >= timeout_) {
        network::OutRequest oreq(0);
        client_->send(oreq);
    }
    return Task::kActive;
}
} // namespace core
} // namespace framework
} // namespace mtl
