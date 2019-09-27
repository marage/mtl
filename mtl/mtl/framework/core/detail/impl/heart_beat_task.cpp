#include "mtl/framework/core/heart_beat_task.h"
#include <chrono>
#include "mtl/network/out_request.hpp"
#include "mtl/framework/log/log.h"

namespace mtl {
namespace framework {
namespace core {

HeartBeatTask::HeartBeatTask(network::tcp::client_ptr client,
                             int timeout)
    : Task(0, 0), client_(client), timeout_(timeout)
{
}

Task::Status HeartBeatTask::processImpl()
{
    typedef std::chrono::duration<int64_t, std::milli> milli_type;
    auto d = std::chrono::system_clock::now() - client_->lastRequestTime();
    if ((std::chrono::duration_cast<milli_type>(d)).count() >= timeout_) {
        network::OutRequest oreq(0);
        client_->send(oreq);
    }
    return Task::ACTIVE;
}
}
}
}
