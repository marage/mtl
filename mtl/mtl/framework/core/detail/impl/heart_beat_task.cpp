#include "mtl/framework/core/heart_beat_task.h"
#include <chrono>
#include "mtl/network/out_request.hpp"
#include "mtl/framework/log/log.h"

namespace mtl {
namespace framework {
namespace core {

HeartBeatTask::HeartBeatTask(network::tcp::ClientPtr client,
                             int timeout)
  : Task(0, 0), client_(client), timeout_(timeout) {
}

Task::State HeartBeatTask::ProcessImpl() {
  auto d = std::chrono::system_clock::now() - client_->LastRequestTime();
  if ((std::chrono::duration_cast<std::chrono::milliseconds>(d)).count() >= timeout_) {
    network::OutRequest oreq(0);
    client_->Send(oreq);
  }
  return Task::kActive;
}
}
}
}
