#include "mtl/framework/core/connect_task.h"

namespace mtl {
namespace framework {
namespace core {

ConnectTask::ConnectTask(network::tcp::ClientPtr client, int timeout)
    : Task(0, 0), client_(client), timeout_(timeout) {
}

Task::State ConnectTask::ProcessImpl() {
  if (!client_->IsConnected()) {
    auto now = std::chrono::system_clock::now();
    auto d = now - previous_;
    if ((std::chrono::duration_cast<std::chrono::milliseconds>(d)).count() >= timeout_) {
      client_->Open(client_->server_host(), client_->server_port());
      previous_ = now;
    }
  }
  return Task::kActive;
}
}
}
}
