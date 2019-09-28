#include "mtl/network/p2p/detail/p2p_broadcast_task.hpp"
#include <algorithm>
#include "mtl/network/p2p/server.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "mtl/utility/utility.hpp"

namespace mtl {
namespace network {
namespace p2p {

// Broadcast Neighbors
BroadcastNeighborsTask::BroadcastNeighborsTask(
    Client* c, const OutRequest& oreq, int timeout,
    const UdpEndpoint& from)
  : Task(0, timeout), client_(c), request_(oreq), from_(from)
  , next_pos_(0) {
}

Task::State BroadcastNeighborsTask::ProcessImpl() {
  int size = static_cast<int>(client_->neighbors_.size());
  if (next_pos_ < size) {
    auto now = std::chrono::system_clock::now();
    auto d = dead_time() - now;
    int64_t timeout = std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
    if (timeout > 0) {
      uint16_t request_size = request_.size();
      auto it = client_->neighbors_.begin();
      std::advance(it, next_pos_);
      int count = 0;
      while (next_pos_ < size && count <= P2P_NEIGHBOR_COUNT && !client_->dgram_->IsBusy()) {
        if (from_.port() == 0 || (*it).address != from_) {
          if (timeout >= (6*(*it).cost(request_size)/5)) {
            client_->dgram_->SendTo(request_, (*it).address, static_cast<uint32_t>(timeout));
            ++(*it).sent_count;
            ++count;
          }
        }
        ++next_pos_;
        ++it;
      }
    } else {
      next_pos_ = size;
    }
  }
  if (next_pos_ >= size) {
    set_state(Task::kCompleted);
  }
  return state();
}

BroadcastClientsTask::BroadcastClientsTask(Server* s, const OutRequest& oreq, int timeout,
                                           const UdpEndpoint& from)
  : Task(0, timeout), server_(s), request_(oreq), from_(from), next_pos_(0) {
}

void BroadcastClientsTask::ActivateImpl() {
  server_->BfsMembers(from_, targets_);
  if (targets_.size() <= 1) {
    set_state(Task::kCompleted);
  }
}

Task::State BroadcastClientsTask::ProcessImpl() {
  const int CLIENT_COUNT_PER_LOOP = 5;
  int size = static_cast<int>(targets_.size()) - 1;
  if (next_pos_ < size) {
    auto d = dead_time() - std::chrono::system_clock::now();
    int64_t timeout = std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
    if (timeout > 0) {
      auto it = targets_.begin();
      std::advance(it, next_pos_);
      int count = 0;
      while (next_pos_ < size && count <= CLIENT_COUNT_PER_LOOP && !server_->dgram_->IsBusy()) {
        server_->dgram_->SendTo(request_, (*it), static_cast<uint32_t>(timeout));
        ++next_pos_;
        ++it;
        ++count;
      }
    } else {
      next_pos_ = size;
    }
  }
  if (next_pos_ >= size) {
    set_state(Task::kCompleted);
  }
  return state();
}

} // p2p
} // network
} // mtl
