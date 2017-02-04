#include "mtl/network/p2p/detail/p2p_broadcast_task.hpp"
#include <algorithm>
#include "mtl/network/p2p/server.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/udp/dgram.hpp"

namespace mtl {
namespace network {
namespace p2p {

// Broadcast Neighbors
BroadcastNeighborsTask::BroadcastNeighborsTask(Client* c, const OutRequest& oreq, int timeout,
                                               const boost::asio::ip::udp::endpoint& from)
    : Task(0, timeout), client_(c), request_(oreq), from_(from), next_pos_(0)
{
}

Task::Status BroadcastNeighborsTask::processImpl()
{
    using namespace boost::posix_time;
    int size = static_cast<int>(client_->neighbors_.size());
    if (next_pos_ < size) {
        ptime now = boost::posix_time::microsec_clock::local_time();
        ptime::time_duration_type duration = deadTime() - now;
        int64_t timeout = duration.total_milliseconds();
        if (timeout > 0) {
            uint16_t request_size = request_.size();
            auto it = client_->neighbors_.begin();
            std::advance(it, next_pos_);
            int count = 0;
            while (next_pos_ < size && count <= P2P_NEIGHBOR_COUNT && !client_->dgram_->isBusy()) {
                if (from_.port() == 0 || (*it).address != from_) {
                    if (timeout >= (6*(*it).cost(request_size)/5)) {
                        client_->dgram_->sendTo(request_, (*it).address, (uint32_t)timeout);
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
        setStatus(Task::COMPLETED);
    }
    return status();
}

BroadcastClientsTask::BroadcastClientsTask(Server* s, const OutRequest& oreq, int timeout,
                                           const boost::asio::ip::udp::endpoint& from)
    : Task(0, timeout), server_(s), request_(oreq), from_(from), next_pos_(0)
{
}

void BroadcastClientsTask::activateImpl()
{
    server_->bfsMembers(from_, targets_);
    if (targets_.size() <= 1) {
        setStatus(Task::COMPLETED);
    }
}

Task::Status BroadcastClientsTask::processImpl()
{
    const int CLIENT_COUNT_PER_LOOP = 5;
    int size = static_cast<int>(targets_.size()) - 1;
    if (next_pos_ < size) {
        auto d = deadTime() - boost::posix_time::microsec_clock::local_time();
        int64_t timeout = d.total_milliseconds();
        if (timeout > 0) {
            auto it = targets_.begin();
            std::advance(it, next_pos_);
            int count = 0;
            while (next_pos_ < size && count <= CLIENT_COUNT_PER_LOOP && !server_->dgram_->isBusy()) {
                server_->dgram_->sendTo(request_, (*it), (uint32_t)timeout);
                ++next_pos_;
                ++it;
                ++count;
            }
        } else {
            next_pos_ = size;
        }
    }
    if (next_pos_ >= size) {
        setStatus(Task::COMPLETED);
    }
    return status();
}

} // p2p
} // network
} // mtl
