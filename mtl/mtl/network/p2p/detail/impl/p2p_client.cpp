#include "mtl/network/p2p/client.hpp"
#include <boost/asio/placeholders.hpp>
#include <boost/asio/ip/address.hpp>
#include "mtl/network/in_request.hpp"
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include "mtl/network/p2p/detail/p2p_broadcast_task.hpp"
#include "mtl/utility/utility.hpp"

namespace mtl {
namespace network {

using namespace udp;

namespace p2p {

Client::Client(const DgramPtr& dgram, Role role)
  : dgram_(dgram), timeout_(false), seq_range_({ 0, 0xffffffffffffffff })
  , role_(role), status_(kIdleState), task_group_(0, false, 0) {
}

Client::~Client() {
  Close();
}

bool Client::Open(const UdpEndpoint& endpoint) {
  bool ret = dgram_->open(endpoint);
  if (ret) {
    dgram_->sent_signal.connect(boost::bind(&Client::HandlePacketSent, this, _1, _2, _3));
    dgram_->arrival_signal.connect(boost::bind(&Client::HandlePacketArrival, this, _1, _2, _3));
    dgram_->timeout_signal.connect(boost::bind(&Client::HandlePacketTimeout, this, _1, _2));
    dgram_->group_head_arrival_signal.connect(boost::bind(&Client::HandleGroupHeadArrival,
                                                          this, _1, _2, _3));
    dgram_->tick_signal.connect(boost::bind(&Client::ProcessTasks, this));
    // initial
    timeout_ = false;
    status_ = kIdleState;
    role_ = kClientRole;
  }
  return ret;
}

void Client::Close() {
  ClearTempTasks();
  dgram_->close();
}

UdpEndpoint Client::LocalEndpoint() const {
  return dgram_->LocalEndpoint();
}

bool Client::IsNeighbor(const UdpEndpoint& endpoint) const {
  bool ok = (server_endpoint_ == endpoint);
  if (!ok) {
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ++it) {
      if ((*it).address == endpoint) {
        ok = true;
        break;
      }
    }
  }
  return ok;
}

// protocol: token | role
void Client::Join(const UdpEndpoint& to, const std::string& access_token) {
  if (status_ == kIdleState) {
    // change status
    server_endpoint_ = to;
    access_token_ = access_token;
    status_ = kJoiningState;
    // notify server
    OutRequest oreq(PT_P2P_JOIN, 0, kUdpHeaderLength);
    oreq.writeVersion(P2P_VERSION);
    oreq.writeString(access_token);
    oreq.writeInt8((uint8_t)role_);
    dgram_->SendTo(oreq, to, 10000);
  }
}

// protocol: seq | timeout | data
void Client::Broadcast(OutRequest& oreq, int32_t timeout) {
  if (!IsMember()) {
    return;
  }
  assert(oreq.begin() == P2P_DATA_HEADER_LENGTH);
  // filter sent out packet
  uint64_t seq = GetSeq();
  packet_filter_.Passed(seq);
  // request
  OutRequest d_oreq(oreq.buffer(), oreq.size());
  d_oreq.writeCommand(PT_P2P_BROADCAST_DATA);
  d_oreq.writeSequence(Dgram::NextSequence());
  d_oreq.writeLength(oreq.size());
  d_oreq.writeVersion(P2P_VERSION);
  d_oreq.writeInt64(seq);
  d_oreq.writeInt16((uint16_t)timeout);
  d_oreq.skip(0, kSkipEnd);
  // At first, send data to the server
  if (server_endpoint_.port() > 0)
    dgram_->SendTo(d_oreq, server_endpoint_, timeout);
  // send data to the neighbors
  if (d_oreq.size() > kUdpDataSize && !neighbors_.empty()) {
    UdpEndpoint from;
    mtl::TaskPtr t(new BroadcastNeighborsTask(this, d_oreq, timeout, from));
    temp_tasks_mutex_.lock();
    temp_tasks_.push_back(t);
    temp_tasks_mutex_.unlock();
    // async process tasks
    dgram_->GetIOService().post(boost::bind(&Client::ActivateTempTasks, this));
  }
}

void Client::SendTo(OutRequest& oreq, const UdpEndpoint& to, uint32_t timeout) {
  dgram_->SendTo(oreq, to, timeout);
}

// protocol: token | role
void Client::Leave() {
  ClearTempTasks();
  dgram_->ClearPackets();
  if (IsMember()) {
    OutRequest oreq(PT_P2P_LEAVE, 0, kUdpHeaderLength);
    oreq.writeVersion(P2P_VERSION);
    oreq.writeString(access_token_);
    oreq.writeInt8((uint8_t)role());
    dgram_->SendTo(oreq, server_endpoint_);
    status_ = kIdleState;
  }
}

void Client::HandlePacketArrival(InRequest& ireq, const UdpEndpoint& from, 
                                 int32_t milliseconds) {
  uint32_t type = ireq.readCommand();
  if (type == PT_P2P_BROADCAST_DATA) {
    HandleBroadcastData(ireq, from, milliseconds);
  } else if (type == PT_P2P_DIRECT_DATA) {
    InRequest d_ireq(ireq.buffer(), ireq.size(), P2P_DATA_HEADER_LENGTH);
    packet_arrival_signal(d_ireq, from);
  } else if (type == PT_P2P_ADD_NEIGHBOR) {
    HandleAddNeighbor(ireq, from);
  } else if (type == PT_P2P_REPLACE_NEIGHBOR) {
    HandleReplaceNeighbor(ireq, from);
  } else if (type == PT_P2P_JOIN_ACK) {
    HandleJoinAck(ireq, from);
  }
}

void Client::HandleGroupHeadArrival(InRequest& ireq, const UdpEndpoint& from,
                                    bool* passed) {
  assert(passed);
  if (IsNeighbor(from)) {
    ireq.skip(kHeaderLength, kSkipCurrent);
    uint64_t seq = ireq.readInt64();
    *passed = packet_filter_.Passed(seq);
  } else {
    *passed = false;
  }
}

void Client::HandlePacketSent(const OutRequest& oreq, const UdpEndpoint& to,
                              int32_t milliseconds) {
  if (to == server_endpoint_) {
    timeout_ = false;
  } else {
    // stat cost time
    StatCost(to, oreq.size(), milliseconds);
  }
  ProcessTasks();
}

void Client::HandlePacketTimeout(const OutRequest& oreq, const UdpEndpoint& to) {
  InRequest ireq(oreq.buffer(), oreq.size(), oreq.begin());
  uint32_t type = ireq.readCommand();
  if (type == PT_P2P_JOIN) {
    if (status_ == kJoiningState) {
      status_ = kIdleState;
      join_failed_signal();
    }
  } else if (type == PT_P2P_BROADCAST_DATA) {
    HandleBroadcastDataTimeout(oreq, to);
  }
  ProcessTasks();
}

// protocol: sequence range(min,max) | neighbor count | {neighbor(endpoint, role)}
void Client::HandleJoinAck(InRequest& ireq, const UdpEndpoint& /*from*/) {
  if (status_ != kJoiningState) {
    return;
  }
  ClearTempTasks();
  task_group_.RemoveAll();
  neighbors_.clear();
  seq_range_.min = ireq.readInt64();
  seq_range_.max = ireq.readInt64();
  if (seq_range_.size() > 0) {
    uint16_t count = ireq.readInt16();
    for (uint16_t i = 0; i < count; ++i) {
      uint8_t role = ireq.readInt8();
      char bytes[16] = { 0 };
      ireq.readBinary(bytes, 16);
      uint16_t port = ireq.readInt16();
      neighbors_.push_back({ role, ToEndpoint(bytes, port), 0, 0, 50 });
    }
    status_ = kJoined;
    dgram_->KeepAlive(server_endpoint_, true);
    join_finished_signal();
  } else {
    status_ = kIdleState;
    join_failed_signal();
  }
}

// protocol: role | ip | port
void Client::HandleAddNeighbor(InRequest& ireq, const UdpEndpoint& /*from*/) {
  if (IsMember()) {
    uint8_t count = ireq.readInt8();
    (void)count;
    uint8_t role = ireq.readInt8();
    char bytes[16] = { 0 };
    ireq.readBinary(bytes, 16);
    uint16_t port = ireq.readInt16();
    UdpEndpoint endpoint = ToEndpoint(bytes, port);
    if (!IsNeighbor(endpoint)) {
      neighbors_.push_back({role, endpoint, 0, 0, 50});
    }
  }
}

// protocol: count | {role | ip | port} | count | {role | ip : port}
void Client::HandleReplaceNeighbor(InRequest& ireq, const UdpEndpoint& /*from*/) {
  if (IsMember()) {
    uint8_t role;
    char bytes[16];
    uint16_t port;
    uint8_t count;
    UdpEndpoint address;
    // removed
    count = ireq.readInt8();
    for (uint8_t i = 0; i < count; ++i) {
      ireq.readBinary(bytes, 16);
      port = ireq.readInt16();
      RemoveNeighbor(ToEndpoint(bytes, port));
    }
    // added
    count = ireq.readInt8();
    for (uint8_t i = 0; i < count; ++i) {
      role = ireq.readInt8();
      ireq.readBinary(bytes, 16);
      port = ireq.readInt16();
      address = ToEndpoint(bytes, port);
      if (IsNeighbor(address)) {
        neighbors_.push_back({role, address, 0, 0, 50});
      }
    }
  }
}

// protocol: seq | timeout | data
void Client::HandleBroadcastData(InRequest& ireq, const UdpEndpoint& from,
                                 int32_t milliseconds) {
  if (!IsMember() || !IsNeighbor(from)) {
    return;
  }

  uint16_t size = ireq.size();

  // check whether the packet is duplicate
  uint64_t seq = ireq.readInt64();
  if (size <= udp::kUdpDataSize && !packet_filter_.Passed(seq)) {
    return;
  }

  // stat big packet cost time
  StatCost(from, size, milliseconds);

  // notify app
  InRequest d_ireq(ireq.buffer(), ireq.size(), (ireq.begin()+P2P_DATA_HEADER_LENGTH));
  packet_arrival_signal(d_ireq, from);

  // forward data to the neighbors
  if (size > kUdpDataSize && false && !neighbors_.empty() && (server_endpoint_ != from)) {
    uint16_t timeout = ireq.readInt16();
    if (timeout > milliseconds) {
      uint16_t left = timeout - (uint16_t)milliseconds;
      if (left > (milliseconds/3)) {
        OutRequest oreq(ireq.buffer(), ireq.size(), ireq.begin());
        oreq.skip(sizeof(uint64_t), kSkipCurrent);
        oreq.writeInt16(left);
        oreq.skip(0, kSkipEnd);
        mtl::TaskPtr t(new BroadcastNeighborsTask(this, oreq, left, from));
        AppendTask(t);
      }
    }
  }
}

// protocol: token | role | ip | port
void Client::HandleBroadcastDataTimeout(const OutRequest& oreq, const UdpEndpoint& to) {
  if (!IsMember())
    return;
  // if failed to send to server, notify leave
  if (to == server_endpoint_) {
    if (timeout_) {
      if (timeout_end_time_ <= SystemClock::now()) {
        status_ = kIdleState;
        leave_signal();
      }
    } else {
      timeout_ = true;
      timeout_end_time_ = SystemClock::now() + std::chrono::seconds(10);
    }
    return;
  }
  // report
  auto it = std::find_if(neighbors_.begin(), neighbors_.end(), [&to](const Neighbor& n) {
    return (n.address == to);
  });
  if (it != neighbors_.end()) {
    ++(*it).failure_times;
    if ((*it).failureRate() > 0.5) {
      dgram_->ClearPackets(to);
      neighbors_.erase(it);
      // notify server
      if (server_endpoint_.port() > 0) {
        OutRequest oreq(PT_P2P_SEND_FAILED, 0, kUdpHeaderLength);
        oreq.writeVersion(P2P_VERSION);
        oreq.writeString(access_token_);
        oreq.writeInt8((uint8_t) role_);
        char bytes[16];
        oreq.writeBinary(ToBytes(to, bytes), 16);
        oreq.writeInt16(to.port());
        dgram_->SendTo(oreq, server_endpoint_);
      }
    }
  }
  // notify
  packet_timeout_signal(oreq, to);
}

void Client::RemoveNeighbor(const UdpEndpoint& address) {
  auto it = std::find_if(neighbors_.begin(), neighbors_.end(), [&address](const Neighbor& n) {
    return (n.address == address);
  });
  if (it != neighbors_.end()) {
    dgram_->ClearPackets(address);
    neighbors_.erase(it);
  }
}

void Client::ActivateTempTasks() {
  temp_tasks_mutex_.lock();
  for (auto it = temp_tasks_.begin(), end = temp_tasks_.end();
       it != end; ++it) {
    task_group_.Append(*it);
  }
  temp_tasks_.clear();
  temp_tasks_mutex_.unlock();
  ProcessTasks();
}

void Client::ProcessTasks() {
  if (task_group_.IsInactive() || task_group_.IsCompleted()) {
    task_group_.Activate();
  }
  task_group_.Process();
}

void Client::ClearTempTasks() {
  temp_tasks_mutex_.lock();
  temp_tasks_.clear();
  temp_tasks_mutex_.unlock();
}

void Client::AppendTask(const TaskPtr& t) {
  const int MAX_TASK_COUNT = 25;
  task_group_.Append(t);
  if (task_group_.count() >= MAX_TASK_COUNT) {
    task_group_.RemoveFirst();
  }
  ProcessTasks();
}

void Client::StatCost(const UdpEndpoint& from, uint16_t size,
                      int32_t milliseconds) {
  auto it = std::find_if(neighbors_.begin(), neighbors_.end(), [&from](const Neighbor& n) {
    return (n.address == from);
  });
  if (it != neighbors_.end()) {
    if (size >= 1024) {
      (*it).cost_per_k = ((*it).cost_per_k +
                          int32_t((double)milliseconds/((double)size/1024.0))) / 2;
    } else {
      (*it).cost_per_k = ((*it).cost_per_k + milliseconds) / 2;
    }
  }
}

uint64_t Client::GetSeq() const {
  static uint64_t seq = seq_range_.min;
  seq++;
  if ((seq % seq_range_.max) == 0) {
    seq = seq_range_.min;
  }
  return seq;
}

}
}
}
