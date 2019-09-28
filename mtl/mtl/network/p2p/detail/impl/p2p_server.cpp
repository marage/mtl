#include "mtl/network/p2p/server.hpp"
#include <algorithm>
#include <thread>
#include <boost/thread/thread.hpp>
#include <boost/asio/placeholders.hpp>
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include "mtl/network/p2p/detail/p2p_broadcast_task.hpp"
#include <iostream>

namespace mtl {
namespace network {

using namespace udp;

namespace p2p {

const int MAX_SEQUENCE_COUNT = 10000;
const int MAX_CLIENT_COUNT = 10000;
const int MAX_EDGE_WEIGHT = 10000;

Server::Server(const udp::DgramPtr& dgram, const std::string& token)
  : Client(dgram), token_(token), check_alive_timer_(dgram->GetIOService()) {
  seq_range_.min = 1;
  seq_range_.max = MAX_SEQUENCE_COUNT;
  members_seq_range_.min = MAX_SEQUENCE_COUNT;
  members_seq_range_.max = 0xffffffffffffffff;
  next_member_seq_ = MAX_SEQUENCE_COUNT;
}

bool Server::Open(const UdpEndpoint& endpoint) {
  bool ret = Client::Open(endpoint);
  if (ret) {
    // disconnect signals
    dgram_->keep_alive_signal.disconnect_all_slots();
    dgram_->arrival_signal.disconnect_all_slots();
    dgram_->group_head_arrival_signal.disconnect_all_slots();
    // connect signals
    dgram_->keep_alive_signal.connect(boost::bind(&Server::HandleAlive, this, _1));
    dgram_->arrival_signal.connect(boost::bind(&Server::HandlePacketArrival, this, _1, _2, _3));
    dgram_->group_head_arrival_signal.connect(boost::bind(&Server::HandleGroupHeadArrival,
                                                          this, _1, _2, _3));

    check_alive_timer_.expires_from_now(boost::posix_time::seconds(5));
    check_alive_timer_.async_wait(boost::bind(&Server::HandleCheckAliveTimeout,
                                              boost::static_pointer_cast<Server>(shared_from_this()),
                                              boost::asio::placeholders::error));
    // activate task group
    task_group_.Activate();
  }
  return ret;
}

void Server::Close() {
  dgram_->keep_alive_signal.disconnect_all_slots();
  dgram_->arrival_signal.disconnect_all_slots();
  dgram_->group_head_arrival_signal.disconnect_all_slots();
  Client::Close();
}

void Server::Broadcast(OutRequest& oreq, int32_t timeout) {
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
  if (server_endpoint_.port() > 0) {
    dgram_->SendTo(d_oreq, server_endpoint_, timeout);
  }
  // send data to the neighbors
  if (!neighbors_.empty()) {
    UdpEndpoint from;
    mtl::TaskPtr t(new BroadcastNeighborsTask(this, d_oreq, timeout, from));
    temp_tasks_mutex_.lock();
    temp_tasks_.push_back(t);
    temp_tasks_mutex_.unlock();
  }
  if (!graph_.vertices().empty()) {
    UdpEndpoint from;
    mtl::TaskPtr t(new BroadcastClientsTask(this, d_oreq, timeout, from));
    temp_tasks_mutex_.lock();
    temp_tasks_.push_back(t);
    temp_tasks_mutex_.unlock();
  }
  // async process tasks
  dgram_->GetIOService().post(boost::bind(&Server::ActivateTempTasks, this));
}

void Server::HandlePacketArrival(InRequest& ireq, const UdpEndpoint& from, 
                                 int32_t milliseconds) {
  int s = ireq.size();
  std::cout << "cost:" << milliseconds << "\t" << s << std::endl;
  uint32_t type = ireq.readCommand();
  if (type == PT_P2P_BROADCAST_DATA) {
    HandleBroadcastData(ireq, from, milliseconds);
  } else if (type == PT_P2P_JOIN) {
    HandleJoinRequest(ireq, from);
  } else if (type == PT_P2P_LEAVE) {
    HandleLeaveRequest(ireq, from);
  } else if (type == PT_P2P_SEND_FAILED) {
    HandleFailedReportRequest(ireq, from);
  } else {
    Client::HandlePacketArrival(ireq, from, milliseconds);
    if (type == PT_P2P_JOIN_ACK) {
      uint64_t max_seq = seq_range_.min + MAX_CLIENT_COUNT;
      if (max_seq < seq_range_.max) {
        members_seq_range_.min = max_seq;
        members_seq_range_.max = seq_range_.max;
        seq_range_.max = max_seq;
        next_member_seq_ = members_seq_range_.min;
      }
    }
  }
}

void Server::HandleGroupHeadArrival(InRequest& ireq, const UdpEndpoint& from,
                                    bool* passed) {
  assert(passed);
  if (IsNeighbor(from) || IsClient(from)) {
    ireq.skip(kHeaderLength, kSkipCurrent);
    uint64_t seq = ireq.readInt64();
    *passed = packet_filter_.Passed(seq);
  } else {
    *passed = false;
  }
}

// protocol: token | role
void Server::HandleJoinRequest(InRequest& ireq, const UdpEndpoint& from) {
  // check token
  std::string token = ireq.readString(64);
  if (token != token_)
    return;

  // check exists
  const std::string& from_name = udp::Dgram::ToString(from);
  if (graph_.ContainsVertex(from_name)) {
    OutRequest oreq(PT_P2P_JOIN_ACK, 0, kUdpHeaderLength);
    oreq.writeInt64(0);
    oreq.writeInt64(0);
    dgram_->SendTo(oreq, from);
    return;
  }

  SequenceRange seq_range = { 0, 0 };
  uint8_t role = ireq.readInt8();
  if (role == kClientRole) {
    seq_range = GetUsableSequenceRange(MAX_SEQUENCE_COUNT);
  } else {
    seq_range = GetUsableSequenceRange(MAX_CLIENT_COUNT * MAX_SEQUENCE_COUNT);
  }
  const GraphVertex* from_vertex = nullptr;
  std::list<NearVertex> members;
  if (seq_range.size() > 0) {
    // add new member
    from_vertex = graph_.AddVertex(from_name, role, from, seq_range);
    if (!from_vertex) {
      // failed
      OutRequest oreq(PT_P2P_JOIN_ACK, 0, kUdpHeaderLength);
      oreq.writeInt64(0);
      oreq.writeInt64(0);
      dgram_->SendTo(oreq, from);
      return;
    }
    // near members
    members = NearMembers(from_name, P2P_NEIGHBOR_COUNT);
  }
  // join ack
  {
    OutRequest oreq(PT_P2P_JOIN_ACK, 0, kUdpHeaderLength);
    oreq.writeInt64(seq_range.min);
    oreq.writeInt64(seq_range.max);
    if (seq_range.size() > 0) {
      // response
      oreq.writeInt16((uint16_t)members.size());
      char bytes[16];
      for (auto it = members.begin(), end = members.end(); it != end; ++it) {
        oreq.writeInt8((*it).vertex->role());
        const auto& address = (*it).vertex->address();
        oreq.writeBinary(ToBytes(address, bytes), 16);
        oreq.writeInt16(address.port());
        // add an edge into the graph
        graph_.AddEdge(from_vertex, (*it).vertex, (*it).weight);
      }
    }
    dgram_->SendTo(oreq, from);
  }
  // notify neighbors
  if (!members.empty()) {
    OutRequest oreq(PT_P2P_ADD_NEIGHBOR, 0, kUdpHeaderLength);
    oreq.writeInt8(1);
    oreq.writeInt8(role);
    char bytes[16];
    oreq.writeBinary(ToBytes(from, bytes), 16);
    oreq.writeInt16(from.port());
    for (auto it = members.begin(), end = members.end(); it != end; ++it) {
      dgram_->SendTo(oreq, (*it).vertex->address());
    }
  }
}

// protocol: token
void Server::HandleLeaveRequest(InRequest& ireq, const UdpEndpoint& from) {
  // check token
  std::string token = ireq.readString(64);
  if (token == token_) {
    RemoveMember(udp::Dgram::ToString(from));
  }
}

// protocol: token | role | ip | port
void Server::HandleFailedReportRequest(InRequest& ireq, const UdpEndpoint& from) {
  // check token
  std::string token = ireq.readString(64);
  if (token != token_)
    return;
  uint8_t role = ireq.readInt8();
  ((void)role);
  char bytes[16] = { 0 };
  ireq.readBinary(bytes, 16);
  uint16_t port = ireq.readInt16();
  UdpEndpoint to = ToEndpoint(bytes, port);
  const std::string& to_name = udp::Dgram::ToString(to);
  const std::string& from_name = udp::Dgram::ToString(from);
  // change edge weight
  if (const GraphVertex* u = graph_.Vertex(from_name)) {
    if (const GraphVertex* v = graph_.Vertex(to_name)) {
      if (GraphEdge* e = graph_.Edge(u, v)) {
        e->set_weight(MAX_EDGE_WEIGHT);
      }
    }
    // other replaces neighbor
    ReplaceNeighbor(u, from);
  }
  // new member
  std::list<NearVertex> members = NearMembers(from_name, 1);
  if (!members.empty()) {
    const NearVertex& nv = members.front();
    const auto& address = nv.vertex->address();
    OutRequest oreq(PT_P2P_ADD_NEIGHBOR, 0, kUdpHeaderLength);
    oreq.writeInt8(1);
    oreq.writeInt8(nv.vertex->role());
    oreq.writeBinary(ToBytes(address, bytes), 16);
    oreq.writeInt16(address.port());
    dgram_->SendTo(oreq, from);
  }
}

void Server::HandleCheckAliveTimeout(const boost::system::error_code& e) {
  if (!e) {
    std::list<const GraphVertex*> removed;
    for (auto it = graph_.vertices().begin(), end = graph_.vertices().end();
         it != end; ++it) {
      if (it->second->IncAliveTimeoutTimes() > 3) {
        removed.push_back(it->second);
      }
    }
    for (auto it = removed.begin(), end = removed.end(); it != end; ++it) {
      RemoveMember(*it);
    }
  }
  check_alive_timer_.expires_from_now(boost::posix_time::seconds(5));
  check_alive_timer_.async_wait(boost::bind(&Server::HandleCheckAliveTimeout,
                                            boost::static_pointer_cast<Server>(shared_from_this()),
                                            boost::asio::placeholders::error));
}

void Server::HandleAlive(const UdpEndpoint& from) {
  if (GraphVertex* v = graph_.Vertex(udp::Dgram::ToString(from))) {
    v->set_alive_timeout_times(0);
  }
}

void Server::HandleBroadcastData(InRequest& ireq, const UdpEndpoint& from,
                                 int32_t milliseconds) {
  if (!IsClient(from) && !IsNeighbor(from)) {
    return;
  }

  uint16_t size = ireq.size();

  // check whether the packet is duplicate
  uint64_t seq = ireq.readInt64();
  if (size <= kUdpDataSize && !packet_filter_.Passed(seq))
    return;

  // stat big packet cost time
  StatCost(from, size, milliseconds);

  // notify app
  InRequest d_ireq(ireq.buffer(), ireq.size(), (ireq.begin()+P2P_DATA_HEADER_LENGTH));
  packet_arrival_signal(d_ireq, from);

  // forward data to the neighbors
  uint16_t timeout = ireq.readInt16();
  if (size > kUdpDataSize && !neighbors_.empty() && (server_endpoint_ != from)) {
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

  // forward data to clients
  if (timeout > milliseconds && graph_.vertices().size() > 1) {
    uint16_t left = timeout - (uint16_t)milliseconds;
    OutRequest oreq(ireq.buffer(), ireq.size(), ireq.begin());
    oreq.skip(0, kSkipEnd);
    mtl::TaskPtr t(new BroadcastClientsTask(this, oreq, left, from));
    AppendTask(t);
  }
}

SequenceRange Server::GetUsableSequenceRange(uint64_t size) {
  SequenceRange r = { 0, 0 };
  for (std::list<SequenceRange>::iterator it = usable_sequences_.begin();
       it != usable_sequences_.end(); ++it) {
    if ((*it).size() == size) {
      r = *it;
      usable_sequences_.erase(it);
      return r;
    }
  }
  if (next_member_seq_ + size < members_seq_range_.max) {
    r.min = next_member_seq_;
    r.max = next_member_seq_ + size;
    next_member_seq_ = r.max;
  }
  return r;
}

int evaluateScale(const UdpEndpoint& a,
                  const UdpEndpoint& b) {
  int scale = 0;
  if (a.address().is_v4() && b.address().is_v4()) {
    auto aa = a.address().to_v4().to_bytes();
    auto bb = b.address().to_v4().to_bytes();
    if (aa[0] == bb[0]) {
      scale = 2;
      if (aa[1] == bb[1]) {
        scale *= 2;
        if (aa[2] == bb[2]) {
          scale *= 2;
          if (aa[3] == bb[3]) {
            scale *= 2;
          }
        }
      }
    }
  }
  return scale;
}

std::list<Server::NearVertex> Server::NearMembers(const std::string& center,
                                                  uint32_t max_count) {
  std::list<NearVertex> scale8;
  std::map<std::string, uint32_t>::const_iterator center_it = graph_.vertex_names().find(center);
  std::map<std::string, uint32_t>::const_iterator end = graph_.vertex_names().end();
  if (center_it == end)
    return scale8;

  std::map<std::string, uint32_t>::const_iterator begin = graph_.vertex_names().begin();
  GraphVertex* center_v = graph_.Vertex(center_it->second);

  std::list<NearVertex> scale2;
  std::list<NearVertex> scale4;
  const auto& center_address = center_v->address();
  uint8_t center_role = center_v->role();
  int count = 0;
  std::map<std::string, uint32_t>::const_iterator it = center_it;
  for (++it; (it != end && ++count < 20); ++it) {
    GraphVertex* u = graph_.Vertex(it->first);
    if (u->role() != center_role)
      continue;
    if (graph_.Edge(u, center_v))
      continue;
    if (IsValidNeighbor(u)) {
      int scale = evaluateScale(center_address, u->address());
      if (scale == 16) {
        scale8.push_back(NearVertex{u, 1});
        if (scale8.size() >= max_count)
          break;
      } else if (scale == 8) {
        scale4.push_back(NearVertex{u, 2});
      } else if (scale == 4) {
        scale2.push_back(NearVertex{u, 4});
      }
    }
  }
  it = begin;
  int prev_count = std::distance(it, center_it);
  if (prev_count >= 1) {
    if (prev_count > 20)
      prev_count = 20;
    count = 0;
    it = center_it;
    for (--it; ++count <= prev_count; --it) {
      GraphVertex* u = graph_.Vertex(it->first);
      if (u->role() != center_role)
        continue;
      if (graph_.Edge(u, center_v))
        continue;
      if (IsValidNeighbor(u)) {
        int scale = evaluateScale(center_address, u->address());
        if (scale == 16) {
          scale8.push_back(NearVertex{u, 1});
          if (scale8.size() >= max_count)
            break;
        } else if (scale == 8) {
          scale4.push_back(NearVertex{u, 2});
        } else if (scale == 4) {
          scale2.push_back(NearVertex{u, 4});
        }
      }
    }
  }

  if (scale8.size() >= max_count)
    return scale8;

  int n = max_count - static_cast<int>(scale8.size());
  for (auto it = scale4.begin(), end = scale4.end(); n > 0 && it != end; ++it, --n) {
    scale8.push_back(*it);
  }

  for (auto it = scale2.begin(), end = scale2.end(); n > 0 && it != end; ++it, --n) {
    scale8.push_back(*it);
  }

  return scale8;
}

void Server::RemoveMember(const std::string& name) {
  if (const GraphVertex* v = graph_.Vertex(name)) {
    RemoveMember(v);
  }
}

void Server::RemoveMember(const GraphVertex* v) {
  assert(v);
  // recycle sequence
  usable_sequences_.push_back(v->seq_range());
  packet_filter_.Remove(v->seq_range().min, v->seq_range().max);

  // others replace neighbor
  const UdpEndpoint& ep = v->address();
  const auto& edges = graph_.relationships().Edges(v);
  for (auto it = edges.begin(), end = edges.end(); it != end; ++it) {
    GraphVertex* t = graph_.Vertex((*it)->target());
    GraphVertex* s = graph_.Vertex((*it)->source());
    ReplaceNeighbor((t != v ? t : s), ep);
  }

  // remove the vertex from the graph
  graph_.RemoveVertex(udp::Dgram::ToString(v->address()));
}

void Server::ReplaceNeighbor(const GraphVertex* v, const UdpEndpoint& neighbor) {
  assert(v);
  char bytes[16];
  OutRequest oreq(PT_P2P_REPLACE_NEIGHBOR, 0, kUdpHeaderLength);
  oreq.writeInt8(1);
  oreq.writeBinary(ToBytes(neighbor, bytes), 16);
  oreq.writeInt16(neighbor.port());
  std::list<NearVertex> members = NearMembers(udp::Dgram::ToString(v->address()), 1);
  if (!members.empty()) {
    const NearVertex& nv = members.front();
    const auto& address = nv.vertex->address();
    oreq.writeInt8(1);
    oreq.writeInt8(nv.vertex->role());
    oreq.writeBinary(ToBytes(address, bytes), 16);
    oreq.writeInt16(address.port());
  }
  dgram_->SendTo(oreq, v->address());
}

bool Server::IsValidNeighbor(const GraphVertex* v) {
  assert(v);
  int count = 0;
  const auto& edges = graph_.relationships().Edges(v);
  for (auto it = edges.begin(), end = edges.end(); it != end; ++it) {
    if ((*it)->weight() < MAX_EDGE_WEIGHT) {
      if (++count >= P2P_NEIGHBOR_COUNT) {
        return false;
      }
    }
  }
  return true;
}

void Server::BfsMembers(const UdpEndpoint& start,
                        std::list<UdpEndpoint>& targets) {
  GraphVertex* v = 0;
  if (start.port() > 0) {
    v = graph_.Vertex(udp::Dgram::ToString(start));
  } else {
    if (!graph_.vertices().empty()) {
      v = graph_.vertices().begin()->second;
    }
  }
  if (v) {
    graph_.BreadthFirstSearch(v, targets);
  }
}
}
}
}
