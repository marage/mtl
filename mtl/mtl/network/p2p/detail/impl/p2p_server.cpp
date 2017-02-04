#include "mtl/network/p2p/server.hpp"
#include <algorithm>
#include <thread>
#include <boost/thread/thread.hpp>
#include <boost/asio/placeholders.hpp>
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include "mtl/network/p2p/detail/p2p_broadcast_task.hpp"

namespace mtl {
namespace network {

using namespace udp;

namespace p2p {

const int MAX_SEQUENCE_COUNT = 10000;
const int MAX_CLIENT_COUNT = 10000;
const int MAX_EDGE_WEIGHT = 10000;

Server::Server(const udp::dgram_ptr& dgram, const std::string& token)
    : Client(dgram), token_(token), check_alive_timer_(dgram->getIOService())
{
    seq_range_.min = 1;
    seq_range_.max = MAX_SEQUENCE_COUNT;
    members_seq_range_.min = MAX_SEQUENCE_COUNT;
    members_seq_range_.max = 0xffffffffffffffff;
    next_member_seq_ = MAX_SEQUENCE_COUNT;
}

bool Server::open(const boost::asio::ip::udp::endpoint& endpoint, uint32_t frequency)
{
    bool ret = Client::open(endpoint, frequency);
    if (ret) {
        // disconnect signals
        dgram_->keep_alive_signal.disconnect_all_slots();
        dgram_->arrival_signal.disconnect_all_slots();
        dgram_->group_head_arrival_signal.disconnect_all_slots();
        // connect signals
        dgram_->keep_alive_signal.connect(boost::bind(&Server::handleAlive, this, _1));
        dgram_->arrival_signal.connect(boost::bind(&Server::handlePacketArrival, this, _1, _2, _3));
        dgram_->group_head_arrival_signal.connect(boost::bind(&Server::handleGroupHeadArrival,
                                                              this, _1, _2, _3));

        check_alive_timer_.expires_from_now(boost::posix_time::seconds(5));
        check_alive_timer_.async_wait(boost::bind(&Server::handleCheckAliveTimeout,
                                                  boost::static_pointer_cast<Server>(shared_from_this()),
                                                  boost::asio::placeholders::error));
        // activate task group
        task_group_.activate();
    }
    return ret;
}

void Server::close()
{
    dgram_->keep_alive_signal.disconnect_all_slots();
    dgram_->arrival_signal.disconnect_all_slots();
    dgram_->group_head_arrival_signal.disconnect_all_slots();
    Client::close();
}

void Server::broadcast(OutRequest& oreq, int32_t timeout)
{
    assert(oreq.begin() == P2P_DATA_HEADER_LENGTH);
    // filter sent out packet
    uint64_t seq = getSeq();
    packet_filter_.passed(seq);
    // request
    OutRequest d_oreq(oreq.buffer(), oreq.size());
    d_oreq.writeCommand(PT_P2P_BROADCAST_DATA);
    d_oreq.writeSequence(Dgram::nextSequence());
    d_oreq.writeLength(oreq.size());
    d_oreq.writeVersion(P2P_VERSION);
    d_oreq.writeInt64(seq);
    d_oreq.writeInt16((uint16_t)timeout);
    d_oreq.skip(0, SKIP_END);
    // At first, send data to the server
    if (server_endpoint_.port() > 0)
        dgram_->sendTo(d_oreq, server_endpoint_, timeout);
    // send data to the neighbors
    if (!neighbors_.empty()) {
        boost::asio::ip::udp::endpoint from;
        mtl::task_ptr t(new BroadcastNeighborsTask(this, d_oreq, timeout, from));
        temp_tasks_mutex_.lock();
        temp_tasks_.push_back(t);
        temp_tasks_mutex_.unlock();
    }
    if (!graph_.vertices().empty()) {
        boost::asio::ip::udp::endpoint from;
        mtl::task_ptr t(new BroadcastClientsTask(this, d_oreq, timeout, from));
        temp_tasks_mutex_.lock();
        temp_tasks_.push_back(t);
        temp_tasks_mutex_.unlock();
    }
    // async process tasks
    dgram_->getIOService().post(boost::bind(&Server::activateTempTasks, this));
}

void Server::handlePacketArrival(InRequest& ireq, const boost::asio::ip::udp::endpoint& from, 
                                 int32_t milliseconds)
{
    uint32_t type = ireq.readCommand();
    if (type == PT_P2P_BROADCAST_DATA) {
        handleBroadcastData(ireq, from, milliseconds);
    } else if (type == PT_P2P_JOIN) {
        handleJoinRequest(ireq, from);
    } else if (type == PT_P2P_LEAVE) {
        handleLeaveRequest(ireq, from);
    } else if (type == PT_P2P_SEND_FAILED) {
        handleFailedReportRequest(ireq, from);
    } else {
        Client::handlePacketArrival(ireq, from, milliseconds);
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

void Server::handleGroupHeadArrival(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                                    bool* passed)
{
    assert(passed);
    if (isNeighbor(from) || isClient(from)) {
        ireq.skip(HEADER_LENGTH, SKIP_CURRENT);
        uint64_t seq = ireq.readInt64();
        *passed = packet_filter_.passed(seq);
    } else {
        *passed = false;
    }
}

// protocol: token | role
void Server::handleJoinRequest(InRequest& ireq, const boost::asio::ip::udp::endpoint& from)
{
    // check token
    std::string token = ireq.readString(64);
    if (token != token_)
        return;

    // check exists
    const std::string& from_name = toString(from);
    if (graph_.containsVertex(from_name)) {
        OutRequest oreq(PT_P2P_JOIN_ACK, 0, UDP_HEADER_LENGTH);
        oreq.writeInt64(0);
        oreq.writeInt64(0);
        dgram_->sendTo(oreq, from);
        return;
    }

    SequenceRange seq_range = { 0, 0 };
    uint8_t role = ireq.readInt8();
    if (role == CLIENT) {
        seq_range = getUsableSequenceRange(MAX_SEQUENCE_COUNT);
    } else {
        seq_range = getUsableSequenceRange(MAX_CLIENT_COUNT * MAX_SEQUENCE_COUNT);
    }
    const GraphVertex* from_vertex = nullptr;
    std::list<NearVertex> members;
    if (seq_range.size() > 0) {
        // add new member
        from_vertex = graph_.addVertex(from_name, role, from, seq_range);
        if (!from_vertex) {
            // failed
            OutRequest oreq(PT_P2P_JOIN_ACK, 0, UDP_HEADER_LENGTH);
            oreq.writeInt64(0);
            oreq.writeInt64(0);
            dgram_->sendTo(oreq, from);
            return;
        }
        // near members
        members = nearMembers(from_name, P2P_NEIGHBOR_COUNT);
    }
    // join ack
    {
        OutRequest oreq(PT_P2P_JOIN_ACK, 0, UDP_HEADER_LENGTH);
        oreq.writeInt64(seq_range.min);
        oreq.writeInt64(seq_range.max);
        if (seq_range.size() > 0) {
            // response
            oreq.writeInt16((uint16_t)members.size());
            char bytes[16];
            for (auto it = members.begin(), end = members.end(); it != end; ++it) {
                oreq.writeInt8((*it).vertex->role());
                const auto& address = (*it).vertex->address();
                oreq.writeBinary(toBytes(address, bytes), 16);
                oreq.writeInt16(address.port());
                // add an edge into the graph
                graph_.addEdge(from_vertex, (*it).vertex, (*it).weight);
            }
        }
        dgram_->sendTo(oreq, from);
    }
    // notify neighbors
    if (!members.empty()) {
        OutRequest oreq(PT_P2P_ADD_NEIGHBOR, 0, UDP_HEADER_LENGTH);
        oreq.writeInt8(1);
        oreq.writeInt8(role);
        char bytes[16];
        oreq.writeBinary(toBytes(from, bytes), 16);
        oreq.writeInt16(from.port());
        for (auto it = members.begin(), end = members.end(); it != end; ++it) {
            dgram_->sendTo(oreq, (*it).vertex->address());
        }
    }
}

// protocol: token
void Server::handleLeaveRequest(InRequest& ireq, const boost::asio::ip::udp::endpoint& from)
{
    // check token
    std::string token = ireq.readString(64);
    if (token == token_) {
        removeMember(toString(from));
    }
}

// protocol: token | role | ip | port
void Server::handleFailedReportRequest(InRequest& ireq, const boost::asio::ip::udp::endpoint& from)
{
    // check token
    std::string token = ireq.readString(64);
    if (token != token_)
        return;
    uint8_t role = ireq.readInt8();
    ((void)role);
    char bytes[16] = { 0 };
    ireq.readBinary(bytes, 16);
    uint16_t port = ireq.readInt16();
    boost::asio::ip::udp::endpoint to = toEndpoint(bytes, port);
    const std::string& to_name = toString(to);
    const std::string& from_name = toString(from);
    // change edge weight
    if (const GraphVertex* u = graph_.vertex(from_name)) {
        if (const GraphVertex* v = graph_.vertex(to_name)) {
            if (GraphEdge* e = graph_.edge(u, v)) {
                e->setWeight(MAX_EDGE_WEIGHT);
            }
        }
        // other replaces neighbor
        replaceNeighbor(u, from);
    }
    // new member
    std::list<NearVertex> members = nearMembers(from_name, 1);
    if (!members.empty()) {
        const NearVertex& nv = members.front();
        const auto& address = nv.vertex->address();
        OutRequest oreq(PT_P2P_ADD_NEIGHBOR, 0, UDP_HEADER_LENGTH);
        oreq.writeInt8(1);
        oreq.writeInt8(nv.vertex->role());
        oreq.writeBinary(toBytes(address, bytes), 16);
        oreq.writeInt16(address.port());
        dgram_->sendTo(oreq, from);
    }
}

void Server::handleCheckAliveTimeout(const boost::system::error_code& e)
{
    if (!e) {
        std::list<const GraphVertex*> removed;
        for (auto it = graph_.vertices().begin(), end = graph_.vertices().end();
             it != end; ++it) {
            if (it->second->incAliveTimeoutTimes() > 3) {
                removed.push_back(it->second);
            }
        }
        for (auto it = removed.begin(), end = removed.end(); it != end; ++it) {
            removeMember(*it);
        }
    }
    check_alive_timer_.expires_from_now(boost::posix_time::seconds(5));
    check_alive_timer_.async_wait(boost::bind(&Server::handleCheckAliveTimeout,
                                              boost::static_pointer_cast<Server>(shared_from_this()),
                                              boost::asio::placeholders::error));
}

void Server::handleAlive(const boost::asio::ip::udp::endpoint& from)
{
    if (GraphVertex* v = graph_.vertex(toString(from))) {
        v->setAliveTimeoutTimes(0);
    }
}

void Server::handleBroadcastData(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                                 int32_t milliseconds)
{
    if (!isClient(from) && !isNeighbor(from)) {
        return;
    }

    uint16_t size = ireq.size();

    // check whether the packet is duplicate
    uint64_t seq = ireq.readInt64();
    if (size <= UDP_DATA_SIZE && !packet_filter_.passed(seq))
        return;

    // stat big packet cost time
    statCost(from, size, milliseconds);

    // notify app
    InRequest d_ireq(ireq.buffer(), ireq.size(), (ireq.begin()+P2P_DATA_HEADER_LENGTH));
    packet_arrival_signal(d_ireq, from);

    // forward data to the neighbors
    uint16_t timeout = ireq.readInt16();
    if (size > UDP_DATA_SIZE && !neighbors_.empty() && (server_endpoint_ != from)) {
        if (timeout > milliseconds) {
            uint16_t left = timeout - (uint16_t)milliseconds;
            if (left > (milliseconds/3)) {
                OutRequest oreq(ireq.buffer(), ireq.size(), ireq.begin());
                oreq.skip(sizeof(uint64_t), SKIP_CURRENT);
                oreq.writeInt16(left);
                oreq.skip(0, SKIP_END);
                mtl::task_ptr t(new BroadcastNeighborsTask(this, oreq, left, from));
                addTask(t);
            }
        }
    }

    // forward data to clients
    if (timeout > milliseconds && graph_.vertices().size() > 1) {
        uint16_t left = timeout - (uint16_t)milliseconds;
        OutRequest oreq(ireq.buffer(), ireq.size(), ireq.begin());
        oreq.skip(0, SKIP_END);
        mtl::task_ptr t(new BroadcastClientsTask(this, oreq, left, from));
        addTask(t);
    }
}

SequenceRange Server::getUsableSequenceRange(uint64_t size)
{
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

int evaluateScale(const boost::asio::ip::udp::endpoint& a,
                  const boost::asio::ip::udp::endpoint& b)
{
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

std::list<Server::NearVertex> Server::nearMembers(const std::string& center,
                                                  uint32_t max_count)
{
    std::list<NearVertex> scale8;
    std::map<std::string, uint32_t>::const_iterator center_it = graph_.vertexNames().find(center);
    std::map<std::string, uint32_t>::const_iterator end = graph_.vertexNames().end();
    if (center_it == end)
        return scale8;

    std::map<std::string, uint32_t>::const_iterator begin = graph_.vertexNames().begin();
    GraphVertex* center_v = graph_.vertex(center_it->second);

    std::list<NearVertex> scale2;
    std::list<NearVertex> scale4;
    const auto& center_address = center_v->address();
    uint8_t center_role = center_v->role();
    int count = 0;
    std::map<std::string, uint32_t>::const_iterator it = center_it;
    for (++it; (it != end && ++count < 20); ++it) {
        GraphVertex* u = graph_.vertex(it->first);
        if (u->role() != center_role)
            continue;
        if (graph_.edge(u, center_v))
            continue;
        if (isValidNeighbor(u)) {
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
            GraphVertex* u = graph_.vertex(it->first);
            if (u->role() != center_role)
                continue;
            if (graph_.edge(u, center_v))
                continue;
            if (isValidNeighbor(u)) {
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

void Server::removeMember(const std::string& name)
{
    if (const GraphVertex* v = graph_.vertex(name)) {
        removeMember(v);
    }
}

void Server::removeMember(const GraphVertex* v)
{
    assert(v);
    // recycle sequence
    usable_sequences_.push_back(v->seqRange());
    packet_filter_.remove(v->seqRange().min, v->seqRange().max);

    // others replace neighbor
    const boost::asio::ip::udp::endpoint& ep = v->address();
    const auto& edges = graph_.relationships().edges(v);
    for (auto it = edges.begin(), end = edges.end(); it != end; ++it) {
        GraphVertex* t = graph_.vertex((*it)->target());
        GraphVertex* s = graph_.vertex((*it)->source());
        replaceNeighbor((t != v ? t : s), ep);
    }

    // remove the vertex from the graph
    graph_.removeVertex(toString(v->address()));
}

void Server::replaceNeighbor(const GraphVertex* v, const boost::asio::ip::udp::endpoint& neighbor)
{
    assert(v);
    char bytes[16];
    OutRequest oreq(PT_P2P_REPLACE_NEIGHBOR, 0, UDP_HEADER_LENGTH);
    oreq.writeInt8(1);
    oreq.writeBinary(toBytes(neighbor, bytes), 16);
    oreq.writeInt16(neighbor.port());
    std::list<NearVertex> members = nearMembers(toString(v->address()), 1);
    if (!members.empty()) {
        const NearVertex& nv = members.front();
        const auto& address = nv.vertex->address();
        oreq.writeInt8(1);
        oreq.writeInt8(nv.vertex->role());
        oreq.writeBinary(toBytes(address, bytes), 16);
        oreq.writeInt16(address.port());
    }
    dgram_->sendTo(oreq, v->address());
}

bool Server::isValidNeighbor(const GraphVertex* v)
{
    assert(v);
    int count = 0;
    const auto& edges = graph_.relationships().edges(v);
    for (auto it = edges.begin(), end = edges.end(); it != end; ++it) {
        if ((*it)->weight() < MAX_EDGE_WEIGHT) {
            if (++count >= P2P_NEIGHBOR_COUNT) {
                return false;
            }
        }
    }
    return true;
}

void Server::bfsMembers(const boost::asio::ip::udp::endpoint& start,
                        std::list<boost::asio::ip::udp::endpoint>& targets)
{
    GraphVertex* v = 0;
    if (start.port() > 0) {
        v = graph_.vertex(toString(start));
    } else {
        if (!graph_.vertices().empty()) {
            v = graph_.vertices().begin()->second;
        }
    }
    if (v) {
        graph_.breadthFirstSearch(v, targets);
    }
}
}
}
}
