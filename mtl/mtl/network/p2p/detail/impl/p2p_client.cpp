#include "mtl/network/p2p/client.hpp"
#include <boost/asio/placeholders.hpp>
#include <boost/asio/ip/address.hpp>
#include "mtl/network/in_request.hpp"
#include "mtl/network/udp/protocol.hpp"
#include "mtl/network/p2p/protocol.hpp"
#include "mtl/network/p2p/detail/p2p_broadcast_task.hpp"

namespace mtl {
namespace network {

using namespace udp;

namespace p2p {

Client::Client(const DgramPtr& dgram, Role role)
    : dgram_(dgram), timeout_(false), seq_range_({ 0, 0xffffffffffffffff })
    , role_(role), status_(kIdleState), task_group_(0, false, 0)
{
}

Client::~Client()
{
    close();
}

bool Client::open(const UdpEndpoint& endpoint)
{
    bool ret = dgram_->open(endpoint);
    if (ret) {
        dgram_->sent_signal.connect(boost::bind(&Client::handlePacketSent, this, _1, _2, _3));
        dgram_->arrival_signal.connect(boost::bind(&Client::handlePacketArrival, this, _1, _2, _3));
        dgram_->timeout_signal.connect(boost::bind(&Client::handlePacketTimeout, this, _1, _2));
        dgram_->group_head_arrival_signal.connect(boost::bind(&Client::handleGroupHeadArrival,
                                                              this, _1, _2, _3));
        dgram_->tick_signal.connect(boost::bind(&Client::processTasks, this));
        // initial
        timeout_ = false;
        status_ = kIdleState;
        role_ = kClientRole;
    }
    return ret;
}

void Client::close()
{
    clearTempTasks();
    dgram_->close();
}

UdpEndpoint Client::localEndpoint() const
{
    return dgram_->localEndpoint();
}

bool Client::isNeighbor(const UdpEndpoint& endpoint) const
{
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
void Client::join(const UdpEndpoint& to, const std::string& access_token)
{
    if (status_ == kIdleState) {
        // change status
        server_endpoint_ = to;
        access_token_ = access_token;
        status_ = kJoiningState;
        // notify server
        OutRequest oreq(PT_P2P_JOIN, 0, kUdpHeaderLength);
        oreq.writeVersion(P2P_VERSION);
        oreq.writeString(access_token);
        oreq.writeInt8(uint8_t(role_));
        dgram_->sendTo(oreq, to, 10000);
    }
}

// protocol: seq | timeout | data
void Client::broadcast(OutRequest& oreq, uint32_t timeout)
{
    if (!isMember()) {
        return;
    }
    assert(oreq.begin() == P2P_DATA_HEADER_LENGTH);
    // filter sent out packet
    uint64_t seq = genSeq();
    packet_filter_.isPassed(seq);
    // request
    OutRequest d_oreq(oreq.buffer(), oreq.size());
    d_oreq.writeCommand(PT_P2P_BROADCAST_DATA);
    d_oreq.writeSequence(Dgram::nextSequence());
    d_oreq.writeLength(oreq.size());
    d_oreq.writeVersion(P2P_VERSION);
    d_oreq.writeInt64(seq);
    d_oreq.writeInt16(uint16_t(timeout));
    d_oreq.skip(0, kSkipEnd);
    // At first, send data to the server
    if (server_endpoint_.port() > 0) {
        dgram_->sendTo(d_oreq, server_endpoint_, timeout);
    }
    // send data to the neighbors
    if (d_oreq.size() > kUdpDataSize && !neighbors_.empty()) {
        UdpEndpoint from;
        mtl::TaskPtr t(new BroadcastNeighborsTask(this, d_oreq, int(timeout), from));
        temp_tasks_mutex_.lock();
        temp_tasks_.push_back(t);
        temp_tasks_mutex_.unlock();
        // async process tasks
        dgram_->context().post(boost::bind(&Client::activateTempTasks, this));
    }
}

void Client::sendTo(OutRequest& oreq, const UdpEndpoint& to, uint32_t timeout)
{
    dgram_->sendTo(oreq, to, timeout);
}

// protocol: token | role
void Client::leave()
{
    clearTempTasks();
    dgram_->clearPackets();
    if (isMember()) {
        OutRequest oreq(PT_P2P_LEAVE, 0, kUdpHeaderLength);
        oreq.writeVersion(P2P_VERSION);
        oreq.writeString(access_token_);
        oreq.writeInt8(uint8_t(role_));
        dgram_->sendTo(oreq, server_endpoint_);
        status_ = kIdleState;
    }
}

void Client::handlePacketArrival(InRequest& ireq, const UdpEndpoint& from, 
                                 int32_t milliseconds)
{
    uint32_t type = ireq.readCommand();
    if (type == PT_P2P_BROADCAST_DATA) {
        handleBroadcastData(ireq, from, milliseconds);
    } else if (type == PT_P2P_DIRECT_DATA) {
        InRequest d_ireq(ireq.buffer(), ireq.size(), P2P_DATA_HEADER_LENGTH);
        packet_arrival_signal(d_ireq, from);
    } else if (type == PT_P2P_ADD_NEIGHBOR) {
        handleAddNeighbor(ireq, from);
    } else if (type == PT_P2P_REPLACE_NEIGHBOR) {
        handleReplaceNeighbor(ireq, from);
    } else if (type == PT_P2P_JOIN_ACK) {
        handleJoinAck(ireq, from);
    }
}

void Client::handleGroupHeadArrival(InRequest& ireq, const UdpEndpoint& from,
                                    bool* passed)
{
    assert(passed);
    if (isNeighbor(from)) {
        ireq.skip(kHeaderLength, kSkipCurrent);
        uint64_t seq = ireq.readInt64();
        *passed = packet_filter_.isPassed(seq);
    } else {
        *passed = false;
    }
}

void Client::handlePacketSent(const OutRequest& oreq, const UdpEndpoint& to,
                              int32_t milliseconds)
{
    if (to == server_endpoint_) {
        timeout_ = false;
    } else {
        // stat cost time
        statCost(to, oreq.size(), milliseconds);
    }
    processTasks();
}

void Client::handlePacketTimeout(const OutRequest& oreq, const UdpEndpoint& to)
{
    InRequest ireq(oreq.buffer(), oreq.size(), oreq.begin());
    uint32_t type = ireq.readCommand();
    if (type == PT_P2P_JOIN) {
        if (status_ == kJoiningState) {
            status_ = kIdleState;
            join_failed_signal();
        }
    } else if (type == PT_P2P_BROADCAST_DATA) {
        handleBroadcastDataTimeout(oreq, to);
    }
    processTasks();
}

// protocol: sequence range(min,max) | neighbor count | {neighbor(endpoint, role)}
void Client::handleJoinAck(InRequest& ireq, const UdpEndpoint& /*from*/)
{
    if (status_ != kJoiningState) {
        return;
    }
    clearTempTasks();
    task_group_.removeAll();
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
            neighbors_.push_back({ role, toEndpoint(bytes, port), 0, 0, 50 });
        }
        status_ = kJoined;
        dgram_->keepAlive(server_endpoint_, true);
        join_finished_signal();
    } else {
        status_ = kIdleState;
        join_failed_signal();
    }
}

// protocol: role | ip | port
void Client::handleAddNeighbor(InRequest& ireq, const UdpEndpoint& /*from*/)
{
    if (isMember()) {
        uint8_t count = ireq.readInt8();
        (void)count;
        uint8_t role = ireq.readInt8();
        char bytes[16] = { 0 };
        ireq.readBinary(bytes, 16);
        uint16_t port = ireq.readInt16();
        UdpEndpoint endpoint = toEndpoint(bytes, port);
        if (!isNeighbor(endpoint)) {
            neighbors_.push_back({role, endpoint, 0, 0, 50});
        }
    }
}

// protocol: count | {role | ip | port} | count | {role | ip : port}
void Client::handleReplaceNeighbor(InRequest& ireq, const UdpEndpoint& /*from*/)
{
    if (isMember()) {
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
            removeNeighbor(toEndpoint(bytes, port));
        }
        // added
        count = ireq.readInt8();
        for (uint8_t i = 0; i < count; ++i) {
            role = ireq.readInt8();
            ireq.readBinary(bytes, 16);
            port = ireq.readInt16();
            address = toEndpoint(bytes, port);
            if (isNeighbor(address)) {
                neighbors_.push_back({role, address, 0, 0, 50});
            }
        }
    }
}

// protocol: seq | timeout | data
void Client::handleBroadcastData(InRequest& ireq, const UdpEndpoint& from,
                                 int32_t milliseconds)
{
    if (!isMember() || !isNeighbor(from)) {
        return;
    }

    uint16_t size = ireq.size();

    // check whether the packet is duplicate
    uint64_t seq = ireq.readInt64();
    if (size <= udp::kUdpDataSize && !packet_filter_.isPassed(seq)) {
        return;
    }

    // stat big packet cost time
    statCost(from, size, milliseconds);

    // notify app
    InRequest d_ireq(ireq.buffer(), ireq.size(), (ireq.begin()+P2P_DATA_HEADER_LENGTH));
    packet_arrival_signal(d_ireq, from);

    // forward data to the neighbors
    if (size > kUdpDataSize && false && !neighbors_.empty() && (server_endpoint_ != from)) {
        uint16_t timeout = ireq.readInt16();
        if (timeout > milliseconds) {
            uint16_t left = timeout - uint16_t(milliseconds);
            if (left > (milliseconds/3)) {
                OutRequest oreq(ireq.buffer(), ireq.size(), ireq.begin());
                oreq.skip(sizeof(uint64_t), kSkipCurrent);
                oreq.writeInt16(left);
                oreq.skip(0, kSkipEnd);
                mtl::TaskPtr t(new BroadcastNeighborsTask(this, oreq, left, from));
                appendTask(t);
            }
        }
    }
}

// protocol: token | role | ip | port
void Client::handleBroadcastDataTimeout(const OutRequest& oreq, const UdpEndpoint& to)
{
    if (!isMember()) {
        return;
    }
    // if failed to send to server, notify leave
    if (to == server_endpoint_) {
        if (timeout_) {
            if (timeout_end_time_ <= std::chrono::system_clock::now()) {
                status_ = kIdleState;
                leave_signal();
            }
        } else {
            timeout_ = true;
            timeout_end_time_ = std::chrono::system_clock::now() + std::chrono::seconds(10);
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
            dgram_->clearPackets(to);
            neighbors_.erase(it);
            // notify server
            if (server_endpoint_.port() > 0) {
                OutRequest oreq(PT_P2P_SEND_FAILED, 0, kUdpHeaderLength);
                oreq.writeVersion(P2P_VERSION);
                oreq.writeString(access_token_);
                oreq.writeInt8(uint8_t(role_));
                char bytes[16];
                oreq.writeBinary(toBytes(to, bytes), 16);
                oreq.writeInt16(to.port());
                dgram_->sendTo(oreq, server_endpoint_);
            }
        }
    }
    // notify
    packet_timeout_signal(oreq, to);
}

void Client::removeNeighbor(const UdpEndpoint& address)
{
    auto it = std::find_if(neighbors_.begin(), neighbors_.end(), [&address](const Neighbor& n) {
        return (n.address == address);
    });
    if (it != neighbors_.end()) {
        dgram_->clearPackets(address);
        neighbors_.erase(it);
    }
}

void Client::activateTempTasks()
{
    temp_tasks_mutex_.lock();
    for (auto it = temp_tasks_.begin(), end = temp_tasks_.end();
         it != end; ++it) {
        task_group_.append(*it);
    }
    temp_tasks_.clear();
    temp_tasks_mutex_.unlock();
    processTasks();
}

void Client::processTasks()
{
    if (task_group_.isInactive() || task_group_.isCompleted()) {
        task_group_.activate();
    }
    task_group_.process();
}

void Client::clearTempTasks()
{
    temp_tasks_mutex_.lock();
    temp_tasks_.clear();
    temp_tasks_mutex_.unlock();
}

void Client::appendTask(const TaskPtr& t)
{
    const int MAX_TASK_COUNT = 25;
    task_group_.append(t);
    if (task_group_.count() >= MAX_TASK_COUNT) {
        task_group_.removeFirst();
    }
    processTasks();
}

void Client::statCost(const UdpEndpoint& from, uint16_t size, int32_t milliseconds)
{
    auto it = std::find_if(neighbors_.begin(), neighbors_.end(), [&from](const Neighbor& n) {
        return (n.address == from);
    });
    if (it != neighbors_.end()) {
        if (size >= 1024) {
            (*it).cost_per_k = ((*it).cost_per_k +
                                int32_t(double(milliseconds)/(double(size)/1024.0))) / 2;
        } else {
            (*it).cost_per_k = ((*it).cost_per_k + milliseconds) / 2;
        }
    }
}

uint64_t Client::genSeq() const
{
    static uint64_t seq = seq_range_.min;
    seq++;
    if ((seq % seq_range_.max) == 0) {
        seq = seq_range_.min;
    }
    return seq;
}

} // namespace p2p
} // namespace network
} // namespace mtl
