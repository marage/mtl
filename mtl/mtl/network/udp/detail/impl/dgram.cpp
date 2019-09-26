#include "mtl/network/udp/dgram.hpp"
#include <time.h>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "mtl/network/in_request.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/udp/detail/dgram_group_recv_task.hpp"
#include "mtl/network/udp/detail/dgram_group_send_task.hpp"

namespace posix_time = boost::posix_time;
namespace asio = boost::asio;

namespace mtl {
namespace network {
namespace udp {

Dgram::Dgram(boost::asio::io_context& io_context)
    : io_context_(io_context), socket_(io_context), timer_(io_context)
    , open_(false), closing_(false), alive_(false), timer_frequency_(5000)
{
}

Dgram::~Dgram()
{
}

bool Dgram::open(const boost::asio::ip::udp::endpoint& endpoint, uint32_t frequency)
{
    if (open_) {
        return false;
    }
    boost::system::error_code ec;
    socket_.open(asio::ip::udp::v4(), ec);
    if (ec) {
        return false;
    }
    socket_.bind(endpoint, ec);
    if (ec) {
        socket_.close();
        return false;
    }

    // options
    socket_.set_option(asio::socket_base::receive_buffer_size(8192));
    socket_.set_option(asio::socket_base::send_buffer_size(8192));

    open_ = true;
    closing_ = false;
    timer_frequency_ = frequency;
    asyncReceiveFrom();
    timer_.expires_from_now(posix_time::milliseconds(timer_frequency_));
    timer_.async_wait(boost::bind(&Dgram::handleTimeout, this,
                                  asio::placeholders::error));
    return true;
}

void Dgram::close()
{
    if (open_) {
        // disconnect signals
        arrival_signal.disconnect_all_slots();
        timeout_signal.disconnect_all_slots();
        group_head_arrival_signal.disconnect_all_slots();
        keep_alive_signal.disconnect_all_slots();
        timer_tick_signal.disconnect_all_slots();
        // post close
        boost::asio::post(io_context_, boost::bind(&Dgram::handleClose, shared_from_this()));
        int try_count = 0;
        while (open_ && ++try_count <= 100) {
            boost::this_thread::sleep(posix_time::milliseconds(1));
        }
    }
}

boost::asio::ip::udp::endpoint Dgram::localEndpoint() const
{
    boost::system::error_code ec;
    return socket_.local_endpoint(ec);
}

bool Dgram::isBusy()
{
    bool busy;
    mutex_.lock();
    busy = (pending_packets_.size() > 5);
    if (!busy) {
        busy = (group_send_queue_.size() > 5);
        if (!busy) {
            busy = (active_group_send_tasks_.size() > 5);
        }
    }
    mutex_.unlock();
    return busy;
}

bool Dgram::sendTo(const OutRequest& oreq, const boost::asio::ip::udp::endpoint& to, uint32_t timeout)
{
    if (!open_)
        return false;
    posix_time::ptime end_time = posix_time::microsec_clock::local_time() +
            posix_time::milliseconds(timeout);
    if (oreq.begin() >= UDP_HEADER_LENGTH && oreq.size() <= UDP_PACKET_SIZE) {
        OutRequest d_oreq(oreq.buffer(), oreq.size());
        d_oreq.writeCommand(timeout > 0 ? PT_RUDP : PT_UDP);
        d_oreq.writeSequence(nextSequence());
        d_oreq.skip(0, SKIP_END);
        mutex_.lock();
        small_packet_queue_.push_back(SmallPacket{ d_oreq, to, end_time });
        mutex_.unlock();
    } else if (oreq.size() > UDP_DATA_SIZE) {
        GroupSendTask* p = new GroupSendTask(this, oreq.buffer(), oreq.size(), to, end_time);
        mutex_.lock();
        group_send_queue_.push_back(p);
        mutex_.unlock();
    } else {
        OutRequest d_oreq((timeout > 0 ? PT_RUDP : PT_UDP), nextSequence());
        d_oreq.writeBinary(oreq.buffer().get(), oreq.size());
        mutex_.lock();
        small_packet_queue_.push_back(SmallPacket{ d_oreq, to, end_time });
        mutex_.unlock();
    }
    boost::asio::post(io_context_, boost::bind(&Dgram::sendNextPacket, this));
    return false;
}

void Dgram::keepAlive(const boost::asio::ip::udp::endpoint& to, bool alive)
{
    alive_ = alive;
    alive_endpoint_ = to;
    if (alive) {
        last_alive_time_ = posix_time::microsec_clock::local_time()
                + posix_time::seconds(5);
    }
}

void Dgram::clearPackets(const boost::asio::ip::udp::endpoint& to)
{
    bool exists = false;
    mutex_.lock();
    for (auto it = small_packet_queue_.begin(); it != small_packet_queue_.end();) {
        if ((*it).to == to) {
            it = small_packet_queue_.erase(it);
        } else {
            ++it;
        }
    }
    if (!active_group_send_tasks_.empty()) {
        posix_time::ptime end_time = posix_time::microsec_clock::local_time() +
                posix_time::time_duration(posix_time::milliseconds(500));
        for (auto it = active_group_send_tasks_.begin(); it != active_group_send_tasks_.end();) {
            if ((*it)->to() == to) {
                OutRequest oreq(PT_RUDP, nextSequence());
                oreq.writeInt16((*it)->id());
                oreq.writeInt8(PT_GROUP_CANCEL);
                oreq.writeLength(oreq.size());
                small_packet_queue_.push_back(SmallPacket{ oreq, (*it)->to(), end_time });
                delete *it;
                it = active_group_send_tasks_.erase(it);
                exists = true;
            } else {
                ++it;
            }
        }
    }
    for (auto it = group_send_queue_.begin(); it != group_send_queue_.end();) {
        if ((*it)->to() == to) {
            delete *it;
            it = group_send_queue_.erase(it);
        } else {
            ++it;
        }
    }
    mutex_.unlock();
    if (exists) {
        boost::asio::post(io_context_, boost::bind(&Dgram::sendNextPacket, this));
    }
}

void Dgram::clearPackets()
{
    posix_time::ptime end_time = posix_time::microsec_clock::local_time() +
            posix_time::time_duration(posix_time::milliseconds(500));
    mutex_.lock();
    small_packet_queue_.clear();
    pending_packets_.clear();
    bool exists = !active_group_send_tasks_.empty();
    if (exists) {
        for (auto it = active_group_send_tasks_.begin(), end = active_group_send_tasks_.end();
             it != end; ++it) {
            OutRequest oreq(PT_RUDP, nextSequence());
            oreq.writeInt16((*it)->id());
            oreq.writeInt8(PT_GROUP_CANCEL);
            oreq.writeLength(oreq.size());
            small_packet_queue_.push_back(SmallPacket{ oreq, (*it)->to(), end_time });
            delete *it;
        }
    }
    while (!group_send_queue_.empty()) {
        delete group_send_queue_.front();
        group_send_queue_.pop_front();
    }
    mutex_.unlock();
    boost::asio::post(io_context_, boost::bind(&Dgram::sendNextPacket, this));
}

void Dgram::sendNextPacket()
{
    posix_time::ptime now = posix_time::microsec_clock::local_time();
    boost::mutex& mutex = mutex_;
    auto& small_packet_queue = small_packet_queue_;
    auto& group_sending_tasks = active_group_send_tasks_;
    auto& group_send_queue = group_send_queue_;

    // send small packets
    while (1) {
        mutex.lock();
        if (small_packet_queue.empty()) {
            mutex.unlock();
            break;
        } else {
            SmallPacket& sp = small_packet_queue.front();
            if (sp.end_time > now) {
                if (!destIsRecving(sp.to)) {
                    asyncSendTo(sp.oreq, sp.to, 0);
                    // udp
                    if (sp.oreq.command() == PT_RUDP) {
                        posix_time::ptime next_time = now + posix_time::milliseconds(UDP_TRYOUT_TIMEOUT);
                        pending_packets_.insert(
                                    std::make_pair(sp.oreq.sequence(),
                                                   PendingPacket{ sp.oreq, sp.to, sp.end_time, next_time }));
                    }
                    small_packet_queue.pop_front();
                }
                mutex.unlock();
                break;
            } else {
                SmallPacket sp2 = sp;
                small_packet_queue.pop_front();
                mutex.unlock();
                timeout_signal(OutRequest(sp2.oreq.buffer(), sp2.oreq.size(), UDP_HEADER_LENGTH), sp2.to);
            }
        }
    }
    // select a new group_send_task from the send queue
    while (1) {
        mutex.lock();
        if (group_sending_tasks.size() < 5 && group_send_queue.size() > 0) {
            GroupSendTask* t = group_send_queue.front();
            if (t->end_time() > now) {
                if (!destIsRecving(t->to())) {
                    t->handleSendComplete();
                    group_sending_tasks.push_back(t);
                    group_send_queue.pop_front();
                }
                mutex.unlock();
                break;
            } else {
                group_send_queue.pop_front();
                mutex.unlock();
                timeout_signal(OutRequest(t->buffer(), t->size()), t->to());
                delete t;
            }
        } else {
            mutex.unlock();
            break;
        }
    }
}

void Dgram::sendPacketAck(uint32_t seq, const boost::asio::ip::udp::endpoint& to)
{
    OutRequest oreq(PT_ACK, 0);
    oreq.writeInt32(seq);
    oreq.writeLength(oreq.size());
    asyncSendTo(oreq, to, 0);
}

void Dgram::asyncReceiveFrom()
{
    buffer_ = SingletonBufferPool::getSingleton().malloc(UDP_PACKET_SIZE);
    socket_.async_receive_from(asio::buffer(buffer_.get(), UDP_PACKET_SIZE), sender_endpoint_,
                               boost::bind(&Dgram::handleReceiveFrom, this,
                                           asio::placeholders::error,
                                           asio::placeholders::bytes_transferred));
}

void Dgram::asyncSendTo(OutRequest& oreq, const boost::asio::ip::udp::endpoint& to, uint32_t timeout)
{
    if (!closing_) {
        oreq.writeLength(oreq.size());
        socket_.async_send_to(asio::buffer(oreq.buffer().get(), oreq.size()), to,
                              boost::bind(&Dgram::handleSendTo, this,
                                          oreq.buffer(), asio::placeholders::error,
                                          asio::placeholders::bytes_transferred));
        // udp
        if (timeout > 0) {
            posix_time::ptime now = posix_time::microsec_clock::local_time();
            posix_time::ptime end_time = now + posix_time::milliseconds(timeout);
            posix_time::ptime next_time = now + posix_time::milliseconds(UDP_TRYOUT_TIMEOUT);
            pending_packets_.insert(
                        std::make_pair(oreq.sequence(),
                                       PendingPacket{ oreq, to, end_time, next_time }));
        }
    }
}

void Dgram::handleReceiveFrom(const boost::system::error_code& error,
                              size_t bytes_transferred)
{
    if (!closing_) {
        if (!error && bytes_transferred > 0) {
            handleReceiveFinished(bytes_transferred);
        }
        asyncReceiveFrom();
    }
}

void Dgram::handleReceiveFinished(size_t bytes_recvd)
{
    InRequest ireq(buffer_, static_cast<uint16_t>(bytes_recvd), 0);
    uint16_t len = ireq.readLength();
    if (len != bytes_recvd)
        return;

    uint32_t type = ireq.readCommand();
    uint32_t seq = ireq.readSequence();
    if (type == PT_RUDP) {
        sendPacketAck(seq, sender_endpoint_);
    } else if (type == PT_ACK) {
        handlePacketAck(ireq);
        return;
    } else if (type == PT_KEEPALIVE) {
        keep_alive_signal(sender_endpoint_);
        return;
    }
    if (packet_record_.passed(seq, sender_endpoint_)) {
        switch (type) {
        case PT_RUDP:
        {
            InRequest ir(buffer_, uint16_t(bytes_recvd), UDP_HEADER_LENGTH);
            ir.setFrom(sender_endpoint_);
            arrival_signal(ir, sender_endpoint_, 10);
        }
            break;
        case PT_GROUP_OUT:
            handleReceiveGroupPacket(ireq, sender_endpoint_);
            break;
        case PT_GROUP_IN:
            handleReceiveGroupPacketAck(ireq, sender_endpoint_);
            break;
        case PT_UDP:
        {
            InRequest ir(buffer_, uint16_t(bytes_recvd), UDP_HEADER_LENGTH);
            ir.setFrom(sender_endpoint_);
            arrival_signal(ir, sender_endpoint_, 0);
        }
            break;
        default:
            break;
        }
    }
}

void Dgram::handlePacketAck(InRequest& ireq)
{
    uint32_t seq = ireq.readInt32();
    std::map<uint32_t, PendingPacket>::iterator it = pending_packets_.find(seq);
    if (it != pending_packets_.end()) {
        sent_signal(OutRequest((*it).second.oreq.buffer(), UDP_HEADER_LENGTH), (*it).second.to, 0);
        pending_packets_.erase(it);
    }
}

void Dgram::handleReceiveGroupPacket(InRequest& ireq, const boost::asio::ip::udp::endpoint& from)
{
    uint16_t id = ireq.readInt16();
    GroupRecvTask* grt = getRecvTask(id, from);
    if (!grt) {
        uint16_t cur_pos = ireq.left();
        ireq.skip(sizeof(uint8_t), SKIP_CURRENT);
        uint16_t count = ireq.readInt16();
        ireq.skip(cur_pos, SKIP_END);
        if (!group_record_.exists(id, count, from)) {
            OutRequest oreq(PT_GROUP_IN, nextSequence());
            oreq.writeInt16(id);
            oreq.writeInt8(PT_GROUP_REPORT);
            oreq.writeInt16(count);
            asyncSendTo(oreq, from, 0);
        } else {
            // not found, it's a possible new packet
            grt = new GroupRecvTask(this, id, from);
            if (grt) {
                active_group_recv_tasks_.push_back(grt);
            }
        }
    }
    if (grt) {
        grt->handleReceivePacket(ireq, from);
        if (grt->isCompleted() || grt->isFailed()) {
            for (auto it = active_group_recv_tasks_.begin(), end = active_group_recv_tasks_.end();
                 it != end; ++it) {
                if ((*it)->compareId(id, from)) {
                    group_record_.append(id, grt->blockCount(), from);
                    if (grt->isCompleted()) {
                        InRequest ireq(grt->data(), grt->size());
                        ireq.setFrom(from);
                        arrival_signal(ireq, from, int32_t(grt->cost()));
                    }
                    active_group_recv_tasks_.erase(it);
                    break;
                }
            }
            delete grt;
        }
    }
}

void Dgram::handleReceiveGroupPacketAck(InRequest& ireq, const boost::asio::ip::udp::endpoint& from)
{
    uint16_t id = ireq.readInt16();
    for (auto it = active_group_send_tasks_.begin(), end = active_group_send_tasks_.end();
         it != end; ++it) {
        GroupSendTask* p = *it;
        if (p->id() == id) {
            p->handleReceivePacket(ireq, from);
            if (p->isCompleted() || p->isFailed()) {
                if (p->isCompleted())
                    sent_signal(OutRequest(p->buffer(), p->size()), p->to(), p->cost());
                else
                    timeout_signal(OutRequest(p->buffer(), p->size()), p->to());
                delete p;
                active_group_send_tasks_.erase(it);
                break;
            }
        }
    }
}

void Dgram::handleSendTo(const SharedBuffer& /*buffer*/,
                         const boost::system::error_code& /*error*/, size_t /*bytes_transferred*/)
{
    if (!closing_) {
        for (auto it = active_group_send_tasks_.begin(), end = active_group_send_tasks_.end();
             it != end; ++it) {
            (*it)->handleSendComplete();
        }
        sendNextPacket();
    }
}

void Dgram::handleTimeout(const boost::system::error_code& /*e*/)
{
    if (closing_)
        return;
    posix_time::ptime now = posix_time::microsec_clock::local_time();
    auto& mutex = mutex_;
    // pending packets
    {
        auto& pending_packets = pending_packets_;
        bool unlocked = false;
        mutex.lock();
        for (auto it = pending_packets.begin(), end = pending_packets.end();
             it != end; ++it) {
            PendingPacket& pp = it->second;
            if (now < pp.end_time) {
                if (now >= pp.next_time) {
                    asyncSendTo(pp.oreq, pp.to, 0);
                    pp.next_time = now + posix_time::milliseconds(UDP_TRYOUT_TIMEOUT);
                }
            } else {
                PendingPacket pp2 = pp;
                pending_packets.erase(it);
                mutex.unlock();
                unlocked = true;
                timeout_signal(OutRequest(pp2.oreq.buffer(), pp2.oreq.size(), UDP_HEADER_LENGTH),
                               pp2.to);
                break;
            }
        }
        if (!unlocked) {
            mutex.unlock();
        }
    }
    // sending packets
    {
        auto& group_sending_tasks = active_group_send_tasks_;
        bool unlocked = false;
        mutex_.lock();
        for (auto it = group_sending_tasks.begin(), end = group_sending_tasks.end();
             it != end; ++it) {
            (*it)->handleTimeout(now);
            if ((*it)->isFailed()) {
                GroupSendTask* t = *it;
                group_sending_tasks.erase(it);
                mutex.unlock();
                unlocked = true;
                timeout_signal(OutRequest(t->buffer(), t->size()), t->to());
                delete t;
                break;
            }
        }
        if (!unlocked) {
            mutex.unlock();
        }
    }
    // receiving packets
    for (auto it = active_group_recv_tasks_.begin(), end = active_group_recv_tasks_.end();
         it != end; ++it) {
        (*it)->handleTimeout(now);
        if ((*it)->isFailed()) {
            delete *it;
            active_group_recv_tasks_.erase(it);
            break;
        }
    }
    // alive
    if (alive_ && last_alive_time_ < now) {
        OutRequest oreq(PT_KEEPALIVE, 0);
        asyncSendTo(oreq, alive_endpoint_, 0);
        last_alive_time_ = now + posix_time::seconds(5);
    }
    // timer tick
    timer_tick_signal();
    // another timer
    timer_.expires_from_now(posix_time::milliseconds(timer_frequency_));
    timer_.async_wait(boost::bind(&Dgram::handleTimeout, this,
                                  asio::placeholders::error));
}

void Dgram::handleClose()
{
    if (!open_)
        return;

    closing_ = true;

    auto& small_packet_queue = small_packet_queue_;
    auto& group_send_queue = group_send_queue_;
    mutex_.lock();
    while (!small_packet_queue.empty()) {
        small_packet_queue.pop_front();
    }
    while (!group_send_queue.empty()) {
        delete group_send_queue.front();
        group_send_queue.pop_front();
    }
    mutex_.unlock();

    pending_packets_.clear();

    for (auto it = active_group_send_tasks_.begin(), end = active_group_send_tasks_.end();
         it != end; ++it)
        delete *it;
    active_group_send_tasks_.clear();

    for (auto it = active_group_recv_tasks_.begin(), end = active_group_recv_tasks_.end();
         it != end; ++it)
        delete *it;
    active_group_recv_tasks_.clear();

    boost::system::error_code ec;
    socket_.shutdown(asio::ip::udp::socket::shutdown_both, ec);
    socket_.close();
    timer_.cancel(ec);
    open_ = false;
}

GroupRecvTask* Dgram::getRecvTask(uint16_t id, const boost::asio::ip::udp::endpoint& from)
{
    GroupRecvTask* t = nullptr;
    for (auto it = active_group_recv_tasks_.begin(), end = active_group_recv_tasks_.end();
         it != end; ++it) {
        if ((*it)->compareId(id, from)) {
            t = (*it);
            break;
        }
    }
    return t;
}

bool Dgram::destIsRecving(const boost::asio::ip::udp::endpoint& dest)
{
    bool recving = false;
    for (auto it = pending_packets_.begin(), end = pending_packets_.end();
         it != end; ++it) {
        if (dest == it->second.to) {
            recving = true;
            break;
        }
    }
    if (!recving) {
        for (auto it = active_group_send_tasks_.begin(), end = active_group_send_tasks_.end();
             it != end; ++it) {
            if (dest == (*it)->to()) {
                recving = true;
                break;
            }
        }
    }
    return recving;
}

uint32_t Dgram::nextSequence()
{
    static uint32_t seq = uint32_t(time(nullptr));
    ++seq;
    seq = seq % 0xFFFFFFFF;
    return seq;
}
} // udp
} // network
} // mtl
