#include "mtl/network/udp/dgram.hpp"
#include <functional>
#include <time.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "mtl/network/in_request.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/udp/detail/dgram_group_receive_task.hpp"
#include "mtl/network/udp/detail/dgram_group_send_task.hpp"

namespace asio = boost::asio;

namespace mtl {
namespace network {
namespace udp {

const int BUFFER_SIZE = 8192;

Dgram::Dgram(asio::io_context& io_context)
  : socket_(io_context), thread_(nullptr), running_(false)
  , open_(false), closing_(false), alive_(false)
  , available_send_buffer_size_(BUFFER_SIZE) {
}

Dgram::~Dgram() {
}

bool Dgram::open(const UdpEndpoint& endpoint) {
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
  socket_.set_option(asio::socket_base::receive_buffer_size(BUFFER_SIZE));
  socket_.set_option(asio::socket_base::send_buffer_size(BUFFER_SIZE));

  open_ = true;
  running_ = true;
  closing_ = false;
  thread_ = new std::thread(std::bind(&Dgram::MainLoop, this));

  return true;
}

void Dgram::close() {
  if (!open_) {
    return;
  }

  // stop thread
  running_ = false;
  thread_->join();
  delete thread_;
  thread_ = nullptr;

  // disconnect signals
  arrival_signal.disconnect_all_slots();
  timeout_signal.disconnect_all_slots();
  group_head_arrival_signal.disconnect_all_slots();
  keep_alive_signal.disconnect_all_slots();
  tick_signal.disconnect_all_slots();

  // post close
  socket_.get_io_service().post(boost::bind(&Dgram::HandleClose,
                                            shared_from_this()));
  int try_count = 0;
  while (open_ && ++try_count <= 100) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

asio::io_service& Dgram::GetIOService() {
  return socket_.get_io_service();
}

UdpEndpoint Dgram::LocalEndpoint() const {
  boost::system::error_code ec;
  return socket_.local_endpoint(ec);
}

bool Dgram::IsBusy() {
  bool busy;
  mutex_.lock();
  busy = (pending_packets_.size() > 5);
  if (!busy) {
    busy = (inactive_group_send_tasks_.size() > 5);
    if (!busy) {
      busy = (active_group_send_tasks_.size() > 5);
    }
  }
  mutex_.unlock();
  return busy;
}

bool Dgram::SendTo(const OutRequest& oreq, const UdpEndpoint& to, uint32_t timeout) {
  if (!open_) {
    return false;
  }
  TimePoint end_time = SystemClock::now() + std::chrono::milliseconds(timeout);
  uint16_t size = oreq.size();
  if (oreq.begin() >= kUdpHeaderLength && size <= kUdpMaxPacketSize) {
    OutRequest d_oreq(oreq.buffer(), size);
    d_oreq.writeCommand(timeout > 0 ? kRudpType : kUdpType);
    d_oreq.writeSequence(NextSequence());
    d_oreq.skip(0, kSkipEnd);
    mutex_.lock();
    small_packets_.push_back(SmallPacket{ d_oreq, to, end_time });
    mutex_.unlock();
  } else if (size > kUdpDataSize) {
    GroupSendTask* p = new GroupSendTask(this, oreq.buffer(), size, to, end_time);
    mutex_.lock();
    inactive_group_send_tasks_.push_back(p);
    mutex_.unlock();
  } else {
    OutRequest d_oreq((timeout > 0 ? kRudpType : kUdpType), NextSequence());
    d_oreq.writeBinary(oreq.buffer().get(), size);
    mutex_.lock();
    small_packets_.push_back(SmallPacket{ d_oreq, to, end_time });
    mutex_.unlock();
  }
  // post an event
  events_mutex_.lock();
  events_.push_back(Event(Event::kSendNextPacket));
  events_mutex_.unlock();
  events_cv_.notify_all();
  return false;
}

void Dgram::KeepAlive(const UdpEndpoint& to, bool alive) {
  alive_ = alive;
  alive_endpoint_ = to;
  if (alive) {
    last_alive_time_ = std::chrono::system_clock::now()
        + std::chrono::seconds(5);
  }
}

void Dgram::ClearPackets(const UdpEndpoint& to) {
  bool exists = false;
  mutex_.lock();
  for (auto it = small_packets_.begin(); it != small_packets_.end();) {
    if ((*it).to == to) {
      it = small_packets_.erase(it);
    } else {
      it++;
    }
  }
  if (!active_group_send_tasks_.empty()) {
    TimePoint end_time = SystemClock::now() + std::chrono::milliseconds(500);
    for (auto it = active_group_send_tasks_.begin();
         it != active_group_send_tasks_.end();) {
      if ((*it)->to() == to) {
        OutRequest oreq(kRudpType, NextSequence());
        oreq.writeInt16((*it)->id());
        oreq.writeInt8(kGroupCancelType);
        oreq.writeLength(oreq.size());
        small_packets_.push_back(SmallPacket{ oreq, (*it)->to(), end_time });
        delete *it;
        it = active_group_send_tasks_.erase(it);
        exists = true;
      } else {
        it++;
      }
    }
  }
  for (auto it = inactive_group_send_tasks_.begin();
       it != inactive_group_send_tasks_.end();) {
    if ((*it)->to() == to) {
      delete *it;
      it = inactive_group_send_tasks_.erase(it);
    } else {
      it++;
    }
  }
  mutex_.unlock();
  if (exists) {
    socket_.get_io_service().post(boost::bind(&Dgram::SendNextPacket, this));
  }
}

void Dgram::ClearPackets() {
  TimePoint end_time = SystemClock::now() + std::chrono::milliseconds(500);
  mutex_.lock();
  small_packets_.clear();
  pending_packets_.clear();
  if (!active_group_send_tasks_.empty()) {
    for (auto it = active_group_send_tasks_.begin(),
         end = active_group_send_tasks_.end(); it != end; ++it) {
      OutRequest oreq(kRudpType, NextSequence());
      oreq.writeInt16((*it)->id());
      oreq.writeInt8(kGroupCancelType);
      oreq.writeLength(oreq.size());
      small_packets_.push_back(SmallPacket{ oreq, (*it)->to(), end_time });
      delete *it;
    }
  }
  while (!inactive_group_send_tasks_.empty()) {
    delete inactive_group_send_tasks_.front();
    inactive_group_send_tasks_.pop_front();
  }
  mutex_.unlock();
  socket_.get_io_service().post(boost::bind(&Dgram::SendNextPacket, this));
}

bool Dgram::DestIsReceiving(const UdpEndpoint& dest) const {
  bool receiving = false;
  for (auto it = pending_packets_.begin(), end = pending_packets_.end();
       it != end; ++it) {
    if (dest == it->second.to) {
      receiving = true;
      break;
    }
  }
  if (!receiving) {
    for (auto it = active_group_send_tasks_.begin(),
         end = active_group_send_tasks_.end(); it != end; ++it) {
      if (dest == (*it)->to()) {
        receiving = true;
        break;
      }
    }
  }
  return receiving;
}

GroupReceiveTask* Dgram::GetReceiveTask(uint16_t id, const UdpEndpoint& from) const {
  GroupReceiveTask* t = nullptr;
  for (auto it = active_group_receive_tasks_.begin(),
       end = active_group_receive_tasks_.end(); it != end; ++it) {
    if ((*it)->CompareId(id, from)) {
      t = (*it);
      break;
    }
  }
  return t;
}

void Dgram::AsyncReceiveFrom() {
  buffer_ = SingletonBufferPool::getSingleton().Malloc(kUdpMaxPacketSize);
  socket_.async_receive_from(asio::buffer(buffer_.get(), kUdpMaxPacketSize),
                             sender_endpoint_,
                             boost::bind(&Dgram::HandleReceiveFrom, this,
                                         asio::placeholders::error,
                                         asio::placeholders::bytes_transferred));
}

void Dgram::AsyncSendTo(OutRequest& oreq, const UdpEndpoint& to, uint32_t timeout) {
  if (!closing_) {
    uint16_t size = oreq.size();
    oreq.writeLength(size);
    socket_.async_send_to(asio::buffer(oreq.buffer().get(), oreq.size()), to,
                          boost::bind(&Dgram::HandleSendTo, this,
                                      oreq.buffer(), asio::placeholders::error,
                                      asio::placeholders::bytes_transferred));
    // rudp
    if (timeout > 0) {
      TimePoint end_time = now_ + std::chrono::milliseconds(timeout);
      TimePoint next_time = now_ + std::chrono::milliseconds(kUdpTryoutTimeout);
      pending_packets_.insert(
            std::make_pair(oreq.sequence(),
                           PendingPacket{ oreq, to, end_time, next_time }));
    }
    available_send_buffer_size_ -= size;
  }
}

void Dgram::HandleReceiveFrom(const boost::system::error_code& error,
                              size_t bytes_transferred) {
  if (!error && bytes_transferred > 0) {
    events_mutex_.lock();
    events_.push_back(Event{Event::kReceiveComplete, sender_endpoint_,
                            buffer_, bytes_transferred});
    events_mutex_.unlock();
    events_cv_.notify_all();
    AsyncReceiveFrom();
  }
}

void Dgram::HandleSendTo(const SharedBuffer& /*buffer*/,
                         const boost::system::error_code& error,
                         size_t bytes_transferred) {
  if (!closing_ && !error) {
    events_mutex_.lock();
    events_.push_back(Event(Event::kSendNextPacket));
    available_send_buffer_size_ += bytes_transferred;
    events_mutex_.unlock();
    events_cv_.notify_all();
  }
}

void Dgram::HandleClose() {
  if (!open_)
    return;

  closing_ = true;

  mutex_.lock();

  auto& small_packets = small_packets_;
  auto& inactive_group_send_tasks = inactive_group_send_tasks_;

  // small packets
  while (!small_packets.empty()) {
    small_packets.pop_front();
  }

  // pending packets
  pending_packets_.clear();

  // group packets
  while (!inactive_group_send_tasks.empty()) {
    delete inactive_group_send_tasks.front();
    inactive_group_send_tasks.pop_front();
  }

  // active group send tasks
  for (auto it = active_group_send_tasks_.begin(),
       end = active_group_send_tasks_.end(); it != end; ++it) {
    delete *it;
  }
  active_group_send_tasks_.clear();

  // active group receive tasks
  for (auto it = active_group_receive_tasks_.begin(),
       end = active_group_receive_tasks_.end(); it != end; ++it) {
    delete *it;
  }
  active_group_receive_tasks_.clear();
  mutex_.unlock();

  boost::system::error_code ec;
  socket_.shutdown(asio::ip::udp::socket::shutdown_both, ec);
  socket_.close();
  open_ = false;
}

void Dgram::SendPacketAck(uint32_t seq, const UdpEndpoint& to) {
  OutRequest oreq(kAckType, 0);
  oreq.writeInt32(seq);
  oreq.writeLength(oreq.size());
  AsyncSendTo(oreq, to, 0);
}

bool Dgram::SendNextPacket() {
  bool has_packets = true;
  const TimePoint& now = now_;
  std::mutex& mutex = mutex_;
  auto& small_packets = small_packets_;
  auto& inactive_group_send_tasks = inactive_group_send_tasks_;
  auto& group_sending_tasks = active_group_send_tasks_;

  // send small packets
  while (1) {
    mutex.lock();
    if (small_packets.empty()) {
      has_packets = false;
      mutex.unlock();
      break;
    } else {
      SmallPacket& sp = small_packets.front();
      if (sp.end_time > now) {
        if (!DestIsReceiving(sp.to)) {
          AsyncSendTo(sp.oreq, sp.to, 0);
          // udp
          if (sp.oreq.command() == kRudpType) {
            TimePoint next_time = now + std::chrono::milliseconds(kUdpTryoutTimeout);
            pending_packets_.insert(
                  std::make_pair(sp.oreq.sequence(),
                                 PendingPacket{ sp.oreq, sp.to, sp.end_time, next_time }));
          }
          small_packets.pop_front();
        }
        has_packets = !small_packets.empty();
        mutex.unlock();
        break;
      } else {
        SmallPacket sp2 = sp;
        small_packets.pop_front();
        has_packets = !small_packets.empty();
        mutex.unlock();
        timeout_signal(OutRequest(sp2.oreq.buffer(), sp2.oreq.size(), kUdpHeaderLength), sp2.to);
      }
    }
  }
  // select a new group_send_task from the send queue
  while (1) {
    mutex.lock();
    if (group_sending_tasks.size() < 5 && inactive_group_send_tasks.size() > 0) {
      GroupSendTask* t = inactive_group_send_tasks.front();
      if (t->end_time() > now) {
        if (!DestIsReceiving(t->to())) {
          t->HandleSendComplete(now);
          group_sending_tasks.push_back(t);
          inactive_group_send_tasks.pop_front();
          has_packets |= !inactive_group_send_tasks.empty();
        }
        mutex.unlock();
        break;
      } else {
        inactive_group_send_tasks.pop_front();
        has_packets |= !inactive_group_send_tasks.empty();
        mutex.unlock();
        timeout_signal(OutRequest(t->buffer(), t->size()), t->to());
        delete t;
      }
    } else {
      mutex.unlock();
      break;
    }
  }
  return has_packets;
}

void Dgram::HandleReceiveComplete(const SharedBuffer& buffer, size_t bytes_recvd,
                                  const UdpEndpoint& from) {
  InRequest ireq(buffer, static_cast<uint16_t>(bytes_recvd), 0);
  uint16_t len = ireq.readLength();
  if (len != bytes_recvd)
    return;

  uint32_t type = ireq.readCommand();
  uint32_t seq = ireq.readSequence();
  if (type == kRudpType) {
    SendPacketAck(seq, from);
  } else if (type == kAckType) {
    HandlePacketAck(ireq);
    return;
  } else if (type == kKeepAliveType) {
    keep_alive_signal(from);
    return;
  }
  if (packet_record_.Passed(seq, from)) {
    switch (type) {
    case kRudpType:
    {
      InRequest ir(buffer, (uint16_t)bytes_recvd, kUdpHeaderLength);
      ir.setFrom(from);
      arrival_signal(ir, from, 10);
      break;
    }
    case kOutGroupType:
    {
      handleReceiveGroupPacket(ireq, from);
      break;
    }
    case kInGroupType:
    {
      handleReceiveGroupPacketAck(ireq, from);
      break;
    }
    case kUdpType:
    {
      InRequest ir(buffer, (uint16_t)bytes_recvd, kUdpHeaderLength);
      ir.setFrom(from);
      arrival_signal(ir, from, 0);
      break;
    }
    default:
      break;
    }
  }
}

void Dgram::HandlePacketAck(InRequest& ireq) {
  uint32_t seq = ireq.readInt32();
  std::map<uint32_t, PendingPacket>::iterator it = pending_packets_.find(seq);
  if (it != pending_packets_.end()) {
    sent_signal(OutRequest((*it).second.oreq.buffer(), kUdpHeaderLength), (*it).second.to, 0);
    pending_packets_.erase(it);
  }
}

void Dgram::handleReceiveGroupPacket(InRequest& ireq, const UdpEndpoint& from) {
  uint16_t id = ireq.readInt16();
  GroupReceiveTask* grt = GetReceiveTask(id, from);
  if (!grt) {
    uint16_t cur_pos = ireq.left();
    ireq.skip(sizeof(uint8_t), kSkipCurrent);
    uint16_t count = ireq.readInt16();
    ireq.skip(cur_pos, kSkipEnd);
    if (!group_record_.Exists(id, count, from)) {
      OutRequest oreq(kInGroupType, NextSequence());
      oreq.writeInt16(id);
      oreq.writeInt8(kGroupReportType);
      oreq.writeInt16(count);
      AsyncSendTo(oreq, from, 0);
    } else {
      // not found, it's a possible new packet
      grt = new GroupReceiveTask(this, id, from);
      if (grt) {
        active_group_receive_tasks_.push_back(grt);
      }
    }
  }
  if (grt) {
    grt->HandleReceivePacket(ireq, from, now_);
    if (grt->IsCompleted() || grt->IsFailed()) {
      for (auto it = active_group_receive_tasks_.begin(), end = active_group_receive_tasks_.end();
           it != end; ++it) {
        if ((*it)->CompareId(id, from)) {
          group_record_.Append(id, grt->count(), from);
          if (grt->IsCompleted()) {
            InRequest ireq(grt->data(), grt->size());
            ireq.setFrom(from);
            arrival_signal(ireq, from, (int32_t)grt->Cost());
          }
          active_group_receive_tasks_.erase(it);
          break;
        }
      }
      delete grt;
    }
  }
}

void Dgram::handleReceiveGroupPacketAck(InRequest& ireq, const UdpEndpoint& from) {
  uint16_t id = ireq.readInt16();
  for (auto it = active_group_send_tasks_.begin(), end = active_group_send_tasks_.end();
       it != end; ++it) {
    GroupSendTask* p = *it;
    if (p->id() == id) {
      p->HandleReceivePacket(ireq, from);
      if (p->IsCompleted() || p->IsFailed()) {
        if (p->IsCompleted())
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

void Dgram::Tick() {
  const TimePoint& now = now_;
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
          AsyncSendTo(pp.oreq, pp.to, 0);
          pp.next_time = now + std::chrono::milliseconds(kUdpTryoutTimeout);
        }
      } else {
        PendingPacket pp2 = pp;
        pending_packets.erase(it);
        mutex.unlock();
        unlocked = true;
        timeout_signal(OutRequest(pp2.oreq.buffer(), pp2.oreq.size(),
                                  kUdpHeaderLength), pp2.to);
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
      (*it)->HandleTimeout(now);
      if ((*it)->IsFailed()) {
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
  for (auto it = active_group_receive_tasks_.begin(),
       end = active_group_receive_tasks_.end(); it != end; ++it) {
    (*it)->HandleTimeout(now);
    if ((*it)->IsFailed()) {
      delete *it;
      active_group_receive_tasks_.erase(it);
      break;
    }
  }
  // alive
  if (alive_ && last_alive_time_ < now) {
    OutRequest oreq(kKeepAliveType, 0);
    AsyncSendTo(oreq, alive_endpoint_, 0);
    last_alive_time_ = now + std::chrono::seconds(5);
  }
  // tick signal
  tick_signal();
}

bool Dgram::ProcessEvents() {
  bool has_events;
  bool can_send;
  Event event(0);
  // get one event
  events_mutex_.lock();
  can_send = (available_send_buffer_size_ > 0);
  has_events = !events_.empty();
  if (has_events) {
    event = *events_.begin();
    events_.erase(events_.begin());
    has_events = !events_.empty();
  }
  events_mutex_.unlock();
  // process this event
  if (event.type == Event::kReceiveComplete) {
    HandleReceiveComplete(event.buffer, event.size, event.endpoint);
  }
  if (can_send) {
    for (auto it = active_group_send_tasks_.begin(),
         end = active_group_send_tasks_.end(); it != end; ++it) {
      (*it)->HandleSendComplete(now_);
      if ((*it)->IsSending()) {
        has_events = true;
      }
    }
    has_events |= SendNextPacket();
  }
  return has_events;
}

void Dgram::MainLoop() {
  bool& running = running_;
  TimePoint& now = now_;
  // async receivce
  AsyncReceiveFrom();
  // loop
  bool busy;
  while (running) {
    now = SystemClock::now();
    busy = ProcessEvents();
    Tick();
    if (!busy) {
      std::unique_lock<std::mutex> lock(events_mutex_);
      events_cv_.wait_for(lock, std::chrono::milliseconds(5), [this]{
        bool has_events = !events_.empty();
        if (!has_events && available_send_buffer_size_ > 0) {
          for (auto it = active_group_send_tasks_.begin(),
               end = active_group_send_tasks_.end(); it != end; ++it) {
            if ((*it)->IsSending()) {
              has_events = true;
            }
          }
        }
        return has_events;
      });
    }
  }
}

uint32_t Dgram::NextSequence() {
  static uint32_t seq = (uint32_t)time(0);
  ++seq;
  seq = seq % 0xFFFFFFFF;
  return seq;
}
} // udp
} // network
} // mtl
