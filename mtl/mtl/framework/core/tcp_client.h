#ifndef MTL_FRAMEWORK_CORE_TCP_CLIENT_H
#define MTL_FRAMEWORK_CORE_TCP_CLIENT_H
#include <boost/thread.hpp>
#include "mtl/network/tcp/client.hpp"
#include "mtl/framework/log/log.h"
#include "framework.h"
#include "heart_beat_task.h"
#include "connect_task.h"

namespace mtl {
namespace framework {
namespace core {

template <typename UnitFactory>
class TCPClient : private boost::noncopyable {
public:
  TCPClient(boost::asio::io_service& io_service, UnitFactory* unit_factory)
    : unit_factory_(unit_factory)
    , client_(new network::tcp::Client(io_service)), work_count_(0) {
    client_->new_connection_signal.connect(
          boost::bind(&TCPClient::HandleNewConnection, this, _1));
    client_->packet_arrival_signal.connect(
          boost::bind(&TCPClient::HandleArrival, this, _1));
  }
  ~TCPClient() {}

  bool Open(const std::string& host, const std::string& service,
            size_t work_count = 1) {
    host_ = host;
    service_ = service;
    work_count_ = work_count;
    return client_->Open(host, service);
  }
  void Close();

protected:
  inline network::tcp::Client& client() { return *client_; }

private:
  void HandleNewConnection(const boost::system::error_code& ec);
  void HandleArrival(InRequest& ireq) {
    size_t idx = 0;
    size_t min = 0xffffffff;
    for (size_t i = 0; i < works_.size(); ++i) {
      if (works_.at(i)->request_count() < min) {
        min = works_.at(i)->request_count();
        idx = i;
      }
    }
    works_[idx]->PushRequest(ireq);
  }

  typedef typename UnitFactory::Unit Unit;
  typedef typename UnitFactory::UnitPtr UnitPtr;

  UnitFactory* unit_factory_;
  network::tcp::ClientPtr client_;
  std::string host_;
  std::string service_;
  size_t work_count_;
  std::vector<boost::thread*> threads_;
  typename std::vector<UnitPtr> works_;
};

template <typename UnitFactory>
void TCPClient<UnitFactory>::Close() {
  // stop works
  for (typename std::vector<UnitPtr>::iterator it = works_.begin(),
       end = works_.end(); it != end; ++it) {
    (*it)->Quit();
  }
  // stop and delete threads
  for (typename std::vector<boost::thread*>::iterator it = threads_.begin(),
       end = threads_.end(); it != end; ++it) {
    if ((*it)->joinable())
      (*it)->join();
    delete *it;
  }
  threads_.clear();
  // delete works
  works_.clear();
  // close the connection
  client_->Close();
}

template <typename UnitFactory>
void TCPClient<UnitFactory>::HandleNewConnection(
    const boost::system::error_code& ec) {
  if (!ec) {
    if (threads_.empty()) {
      // create work threads
      threads_.reserve(work_count_);
      works_.reserve(work_count_);
      for (size_t i = 0; i < work_count_; ++i) {
        UnitPtr w = unit_factory_->CreateUnit(0, client_, Unit::kLoop);
        if (works_.empty()) {
          // connect task
          TaskPtr t1(new ConnectTask(client_, 10000));
          w->AppendTask(t1);
          // heart-beat task
          TaskPtr t2(new HeartBeatTask(client_, 5000));
          w->AppendTask(t2);
        }
        w->OnConnected();
        works_.push_back(w);
        boost::thread* t = new boost::thread(&Unit::Process, w);
        threads_.push_back(t);
      }
    } else {
      for (auto& w : works_) {
        w->OnConnected();
      }
    }
    // log
    auto lg = log::global_logger::get();
    BOOST_LOG_FUNCTION()
    BOOST_LOG_SEV(lg, log::kWarning) << work_count_ << "connected.";
  } else {
    // log
    auto lg = log::global_logger::get();
    BOOST_LOG_FUNCTION()
    BOOST_LOG_SEV(lg, log::kWarning) << ec.message() << ec.value();
  }
}

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_TCP_CLIENT_H
