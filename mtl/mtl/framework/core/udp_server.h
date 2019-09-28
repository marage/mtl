#ifndef MTL_FRAMEWORK_CORE_UDP_SERVER_H
#define MTL_FRAMEWORK_CORE_UDP_SERVER_H
#include <boost/thread.hpp>
#include <list>
#include "mtl/network/udp/dgram.hpp"
#include "framework.h"

namespace mtl {
namespace framework {
namespace core {

template <typename UnitFactory>
class UDPServer : private boost::noncopyable {
public:
  UDPServer(boost::asio::io_service& io_service, UnitFactory* unit_factory)
      : unit_factory_(unit_factory)
      , dgram_(new network::udp::Dgram(io_service)) {
    dgram_->arrival_signal.connect(
          boost::bind(&UDPServer::HandleArrival, this, _1, _2, _3));
  }

  ~UDPServer() {}

  bool Open(const std::string& host, uint16_t port, size_t work_count = 1);
  void Close();

  inline void SendTo(const OutRequest& oreq,
                     const AsioUDPEndpoint& to,
                     int64_t timeout = 5000) {
    dgram_->SendTo(oreq, to, timeout);
  }

protected:
  inline network::udp::Dgram& dgram() { return *dgram_; }

private:
  void HandleArrival(InRequest& ireq,
                     const AsioUDPEndpoint& /*from*/,
                     int32_t /*msecs*/) {
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
  network::udp::DgramPtr dgram_;
  std::vector<boost::thread*> threads_;
  typename std::vector<UnitPtr> works_;
};

template <typename UnitFactory>
bool UDPServer<UnitFactory>::Open(const std::string& host, uint16_t port,
                                  size_t work_count) {
  static uint32_t id = 0;

  boost::asio::ip::address addr = boost::asio::ip::address::from_string(host);
  if (!dgram_->open(AsioUDPEndpoint(addr, port)))
    return false;

  // create work threads
  threads_.reserve(work_count);
  works_.reserve(work_count);
  for (size_t i = 0; i < work_count; ++i) {
    UnitPtr w = unit_factory_->CreateUnit(++id, dgram_, Unit::kLoop);
    works_.push_back(w);
    boost::thread* t = new boost::thread(&Unit::Process, w);
    threads_.push_back(t);
  }

  return true;
}

template <typename UnitFactory>
void UDPServer<UnitFactory>::Close() {
  // close the connection
  dgram_->close();

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
}

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_UDP_SERVER_H
