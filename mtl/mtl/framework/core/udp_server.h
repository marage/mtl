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
class UdpServer : private boost::noncopyable
{
public:
    UdpServer(boost::asio::io_context& context, UnitFactory* unit_factory)
        : unit_factory_(unit_factory)
        , dgram_(new network::udp::Dgram(context))
    {
        dgram_->arrival_signal.connect(
                    boost::bind(&UdpServer::handleArrival, this, _1, _2, _3));
    }

    ~UdpServer() {}

    bool open(const std::string& host, uint16_t port, size_t work_count = 1);
    void close();

    void sendTo(const OutRequest& oreq, const UdpEndPoint& to,
                int64_t timeout = 5000) {
        dgram_->sendTo(oreq, to, timeout);
    }

protected:
    network::udp::Dgram& dgram() { return *dgram_; }

private:
    void handleArrival(network::InRequest& ireq, const UdpEndPoint& /*from*/,
                       int32_t /*msecs*/) {
        size_t idx = 0;
        size_t min = 0xffffffff;
        for (size_t i = 0; i < works_.size(); ++i) {
            if (works_.at(i)->requestCount() < min) {
                min = works_.at(i)->requestCount();
                idx = i;
            }
        }
        works_[idx]->pushRequest(ireq);
    }

    typedef typename UnitFactory::Unit Unit;
    typedef typename UnitFactory::UnitPtr UnitPtr;

    UnitFactory* unit_factory_;
    network::udp::DgramPtr dgram_;
    std::vector<boost::thread*> threads_;
    typename std::vector<UnitPtr> works_;
};

template <typename UnitFactory>
bool UdpServer<UnitFactory>::open(const std::string& host, uint16_t port,
                                  size_t work_count)
{
    static uint32_t id = 0;

    boost::asio::ip::address addr = boost::asio::ip::address::from_string(host);
    if (!dgram_->open(UdpEndPoint(addr, port))) {
        return false;
    }

    // create work threads
    threads_.reserve(work_count);
    works_.reserve(work_count);
    for (size_t i = 0; i < work_count; ++i) {
        UnitPtr w = unit_factory_->createUnit(++id, dgram_, Unit::kLoop);
        works_.push_back(w);
        boost::thread* t = new boost::thread(&Unit::process, w);
        threads_.push_back(t);
    }

    return true;
}

template <typename UnitFactory>
void UdpServer<UnitFactory>::close()
{
    // close the connection
    dgram_->close();

    // stop works
    for (typename std::vector<UnitPtr>::iterator it = works_.begin(),
         end = works_.end(); it != end; ++it) {
        (*it)->quit();
    }
    // stop and delete threads
    for (typename std::vector<boost::thread*>::iterator it = threads_.begin(),
         end = threads_.end(); it != end; ++it) {
        if ((*it)->joinable()) {
            (*it)->join();
        }
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
