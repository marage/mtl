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
class UDPServer : private boost::noncopyable
{
public:
    UDPServer(boost::asio::io_context& io_context, UnitFactory* unit_factory);
    ~UDPServer() {}

    bool open(const std::string& host, uint16_t port, int64_t frequency = 1000,
              size_t work_count = 1);
    void close();

    inline void sendTo(const OutRequest& oreq, const boost::asio::ip::udp::endpoint& to,
                       int64_t timeout = 5000)
    {
        dgram_->sendTo(oreq, to, timeout);
    }

protected:
    network::udp::Dgram& dgram() { return *dgram_; }

private:
    void handleArrival(InRequest& ireq, const boost::asio::ip::udp::endpoint& from,
                       int32_t msecs);

    typedef typename UnitFactory::Unit Unit;
    typedef typename UnitFactory::unit_ptr unit_ptr;

    UnitFactory* unit_factory_;
    network::udp::dgram_ptr dgram_;
    std::vector<boost::thread*> threads_;
    typename std::vector<unit_ptr> works_;
};

template <typename UnitFactory>
UDPServer<UnitFactory>::UDPServer(boost::asio::io_context& io_context, UnitFactory* unit_factory)
    : unit_factory_(unit_factory), dgram_(new network::udp::Dgram(io_context))
{
    dgram_->arrival_signal.connect(
                boost::bind(&UDPServer::handleArrival, this, _1, _2, _3));
}

template <typename UnitFactory>
bool UDPServer<UnitFactory>::open(const std::string& host, uint16_t port,
                                  int64_t frequency, size_t work_count)
{
    static uint32_t id = 0;

    boost::asio::ip::address addr = boost::asio::ip::address::from_string(host);
    if (!dgram_->open(UDPEndpoint(addr, port), frequency)) {
        return false;
    }

    // create work threads
    threads_.reserve(work_count);
    works_.reserve(work_count);
    for (size_t i = 0; i < work_count; ++i) {
        unit_ptr w = unit_factory_->createUnit(++id, dgram_, Unit::PROCESS_LOOP);
        works_.push_back(w);
        boost::thread* t = new boost::thread(&Unit::process, w);
        threads_.push_back(t);
    }

    return true;
}

template <typename UnitFactory>
void UDPServer<UnitFactory>::close()
{
    // close the connection
    dgram_->close();

    // stop works
    for (typename std::vector<unit_ptr>::iterator it = works_.begin(), end = works_.end();
         it != end; ++it) {
        (*it)->quit();
    }
    // stop and delete threads
    for (typename std::vector<boost::thread*>::iterator it = threads_.begin(), end = threads_.end();
         it != end; ++it) {
        if ((*it)->joinable())
            (*it)->join();
        delete *it;
    }
    threads_.clear();
    // delete works
    works_.clear();
}

template <typename UnitFactory>
void UDPServer<UnitFactory>::handleArrival(InRequest& ireq,
                                           const boost::asio::ip::udp::endpoint& /*from*/, int32_t /*msecs*/)
{
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

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_UDP_SERVER_H
