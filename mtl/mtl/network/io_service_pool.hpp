//
// io_service_pool.hpp
// ~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MTL_NETWORK_IO_SERVICE_POOL_HPP
#define MTL_NETWORK_IO_SERVICE_POOL_HPP
#include <vector>
#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include "mtl/mtl.hpp"
#include "mtl/singleton/singleton.hpp"

namespace mtl {
namespace network {

/// A pool of io_service objects.
class _MTL_EXPORT IOServicePool
    : public Singleton<IOServicePool>, private boost::noncopyable
{
public:
    /// Construct the io_service pool.
    explicit IOServicePool(std::size_t pool_size);

    static IOServicePool& getSingleton();
    static IOServicePool* getSingletonPtr();

    /// Run all io_service objects in the pool.
    void run();

    /// Join all io_service objects in the pool.
    void joinAll();

    /// Stop all io_service objects in the pool.
    void stop();

    /// Get an io_service to use.
    boost::asio::io_service& getIOService();

private:
    typedef boost::shared_ptr<boost::asio::io_service> io_service_ptr;
    typedef boost::shared_ptr<boost::asio::io_service::work> work_ptr;

    /// The pool of io_services.
    std::vector<io_service_ptr> io_services_;

    /// The work that keeps the io_services running.
    std::vector<work_ptr> work_;

    /// A pool of threads to run all of the io_services.
    std::vector<boost::shared_ptr<boost::thread>> threads_;

    /// The next io_service to use for a connection.
    std::size_t next_io_service_;
};

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_IO_SERVICE_POOL_HPP
