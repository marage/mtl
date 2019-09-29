#ifndef MTL_NETWORK_SINGLATON_BUFFER_POOL_HPP
#define MTL_NETWORK_SINGLATON_BUFFER_POOL_HPP
#include <boost/pool/poolfwd.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/noncopyable.hpp>
#include "mtl/mtl.hpp"
#include "mtl/singleton/singleton.hpp"
#include "./detail/floating_buffer_pool.hpp"
#include "shared_buffer.hpp"

namespace mtl {
namespace network {

class MTL_EXPORT SingletonBufferPool : public Singleton<SingletonBufferPool>
{
public:
    SingletonBufferPool();
    ~SingletonBufferPool();

    static SingletonBufferPool& getSingleton();
    static SingletonBufferPool* getSingletonPtr();

    bool isFrom(void* p);
    SharedBuffer allocBuffer(unsigned int size);
    void freeBuffer(void* p);

    void releaseMemory();

private:
    boost::pool<> fixed_pool_;
    FloatingBufferPool floating_pool_;
    boost::mutex mutex_;
};

} // network
} // mtl

#endif // MTL_NETWORK_SINGLATON_BUFFER_POOL_HPP
