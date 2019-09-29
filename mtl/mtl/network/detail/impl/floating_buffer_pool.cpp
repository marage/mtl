#include "mtl/network/detail/floating_buffer_pool.hpp"
#include <malloc.h>
#include <assert.h>

namespace mtl {
namespace network {

FloatingBufferPool::FloatingBufferPool() : node_list_(nullptr)
{
}

FloatingBufferPool::~FloatingBufferPool()
{
    releaseMemory();
}

void* FloatingBufferPool::malloc(std::size_t n)
{
    constexpr std::size_t kBlockSize = 16;
    char* p = nullptr;

    std::size_t m = n % kBlockSize;
    if (m > 0) {
        n += kBlockSize - m;
    }

    p = static_cast<char*>(getBuffer(n));
    if (!p) {
        p = static_cast<char*>(malloc(n + sizeof(void*)));
        assert( p );
        void* tmp = p + sizeof(void*);
        valid_ptr_list_[tmp] = tmp;
    }

    *(reinterpret_cast<uint32_t*>(p)) = static_cast<uint32_t>(n);
    p += sizeof(void*);

    return p;
}

void FloatingBufferPool::free(void* p)
{
    char* rp = static_cast<char*>(p) - sizeof(void*);
    uint32_t size = *(reinterpret_cast<uint32_t*>(rp));
    Node* last = reinterpret_cast<Node*>(rp);
    last->next = nullptr;
    SizeNode* sn = getNode(size);
    if (sn->tail) {
        sn->tail->next = last;
    } else {
        sn->head = last;
        sn->tail = last;
    }
}

void FloatingBufferPool::releaseMemory()
{
    SizeNode* p = node_list_;
    SizeNode* k;
    while (p) {
        if (p->head) {
            Node* q = p->head;
            Node* t;
            while (q) {
                t = q->next;
                ::free(q);
                q = t;
            }
        }
        k = p->next;
        ::free(p);
        p = k;
    }
    node_list_ = nullptr;
}

void* FloatingBufferPool::getBuffer(std::size_t size)
{
    void* p = nullptr;
    SizeNode* sn = getNode(size);
    if (sn && sn->head) {
        Node* t = reinterpret_cast<Node*>(sn->head);
        if (t == sn->tail) {
            sn->tail = nullptr;
        }
        sn->head = t->next;
        p = t;
    }
    return p;
}

FloatingBufferPool::SizeNode* FloatingBufferPool::getNode(std::size_t size)
{
    SizeNode* p = node_list_;
    SizeNode* pre = p;
    while (p) {
        if (p->size == static_cast<uint32_t>(size))
            break;
        pre = p;
        p = p->next;
    }
    if (!p) {
        p = static_cast<SizeNode*>(::malloc(sizeof(SizeNode)));
        p->size = static_cast<uint32_t>(size);
        p->head = nullptr;
        p->tail = nullptr;
        p->next = nullptr;
        if (pre) {
            pre->next = p;
        } else {
            node_list_ = p;
        }
    }
    return p;
}

} // namespace network
} // namespace mtl
