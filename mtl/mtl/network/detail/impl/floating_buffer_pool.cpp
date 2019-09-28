#include "mtl/network/detail/floating_buffer_pool.hpp"
#include <malloc.h>
#include <assert.h>

namespace mtl {
namespace network {

FloatingBufferPool::FloatingBufferPool() : node_list_(0) {
}

FloatingBufferPool::~FloatingBufferPool() {
  ReleaseMemory();
}

void* FloatingBufferPool::Malloc(std::size_t n) {
  const std::size_t BLOCK_SIZE = 16;
  char* p = 0;

  std::size_t m = n % BLOCK_SIZE;
  if (m > 0) {
    n += BLOCK_SIZE - m;
  }

  p = (char*) GetBuffer(n);
  if (!p) {
    p = (char*) ::malloc(n + sizeof(void*));
    assert( p );
    void* tmp = p + sizeof(void*);
    valid_ptr_list_[tmp] = tmp;
  }

  *((uint32_t*) p) = static_cast<uint32_t>(n);
  p += sizeof(void*);

  return p;
}

void FloatingBufferPool::Free(void* p) {
  char* rp = (char*) p - sizeof(void*);
  uint32_t size = *((uint32_t*) rp);
  Node* last = (Node*) rp;
  last->next = 0;
  SizeNode* sn = GetNode(size);
  if (sn->tail) {
    sn->tail->next = last;
  } else {
    sn->head = last;
    sn->tail = last;
  }
}

void FloatingBufferPool::ReleaseMemory() {
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
  node_list_ = 0;
}

void* FloatingBufferPool::GetBuffer(std::size_t size) {
  void* p = 0;
  SizeNode* sn = GetNode(size);
  if (sn && sn->head) {
    Node* t = (Node*)sn->head;
    if (t == sn->tail) {
      sn->tail = 0;
    }
    sn->head = t->next;
    p = t;
  }
  return p;
}

FloatingBufferPool::SizeNode* FloatingBufferPool::GetNode(std::size_t size) {
  SizeNode* p = node_list_;
  SizeNode* pre = p;
  while (p) {
    if (p->size == static_cast<uint32_t>(size)) {
      break;
    }
    pre = p;
    p = p->next;
  }
  if (!p) {
    p = (SizeNode*) ::malloc(sizeof(SizeNode));
    p->size = static_cast<uint32_t>(size);
    p->head = 0;
    p->tail = 0;
    p->next = 0;
    if (pre) {
      pre->next = p;
    } else {
      node_list_ = p;
    }
  }
  return p;
}
}
}
