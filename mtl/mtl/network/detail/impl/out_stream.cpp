#include "mtl/network/out_stream.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/detail/byte_order.hpp"
#include "mtl/network/protocol.hpp"

namespace mtl {
namespace network {

OutStream::OutStream(uint16_t begin) {
  buffer_ = SingletonBufferPool::getSingleton().Malloc(1024);
  begin_ = begin;
  size_ = 1024;
  cursor_ = buffer_.get() + begin;
}

OutStream::OutStream(const SharedBuffer& buffer, uint16_t size,
                     uint16_t begin)
  : buffer_(buffer), size_(size), begin_(begin) {
  cursor_ = buffer_.get() + begin;
}

uint16_t OutStream::Skip(uint16_t offset, int origin) {
  uint16_t old = Size();
  if (origin == kSkipBegin) {
    uint16_t data_size = size_ - begin_;
    if (offset > data_size)
      offset = data_size;
    cursor_ = (char*)buffer_.get() + begin_ + offset;
  } else if (origin == kSkipCurrent) {
    if (offset > Left())
      offset = Left();
    cursor_ += offset;
  } else {
    uint16_t data_size = size_ - begin_;
    if (offset > data_size)
      offset = data_size;
    cursor_ = buffer_.get() + size_ - offset;
  }
  return old;
}

void OutStream::Resize(uint16_t sz) {
  if (sz > size_) {
    uint16_t new_size = sz + 1024 - sz % 1024;
    SharedBuffer buf = SingletonBufferPool::getSingleton().Malloc(new_size);
    memcpy(buf.get(), buffer_.get(), size_);
    cursor_ = buf.get() + Size();
    buffer_ = buf;
    size_ = new_size;
  }
}

void OutStream::Clear() {
  buffer_.reset();
  size_ = 0;
  cursor_ = nullptr;
}

OutStream& OutStream::WriteInt8(uint8_t val) {
  WriteBinary(reinterpret_cast<const char*>(&val), 1);
  return (*this);
}

OutStream& OutStream::WriteInt16(uint16_t val) {
  uint16_t v = HostToNetwork16(val);
  WriteBinary(reinterpret_cast<const char*>(&v), 2);
  return (*this);
}

OutStream& OutStream::WriteInt32(uint32_t val) {
  uint32_t v = HostToNetwork32(val);
  WriteBinary(reinterpret_cast<const char*>(&v), 4);
  return (*this);
}

OutStream& OutStream::WriteInt64(uint64_t val) {
  uint64_t v = HostToNetwork64(val);
  WriteBinary(reinterpret_cast<const char*>(&v), 8);
  return (*this);
}

OutStream& OutStream::WriteString(const std::string& val) {
  uint16_t len = static_cast<uint16_t>(val.length());
  WriteInt16((uint16_t) len);
  WriteBinary(val.c_str(), len);
  return (*this);
}

OutStream& OutStream::WriteBinary(const char* data, uint16_t data_size) {
  if (Left() < data_size) {
    Resize(size_ + data_size);
  }
  memcpy(cursor_, data, data_size);
  cursor_ += data_size;
  return (*this);
}
}
}
