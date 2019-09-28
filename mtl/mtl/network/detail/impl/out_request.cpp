#include "mtl/network/out_request.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/detail/byte_order.hpp"

namespace mtl {
namespace network {

OutRequest::OutRequest(uint32_t cmd, uint32_t seq, uint16_t begin)
  : OutStream(begin) {
  WriteVersion(kVersion);
  WriteCommand(cmd);
  WriteSequence(seq);
  WriteLength(kHeaderLength);
  Skip(kHeaderLength, kSkipCurrent);
}

OutRequest::OutRequest(const SharedBuffer& buffer, uint16_t size, uint16_t begin)
  : OutStream(buffer, size, begin) {
  Skip(kHeaderLength, kSkipCurrent);
}

uint16_t OutRequest::Length() const {
  uint16_t v;
  char* p = (char*) buffer().get() + begin();
  memcpy(&v, p, sizeof(uint16_t));
  return NetworkToHost16(v);
}

uint8_t OutRequest::Version() const {
  char* p = (char*) buffer().get() + begin() + kLenFieldLength;
  return *((uint8_t*)p);
}

uint32_t OutRequest::Command() const {
  uint32_t v;
  char* p = (char*) buffer().get() + begin() + kLenFieldLength + kVerFieldLength;
  memcpy(&v, p, sizeof(uint32_t));
  return NetworkToHost32(v);
}

uint8_t OutRequest::Option() const {
  char* p = (char*) buffer().get() + begin() + kLenFieldLength + kVerFieldLength
      + kCommandFieldLength;
  return *((uint8_t*)p);
}

uint32_t OutRequest::Sequence() const {
  uint32_t v;
  char* p = (char*)buffer().get() + begin() + kLenFieldLength + kVerFieldLength
      + kCommandFieldLength + kOptionFieldLength;
  memcpy(&v, p, sizeof(uint32_t));
  return NetworkToHost32(v);
}

const char* OutRequest::Data() const {
  return (char*)buffer().get() + begin() + kHeaderLength;
}

uint16_t OutRequest::DataSize() const {
  return (Size() - begin() - kHeaderLength);
}

OutRequest& OutRequest::WriteLength(uint16_t len) {
  char* p = (char*)buffer().get() + begin();
  uint16_t v = HostToNetwork16(len);
  memcpy(p, &v, sizeof(uint16_t));
  return (*this);
}

OutRequest& OutRequest::WriteVersion(uint8_t ver) {
  char* p = (char*)buffer().get() + begin() + kLenFieldLength;
  *((uint8_t*)p) = ver;
  return (*this);
}

OutRequest& OutRequest::WriteCommand(uint32_t cmd) {
  char* p = (char*)buffer().get() + begin() + kLenFieldLength + kVerFieldLength;
  uint32_t v = HostToNetwork32(cmd);
  memcpy(p, &v, sizeof(uint32_t));
  return (*this);
}

OutRequest& OutRequest::WriteOption(uint8_t option) {
  char* p = (char*)buffer().get() + begin() + kLenFieldLength + kVerFieldLength
      + kCommandFieldLength;
  *((uint8_t*)p) = option;
  return (*this);
}

OutRequest& OutRequest::WriteSequence(uint32_t seq) {
  char* p = (char*)buffer().get() + begin() + kLenFieldLength + kVerFieldLength
      + kCommandFieldLength + kOptionFieldLength;
  uint32_t v = HostToNetwork32(seq);
  memcpy(p, &v, sizeof(uint32_t));
  return (*this);
}
}
}
