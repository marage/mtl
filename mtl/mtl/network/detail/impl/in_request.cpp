#include "mtl/network/in_request.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/detail/byte_order.hpp"

namespace mtl {
namespace network {

InRequest::InRequest()
    : handled_(false) {
}

InRequest::InRequest(const SharedBuffer& buffer, uint16_t size,
                     uint16_t begin)
    : InStream(buffer, size, begin), handled_(false)
{
    skip(kHeaderLength, kSkipCurrent);
}

uint16_t InRequest::readLength()
{
    uint16_t v;
    char* p = buffer().get() + begin();
    memcpy(&v, p, sizeof(uint16_t));
    return NetworkToHost16(v);
}

uint8_t InRequest::readVersion()
{
    char* p = buffer().get() + begin() + kLenFieldLength;
    return *(reinterpret_cast<uint8_t*>(p));
}

uint32_t InRequest::readCommand()
{
    uint32_t v;
    char* p = buffer().get() + begin() + kLenFieldLength + kVerFieldLength;
    memcpy(&v, p, sizeof(uint32_t));
    return NetworkToHost32(v);
}

uint8_t InRequest::readOption()
{
    char* p = buffer().get() + begin() + kLenFieldLength + kVerFieldLength
              + kCommandFieldLength;
    return *(reinterpret_cast<uint8_t*>(p));
}

uint32_t InRequest::readSequence()
{
    uint32_t v;
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength
              + kVerFieldLength + kCommandFieldLength + kOptionFieldLength;
    memcpy(&v, p, sizeof(uint32_t));
    return NetworkToHost32(v);
}
} // namespace network
} // namespace mtl
