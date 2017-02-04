#ifndef MTL_NETWORK_PACKET_FIELD_HPP
#define MTL_NETWORK_PACKET_FIELD_HPP
#include <boost/cstdint.hpp>

namespace mtl {
namespace network {

struct PacketField
{
    enum ValueType
    {
        FVT_UCHAR,
        FVT_USHORT,
        FVT_UINT,
        FVT_ULONG,
        FVT_UINT64,
        FVT_STRING,
        FVT_BINARY,
    };

    boost::int8_t type;
    boost::int8_t vt;
    union
    {
        boost::int8_t uc_val;
        boost::int16_t ui_val;
        int32_t un_val;
        int64_t ug_val;
        struct
        {
            char* data;
            short size;
        } str_val;
    } value;
};

} // network
} // mtl

#endif // MTL_NETWORK_PACKET_FIELD_HPP
