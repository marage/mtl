#ifndef MTL_RC4_HPP
#define MTL_RC4_HPP
#include "mtl/mtl.hpp"

namespace mtl {

struct MTL_EXPORT RC4
{
    static void init(unsigned char s[256], const unsigned char* key, int len);
    static void encrypt(unsigned char s[256], unsigned char* data, int len);
};

} // namespace mtl

#endif // MTL_RC4_HPP
