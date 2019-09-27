#ifndef MTL_RC4_HPP
#define MTL_RC4_HPP
#include "mtl/mtl.hpp"

namespace mtl {
namespace rc4 {

MTL_EXPORT void init(unsigned char s[256], const unsigned char* key, int len);
MTL_EXPORT void encrypt(unsigned char s[256], unsigned char* data, int len);

} // namespace rc4
} // namespace mtl

#endif // MTL_RC4_HPP
