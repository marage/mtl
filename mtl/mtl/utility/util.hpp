#ifndef MTL_UTILITY_UTIL_H
#define MTL_UTILITY_UTIL_H
#include <string>
#include "mtl/mtl.hpp"

namespace mtl {
namespace util {

MTL_EXPORT unsigned char* encrypt(unsigned char* data, int data_len,
                                   const unsigned char* key, int key_len);

MTL_EXPORT int evaluateScore(const std::string& ip1, const std::string& ip2);

//MTL_EXPORT std::wstring ansiToUnicode(const std::string& src);
//MTL_EXPORT std::string unicodeToAnsi(const std::wstring& src);

//MTL_EXPORT std::wstring utf8ToUnicode(const std::string& src);
//MTL_EXPORT std::string unicodeToUtf8(const std::wstring& src);

} // util
} // namespace mtl

#endif // MTL_UTILITY_UTIL_H
