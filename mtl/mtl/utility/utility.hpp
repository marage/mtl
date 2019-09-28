#ifndef MTL_UTILITY_UTILITY_H
#define MTL_UTILITY_UTILITY_H
#include <string>
#include "mtl/mtl.hpp"

namespace mtl {
namespace util {

MTL_EXPORT unsigned char* Encrypt(unsigned char* data, int data_len,
                                   const unsigned char* key, int key_len);

MTL_EXPORT std::string Bcd(const unsigned char* data, int len);

MTL_EXPORT int EvaluateScale(const std::string& ip1, const std::string& ip2);

MTL_EXPORT uint64_t GenerateNextId();

MTL_EXPORT std::wstring AnsiToUnicode(const std::string& src);
MTL_EXPORT std::string UnicodeToAnsi(const std::wstring& src);

MTL_EXPORT std::wstring Utf8ToUnicode(const std::string& src);
MTL_EXPORT std::string UnicodeToUtf8(const std::wstring& src);

MTL_EXPORT void StringReplace(std::string& str, const std::string& src,
                               const std::string& dest);
MTL_EXPORT bool HasEspecialCharacter(const std::string& s);
MTL_EXPORT const std::string& ToMysqlString(std::string& s);
MTL_EXPORT const std::string& ToRawString(std::string& s);

} // util
} // namespace mtl

#endif // MTL_UTILITY_UTILITY_H
