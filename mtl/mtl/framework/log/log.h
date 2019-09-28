#ifndef MTL_FRAMEWORK_LOG_H
#define MTL_FRAMEWORK_LOG_H
#include <boost/log/common.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <string>
#include "mtl/mtl.hpp"

namespace mtl {
namespace framework {
namespace log {

enum SeverityLevel {
  kNotice,
  kDebug,
  kWarning,
  kInfo,
  kError,
  kFatal
};

void MTL_EXPORT InitLog(const std::string& directory_name);

BOOST_LOG_INLINE_GLOBAL_LOGGER_DEFAULT(
    global_logger, boost::log::sources::severity_logger_mt<SeverityLevel>)

} // log
} // framework
} // mtl

// The formatting logic for the severity level
template <typename CharT, typename TraitsT>
inline std::basic_ostream<CharT, TraitsT>& operator<<(
    std::basic_ostream<CharT, TraitsT>& strm,
    mtl::framework::log::SeverityLevel lvl) {
  static const char* const str[] = {
    "Notice",
    "Debug",
    "Warning",
    "Info",
    "Error",
    "Fatal"
  };
  if (static_cast<std::size_t>(lvl) < (sizeof(str) / sizeof(*str)))
    strm << str[lvl];
  else
    strm << static_cast<int>(lvl);
  return strm;
}

#endif // MTL_FRAMEWORK_LOG_H
