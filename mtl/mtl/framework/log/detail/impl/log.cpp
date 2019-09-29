#include "mtl/framework/log/log.h"
#include <ostream>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/common.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/expressions/keyword.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/attributes/named_scope.hpp>

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;

BOOST_LOG_ATTRIBUTE_KEYWORD(log_severity, "Severity",
                            mtl::framework::log::SeverityLevel)
BOOST_LOG_ATTRIBUTE_KEYWORD(log_timestamp, "TimeStamp",
                            boost::posix_time::ptime)
BOOST_LOG_ATTRIBUTE_KEYWORD(log_uptime, "Uptime", attrs::timer::value_type)
BOOST_LOG_ATTRIBUTE_KEYWORD(log_scope, "Scope", attrs::named_scope::value_type)

namespace mtl {
namespace framework {
namespace log {

void initLog(const std::string& directory_name)
{
    logging::formatter formatter =
            expr::stream
            << "[" << expr::format_date_time(log_timestamp, "%H:%M:%S")
            << "]" << expr::if_(expr::has_attr(log_uptime))
               [
               expr::stream << " [" << format_date_time(log_uptime, "%O:%M:%S") << "]"
               ]
            << expr::if_(expr::has_attr(log_scope))
               [
               expr::stream << "[" << expr::format_named_scope(log_scope, keywords::format = "%n") << "]"
               ]
            << "<" << log_severity << ">" << expr::message;
    //      << "<" << expr::attr<attrs::current_thread_id::value_type>("ThreadID") << ">"

    logging::add_common_attributes();
    //  logging::core::get()->add_global_attribute("ThreadID", attrs::current_thread_id());

    auto console_sink = logging::add_console_log();
    auto file_sink = logging::add_file_log(
                         keywords::file_name = "logs/%Y-%m-%d_%N.log",      //文件名
                         keywords::rotation_size = 10*1024*1024,       //单个文件限制大小
                         keywords::time_based_rotation = sinks::file::rotation_at_time_point(0,0,0) //每天重建
                                                         );

    file_sink->locked_backend()->set_file_collector(sinks::file::make_collector(
                                                        keywords::target = directory_name,        //文件夹名
                                                        keywords::max_size = 50*1024*1024,    //文件夹所占最大空间
                                                        keywords::min_free_space = 100*1024*1024  //磁盘最小预留空间
                                                                                   ));

    file_sink->set_filter(log_severity >= kWarning);   //日志级别过滤
    file_sink->locked_backend()->scan_for_files();
    console_sink->set_formatter(formatter);
    file_sink->set_formatter(formatter);
    file_sink->locked_backend()->auto_flush(true);

    logging::core::get()->add_global_attribute("Scope", attrs::named_scope());
    logging::core::get()->add_sink(console_sink);
    logging::core::get()->add_sink(file_sink);
}

} // namespace log
} // namespace framework
} // namespace mtl
