#include "mtl/datetime/date_time.hpp"
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time.hpp>
#include <sstream>
#include "mtl/network/in_stream.hpp"
#include "mtl/network/out_stream.hpp"

namespace mtl {

DateTime::DateTime()
    : year_(1970), month_(1), day_(1)
    , hour_(0), minute_(0), second_(0), milliseconds_(0)
{
}

DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day,
                   uint8_t hour, uint8_t minute, uint8_t second, uint16_t milliseconds)
    : year_(year), month_(month), day_(day)
    , hour_(hour), minute_(minute), second_(second), milliseconds_(milliseconds)
{
}

DateTime DateTime::fromString(const std::string& str) // 2009-12-31 12:56:12.888
{
    DateTime dt;
    char sep;
    std::stringstream ss(str);
    ss >> dt.year_ >> sep >> dt.month_ >> sep >> dt.day_ >> sep
            >> dt.hour_ >> sep >> dt.minute_ >> sep >> dt.second_
            >> sep >> dt.milliseconds_;
    return dt;
}

std::string DateTime::toString(const DateTime& dt)
{
    std::string s;
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << dt.year()
       << std::setw(1) << '-'
       << std::setw(2) << dt.month()
       << std::setw(1) << '-'
       << std::setw(2) << dt.day()
       << std::setw(1) << ' '
       << std::setw(2) << dt.hour()
       << std::setw(1) << ':'
       << std::setw(2) << dt.minute()
       << std::setw(1) << ':'
       << std::setw(2) << dt.second()
       << std::setw(1) << '.'
       << std::setw(3) << dt.milliseconds();
    s = ss.str();
    return s;
}

DateTime DateTime::now()
{
    uint16_t year;
    uint8_t	month;
    uint8_t	day;
    uint8_t	hour;
    uint8_t	minute;
    uint8_t	second;
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::gregorian::date d = now.date();
    year = d.year();
    month = static_cast<uint8_t>(d.month());
    day = static_cast<uint8_t>(d.day());
    boost::posix_time::time_duration t = now.time_of_day();
    hour = uint8_t(t.hours());
    minute = uint8_t(t.minutes());
    second = uint8_t(t.seconds());
    return DateTime(year, month, day, hour, minute, second, 0);
}

network::InStream& operator>>(network::InStream& in, DateTime& dt)
{
    dt.year_ = in.readInt16();
    dt.month_ = in.readInt8();
    dt.day_ = in.readInt8();
    dt.hour_ = in.readInt8();
    dt.minute_ = in.readInt8();
    dt.second_ = in.readInt8();
    dt.milliseconds_ = in.readInt16();
    return in;
}

network::OutStream& operator<<(network::OutStream& out, const DateTime& dt)
{
    out.writeInt16(dt.year_);
    out.writeInt8(dt.month_);
    out.writeInt8(dt.day_);
    out.writeInt8(dt.hour_);
    out.writeInt8(dt.minute_);
    out.writeInt8(dt.second_);
    out.writeInt16(dt.milliseconds_);
    return out;
}
}
