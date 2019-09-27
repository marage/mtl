#ifndef MTL_DATE_TIME_HPP
#define MTL_DATE_TIME_HPP
#include "mtl/mtl.hpp"
#include <string>

namespace mtl {

// forward classes
namespace network {
class InStream;
class OutStream;
}

class MTL_EXPORT DateTime
{
public:
    DateTime();
    DateTime(uint16_t year, uint8_t month, uint8_t day,
             uint8_t hour, uint8_t minute, uint8_t second, uint16_t milliseconds);

    uint16_t year() const { return year_; }
    uint8_t month() const { return month_; }
    uint8_t day() const { return day_; }
    uint8_t hour() const { return hour_; }
    uint8_t minute() const { return minute_; }
    uint8_t second() const { return second_; }
    uint16_t milliseconds() const { return milliseconds_; }

    static DateTime fromString(const std::string& t);
    static std::string toString(const DateTime& dt);
    static DateTime now();

    friend MTL_EXPORT network::InStream& operator>>(network::InStream& in, DateTime& dt);
    friend MTL_EXPORT network::OutStream& operator<<(network::OutStream& out, const DateTime& dt);

private:
    uint16_t year_;
    uint8_t month_;
    uint8_t day_;
    uint8_t hour_;
    uint8_t minute_;
    uint8_t second_;
    uint16_t milliseconds_;
};

} // namespace mtl

#endif // MTL_DATE_TIME_HPP
