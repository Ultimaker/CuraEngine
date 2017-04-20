/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "Date.h"

#include <sstream>
#include <cstdio> // sscanf
#include <cstring> // strstr
#include <iomanip> // setw, setfill

namespace cura
{

Date::Date(int year, int month, int day)
: year(year)
, month(month)
, day(day)
{
}

std::string Date::toStringDashed()
{
    std::ostringstream str;
    str << std::setfill('0') << std::setw(4) << year << "-"
        << std::setfill('0') << std::setw(2) << month << "-"
        << std::setfill('0') << std::setw(2) << day;
    return str.str();
}

Date::Date()
: year(-1)
, month(-1)
, day(-1)
{
}

Date Date::getDate()
{
    Date ret;
    // code adapted from http://stackoverflow.com/a/1765088/2683223 Jerry Coffin
    const char* build_date = __DATE__;
    char s_month[5];
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

    std::sscanf(build_date, "%s %d %d", s_month, &ret.day, &ret.year);

    ret.month = (strstr(month_names, s_month) - month_names) / 3;

    ret.month++; // humans count Jan as month 1, not zero
    return ret;
}


} // namespace cura