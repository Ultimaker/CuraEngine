// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/Date.h"

#include <cstdio> // sscanf
#include <cstring> // strstr
#include <iomanip> // setw, setfill
#include <sstream>

namespace cura
{

Date::Date(int year, int month, int day)
    : year_(year)
    , month_(month)
    , day_(day)
{
}

std::string Date::toStringDashed()
{
    std::ostringstream str;
    str << std::setfill('0') << std::setw(4) << year_ << "-" << std::setfill('0') << std::setw(2) << month_ << "-" << std::setfill('0') << std::setw(2) << day_;
    return str.str();
}

Date::Date()
    : year_(-1)
    , month_(-1)
    , day_(-1)
{
}

Date Date::getDate()
{
    Date ret;
    // code adapted from http://stackoverflow.com/a/1765088/2683223 Jerry Coffin
    const char* build_date = __DATE__;
    char s_month[5];
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

    std::sscanf(build_date, "%s %d %d", s_month, &ret.day_, &ret.year_);

    ret.month_ = (strstr(month_names, s_month) - month_names) / 3;

    ret.month_++; // humans count Jan as month 1, not zero
    return ret;
}


} // namespace cura
