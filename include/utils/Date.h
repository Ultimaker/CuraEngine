// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <string>

namespace curaengine
{

/*!
 * Simple class to represent a year, month and day.
 */
class Date
{
public:
    Date(int year, int month, int day); //!< Simple constructor
    static Date getDate(); //!< Get the current date (compile time)
    std::string toStringDashed(); //!< Get a formatted string: yyyy-mm-dd

protected:
    int year_; //!< Year, e.g. 2016
    int month_; //!< Month, e.g. 12, i.e. starting at 1
    int day_; //!< Day, e.g. 31, i.e. starting at 1

private:
    Date(); //!< Simple constructor initializing all to -1
};

} // namespace curaengine
