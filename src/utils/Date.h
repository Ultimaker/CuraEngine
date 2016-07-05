/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_DATE_H
#define UTILS_DATE_H

#include <string>

namespace cura
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
    int year; //!< Year, e.g. 2016
    int month; //!< Month, e.g. 12, i.e. starting at 1
    int day; //!< Day, e.g. 31, i.e. starting at 1
private:
    Date(); //!< Simple constructor initializing all to -1
};

} // namespace cura

#endif // UTILS_DATE_H
