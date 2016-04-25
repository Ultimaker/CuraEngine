/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_DATE_H
#define UTILS_DATE_H

#include <string>

namespace cura
{

class Date
{
public:
    Date(int year, int month, int day);
    static Date getDate();
    std::string toStringDashed();
protected:
    int year, month, day;
private:
    Date();
};

} // namespace cura

#endif // UTILS_DATE_H
