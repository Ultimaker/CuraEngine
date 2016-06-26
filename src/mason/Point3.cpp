/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "Point3.hpp"

namespace mason {

double intToMm(coord_t val)
{
    return static_cast<double>(val)/1e3;
}

coord_t mmToInt(double val)
{
    return static_cast<coord_t>(val * 1e3);
}

double intToUm(coord_t val)
{
    return static_cast<double>(val);
}

coord_t umToInt(double val)
{
    return static_cast<coord_t>(val);
}

Point3::Point3()
{}

Point3::Point3(const coord_t _x, const coord_t _y, const coord_t _z): x(_x), y(_y), z(_z)
{}

cura::Point3 Point3::toCuraPoint3() const
{
    // Can result in loss of precision, converts from 64 bit to 32 bit.
    return cura::Point3(static_cast<int32_t>(x),
                        static_cast<int32_t>(y),
                        static_cast<int32_t>(z));
}

Point3 fromCuraPoint3(const cura::Point3 pt)
{
    return Point3(pt.x,pt.y,pt.z);
}

}
