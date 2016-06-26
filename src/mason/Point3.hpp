/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_POINT3_HPP
#define INCLUDED_MASON_POINT3_HPP

#include "../utils/intpoint.h"

namespace mason {

using coord_t = ClipperLib::cInt;

double intToMm(coord_t val);
coord_t mmToInt(double val);

double intToUm(coord_t val);
coord_t umToInt(double val);

class Point3
{
public:
    coord_t x,y,z;
    Point3();
    Point3(const coord_t _x, const coord_t _y, const coord_t _z);

    cura::Point3 toCuraPoint3() const;
};

Point3 fromCuraPoint3(const cura::Point3 pt);

}

#endif
