/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "Cross3DPrism.h"

// #include <cassert>
// #include <sstream>  // debug TODO

#include "../utils/math.h"
#include "../utils/linearAlg2D.h"
// #include "../utils/gettime.h"


namespace cura {

LineSegment Cross3DPrism::Triangle::getFromEdge() const
{
    LineSegment ret;
    switch(dir)
    {
        case Direction::AB_TO_BC:
            ret = LineSegment(a, b);
            break;
        case Direction::AC_TO_AB:
            ret = LineSegment(straight_corner, a);
            break;
        case Direction::AC_TO_BC:
            ret = LineSegment(straight_corner, a);
            break;
    }
    if (straight_corner_is_left)
    {
        ret.reverse();
    }
    return ret;
}

LineSegment Cross3DPrism::Triangle::getToEdge() const
{
    LineSegment ret;
    switch(dir)
    {
        case Direction::AB_TO_BC:
            ret = LineSegment(straight_corner, b);
            break;
        case Direction::AC_TO_AB:
            ret = LineSegment(b, a);
            break;
        case Direction::AC_TO_BC:
            ret = LineSegment(straight_corner, b);
            break;
    }
    if (straight_corner_is_left)
    {
        ret.reverse();
    }
    return ret;
}

Point Cross3DPrism::Triangle::getMiddle() const
{
    return (straight_corner + a + b) / 3;
}

Polygon Cross3DPrism::Triangle::toPolygon() const
{
    Polygon ret;
    ret.add(straight_corner);
    Point second = a;
    Point third = b;
    if (!straight_corner_is_left)
    {
        std::swap(second, third);
    }
    ret.add(second);
    ret.add(third);
    assert(ret.area() > 0);
    return ret;
}

std::array<Cross3DPrism::Triangle, 2> Cross3DPrism::Triangle::subdivide() const
{
/*
 * Triangles are subdivided into two children like so:
 * |\       |\        .
 * |A \     |A \      .
 * |    \   |    \    . where C is always the 90* straight corner
 * |     C\ |C____B\  .       The direction between A and B is maintained
 * |      / |C    A/
 * |    /   |    /      Note that the polygon direction flips between clockwise and CCW each subdivision
 * |B /     |B /        as does Triangle::straight_corner_is_left
 * |/       |/
 * 
 * The direction of the space filling curve along each triangle is recorded:
 * 
 * |\                           |\                                        .
 * |B \  AC_TO_BC               |B \   AC_TO_AB                           .
 * |  ↑ \                       |  ↑ \                                    .
 * |  ↑  C\  subdivides into    |C_↑__A\                                  .
 * |  ↑   /                     |C ↑  B/                                  .
 * |  ↑ /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BC                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AC_TO_AB               |B \   AC_TO_BC                           .
 * |    \                       |↖   \                                    .
 * |↖    C\  subdivides into    |C_↖__A\                                  .
 * |  ↖   /                     |C ↑  B/                                  .
 * |    /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BC                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AB_TO_BC               |B \   AC_TO_AB                           .
 * |  ↗ \                       |  ↑ \                                    .
 * |↗    C\  subdivides into    |C_↑__A\                                  .
 * |      /                     |C ↗  B/                                  .
 * |    /                       |↗   /                                    .
 * |A /                         |A /   AC_TO_BC                           .
 * |/                           |/                                        .
 */
    std::array<Cross3DPrism::Triangle, 2> ret;
    Point middle = (a + b) / 2;
    ret[0].straight_corner = middle;
    ret[0].a = a;
    ret[0].b = straight_corner;
    ret[0].straight_corner_is_left = !straight_corner_is_left;
    ret[1].straight_corner = middle;
    ret[1].a = straight_corner;
    ret[1].b = b;
    ret[1].straight_corner_is_left = !straight_corner_is_left;
    switch(dir)
    {
        case Triangle::Direction::AB_TO_BC:
            ret[0].dir = Triangle::Direction::AC_TO_BC;
            ret[1].dir = Triangle::Direction::AC_TO_AB;
            break;
        case Triangle::Direction::AC_TO_AB:
            ret[0].dir = Triangle::Direction::AB_TO_BC;
            ret[1].dir = Triangle::Direction::AC_TO_BC;
            break;
        case Triangle::Direction::AC_TO_BC:
            ret[0].dir = Triangle::Direction::AB_TO_BC;
            ret[1].dir = Triangle::Direction::AC_TO_AB;
            break;
    }
    return ret;
}

bool Cross3DPrism::isHalfCube() const
{
    return std::abs(vSize(triangle.straight_corner - triangle.b) - (z_range.max - z_range.min)) < 10;
}
bool Cross3DPrism::isQuarterCube() const
{
    return std::abs(vSize(triangle.a - triangle.b) - (z_range.max - z_range.min)) < 10;
}


}; // namespace cura
