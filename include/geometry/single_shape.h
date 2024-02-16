// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_SINGLE_SHAPE_H
#define GEOMETRY_SINGLE_SHAPE_H

#include "geometry/shape.h"
#include "point2ll.h"

namespace cura
{

class Polygon;

/*!
 * A single area with holes. The first polygon is the outline, while the rest are holes within this outline.
 *
 * This class has little more functionality than Polygons, but serves to show that a specific instance is ordered such that the first Polygon is the outline and the rest are holes.
 */
class SingleShape : public Shape
{
public:
    Polygon& outerPolygon();

    const Polygon& outerPolygon() const;

    /*!
     * Tests whether the given point is inside this polygon part.
     * \param p The point to test whether it is inside.
     * \param border_result If the point is exactly on the border, this will be
     * returned instead.
     */
    bool inside(const Point2LL& p, bool border_result = false) const;
};

} // namespace cura

#endif // GEOMETRY_MONO_SHAPE_H
