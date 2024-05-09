// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_SINGLE_SHAPE_H
#define GEOMETRY_SINGLE_SHAPE_H

#include "geometry/Shape.h"

namespace cura
{

class Polygon;

/*!
 * @brief A single area with holes. The first polygon is the outline, while the rest are holes within this outline.
 * @sa https://github.com/Ultimaker/CuraEngine/wiki/Geometric-Base-Types#singleshape
 *
 * This class has little more functionality than Shape, but serves to show that a specific instance
 * is ordered such that the first Polygon is the outline and the rest are holes.
 */
class SingleShape : public Shape
{
public:
    SingleShape() = default;

    explicit SingleShape(Shape&& shape)
        : Shape{ std::move(shape) } {};

    Polygon& outerPolygon();

    [[nodiscard]] const Polygon& outerPolygon() const;

    /*!
     * Tests whether the given point is inside this polygon part.
     * \param p The point to test whether it is insisinglehde.
     * \param border_result If the point is exactly on the border, this will be
     * returned instead.
     */
    [[nodiscard]] bool inside(const Point2LL& p, bool border_result = false) const;
};

} // namespace cura

#endif // GEOMETRY_SINGLE_SHAPE_H
