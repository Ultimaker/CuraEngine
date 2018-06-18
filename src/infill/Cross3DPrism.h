/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_CROSS_3D_PRISM_H
#define INFILL_CROSS_3D_PRISM_H

#include <array>

#include "../utils/IntPoint.h"
#include "../utils/Range.h"
#include "../utils/LineSegment.h"
#include "../utils/polygon.h"

namespace cura
{
class Cross3DPrism
{
public:
    struct Triangle
    {
        /*!
         * The order in
         * Which the edges of the triangle are crossed by the Sierpinski curve.
         */
        enum class Direction
        {
            AC_TO_AB,
            AC_TO_BC,
            AB_TO_BC
        };
        Point straight_corner; //!< C, the 90* corner of the triangle
        Point a; //!< The corner closer to the start of the space filling curve
        Point b; //!< The corner closer to the end of the space filling curve
        Direction dir; //!< The (order in which) edges being crossed by the Sierpinski curve.
        bool straight_corner_is_left; //!< Whether the \ref straight_corner is left of the curve, rather than right. I.e. whether triangle ABC is counter-clockwise

        Triangle(
            Point straight_corner,
            Point a,
            Point b,
            Direction dir,
            bool straight_corner_is_left)
        : straight_corner(straight_corner)
        , a(a)
        , b(b)
        , dir(dir)
        , straight_corner_is_left(straight_corner_is_left)
        {}

        //! initialize with invalid data
        Triangle()
        {}

        std::array<Triangle, 2> subdivide() const;

        /*!
         * Get the first edge of this triangle crossed by the Sierpinski and/or Cross Fractal curve.
         * The from location is always toward the inside of the curve.
         */
        LineSegment getFromEdge() const;
        /*!
         * Get the second edge of this triangle crossed by the Sierpinski and/or Cross Fractal curve.
         * The from location is always toward the inside of the curve.
         */
        LineSegment getToEdge() const;
        //! Get the middle of the triangle
        Point getMiddle() const;
        //! convert into a polyogon with correct winding order
        Polygon toPolygon() const;
    };
    
    Triangle triangle;
    Range<coord_t> z_range;
    bool is_expanding; //!< Whether the surface is moving away from the space filling tree when going from bottom to top. (Though the eventual surface might be changed due to neighboring constraining cells)

    //! simple constructor
    Cross3DPrism(
        Triangle triangle,
        coord_t z_min,
        coord_t z_max,
        bool is_expanding
    )
    : triangle(triangle)
    , z_range(z_min, z_max)
    , is_expanding(is_expanding)
    {}

    //! initialize with invalid data
    Cross3DPrism()
    {}

    bool isHalfCube() const;
    bool isQuarterCube() const;
};


} // namespace cura


#endif // INFILL_CROSS_3D_PRISM_H
