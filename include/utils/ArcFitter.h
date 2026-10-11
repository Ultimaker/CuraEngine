// Copyright (c) 2025 UltiMaker and contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_ARCFITTER_H
#define UTILS_ARCFITTER_H

#include <vector>

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{

/*!
 * Replaces runs of short straight extrusion segments by circular arcs, so that they can be written as G2/G3 moves.
 *
 * The approach follows the arc fitting of OrcaSlicer (itself derived from ArcWelder): keep adding points to a candidate
 * arc while one single circle still passes within a tolerance of every point, the points keep turning in the same
 * direction, and the arc is about as long as the original polyline. The longest acceptable arc is emitted and the
 * search continues from its end point.
 */
class ArcFitter
{
public:
    /*! A stretch of the input points, printed either as a single arc or as straight lines. */
    struct Segment
    {
        size_t first_index; //!< Index of the point the segment starts at (it is the end point of the previous segment).
        size_t last_index; //!< Index of the last point of the segment.
        bool is_arc{ false };
        // Only for arcs:
        double center_x{ 0.0 }; //!< Centre of the circle, in microns.
        double center_y{ 0.0 };
        double radius{ 0.0 }; //!< In microns.
        bool clockwise{ false };
        double sweep{ 0.0 }; //!< Angle covered by the arc, in radians, always positive.

        /*! Length of the arc, in microns. */
        double arcLength() const
        {
            return radius * sweep;
        }
    };

    /*!
     * Split a polyline in arcs and straight stretches.
     * @param points The polyline. The first point is where the extruder already is; the segments start there.
     * @param tolerance Largest allowed distance between the arc and the original polyline, in microns.
     * @param max_radius Arcs with a larger radius are considered straight lines, in microns.
     * @param min_radius Arcs with a smaller radius are not used, in microns.
     * @return Consecutive segments that together cover all the points; a straight segment can span multiple points.
     */
    static std::vector<Segment> fit(const std::vector<Point2LL>& points, coord_t tolerance, coord_t max_radius, coord_t min_radius);

private:
    /*! Try to fit one arc through the points between (and including) the two indices. */
    static bool tryFit(const std::vector<Point2LL>& points, size_t first, size_t last, coord_t tolerance, coord_t max_radius, coord_t min_radius, Segment& result);
};

} // namespace cura

#endif // UTILS_ARCFITTER_H
