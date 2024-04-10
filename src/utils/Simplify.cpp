// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/Simplify.h"

#include <limits>
#include <queue> //Priority queue to prioritise removing unimportant vertices.

#include "geometry/closed_polyline.h"
#include "geometry/open_polyline.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

Simplify::Simplify(const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation)
    : max_resolution_(max_resolution)
    , max_deviation_(max_deviation)
    , max_area_deviation_(max_area_deviation)
{
}

Simplify::Simplify(const Settings& settings)
    : max_resolution_(settings.get<coord_t>("meshfix_maximum_resolution"))
    , max_deviation_(settings.get<coord_t>("meshfix_maximum_deviation"))
    , max_area_deviation_(settings.get<size_t>("meshfix_maximum_extrusion_area_deviation"))
{
}

Shape Simplify::polygon(const Shape& polygons) const
{
    Shape result;
    for (size_t i = 0; i < polygons.size(); ++i)
    {
        result.push_back(polygon(polygons[i]), true);
    }
    return result;
}

Polygon Simplify::polygon(const Polygon& polygon) const
{
    constexpr bool is_closed = true;
    return simplify(polygon, is_closed);
}

ExtrusionLine Simplify::polygon(const ExtrusionLine& polygon) const
{
    constexpr bool is_closed = true;
    return simplify(polygon, is_closed);
}

template<class LineType>
LinesSet<LineType> Simplify::polyline(const LinesSet<LineType>& polylines) const
{
    LinesSet<LineType> result;
    for (size_t i = 0; i < polylines.size(); ++i)
    {
        result.push_back(polyline(polylines[i]));
    }
    return result;
}

MixedLinesSet Simplify::polyline(const MixedLinesSet& polylines) const
{
    MixedLinesSet result;
    for (const std::shared_ptr<Polyline>& polyline_ptr : polylines)
    {
        if (polyline_ptr)
        {
            if (std::shared_ptr<const OpenPolyline> open_polyline = std::dynamic_pointer_cast<const OpenPolyline>(polyline_ptr))
            {
                result.push_back(std::make_shared<OpenPolyline>(polyline(*open_polyline)));
            }
            if (std::shared_ptr<const ClosedPolyline> closed_polyline = std::dynamic_pointer_cast<const ClosedPolyline>(polyline_ptr))
            {
                result.push_back(std::make_shared<ClosedPolyline>(polyline(*closed_polyline)));
            }
        }
    }
    return result;
}

ClosedPolyline Simplify::polyline(const ClosedPolyline& polyline) const
{
    return simplify(polyline, polyline.isExplicitelyClosed());
}

OpenPolyline Simplify::polyline(const OpenPolyline& polyline) const
{
    constexpr bool is_closed = false;
    return simplify(polyline, is_closed);
}

ExtrusionLine Simplify::polyline(const ExtrusionLine& polyline) const
{
    constexpr bool is_closed = false;
    return simplify(polyline, is_closed);
}

size_t Simplify::nextNotDeleted(size_t index, const std::vector<bool>& to_delete) const
{
    const size_t size = to_delete.size();
    for (index = (index + 1) % size; to_delete[index]; index = (index + 1) % size)
        ; // Changes the index variable in-place until we found one that is not deleted.
    return index;
}

size_t Simplify::previousNotDeleted(size_t index, const std::vector<bool>& to_delete) const
{
    const size_t size = to_delete.size();
    for (index = (index + size - 1) % size; to_delete[index]; index = (index + size - 1) % size)
        ; // Changes the index variable in-place until we found one that is not deleted.
    return index;
}

template<>
ExtrusionLine Simplify::createEmpty(const ExtrusionLine& original) const
{
    ExtrusionLine result(original.inset_idx_, original.is_odd_);
    result.is_closed_ = original.is_closed_;
    return result;
}

template<typename Polygonal>
Polygonal Simplify::createEmpty(const Polygonal& /*original*/) const
{
    return Polygonal();
}

void Simplify::appendVertex(Polyline& polygon, const Point2LL& vertex) const
{
    polygon.push_back(vertex);
}

void Simplify::appendVertex(ExtrusionLine& extrusion_line, const ExtrusionJunction& vertex) const
{
    extrusion_line.junctions_.push_back(vertex);
}

const Point2LL& Simplify::getPosition(const Point2LL& vertex) const
{
    return vertex;
}

const Point2LL& Simplify::getPosition(const ExtrusionJunction& vertex) const
{
    return vertex.p_;
}

Point2LL Simplify::createIntersection([[maybe_unused]] const Point2LL& before, const Point2LL intersection, [[maybe_unused]] const Point2LL& after) const
{
    return intersection;
}

ExtrusionJunction Simplify::createIntersection(const ExtrusionJunction& before, const Point2LL intersection, const ExtrusionJunction& after) const
{
    // Average the extrusion width of the line.
    // More correct would be to see where along the line the intersection occurs with a projection or something.
    // But these details are so small, and this solution is so much quicker and simpler.
    return ExtrusionJunction(intersection, (before.w_ + after.w_) / 2, before.perimeter_index_);
}

coord_t Simplify::getAreaDeviation([[maybe_unused]] const Point2LL& before, [[maybe_unused]] const Point2LL& vertex, [[maybe_unused]] const Point2LL& after) const
{
    return 0; // Fixed-width polygons don't have any deviation.
}

coord_t Simplify::getAreaDeviation(const ExtrusionJunction& before, const ExtrusionJunction& vertex, const ExtrusionJunction& after) const
{
    /*
     * A             B                          C              A                                        C
     * ---------------                                         **************
     * |             |                                         ------------------------------------------
     * |             |--------------------------|  B removed   |            |***************************|
     * |             |                          |  --------->  |            |                           |
     * |             |--------------------------|              |            |***************************|
     * |             |                                         ------------------------------------------
     * ---------------             ^                           **************
     *       ^                B.w + C.w / 2                                       ^
     *  A.w + B.w / 2                                               new_width = weighted_average_width
     *
     *
     * ******** denote the total extrusion area deviation error in the consecutive segments as a result of using the
     * weighted-average width for the entire extrusion line.
     *
     * */
    const coord_t ab_length = vSize(vertex - before);
    const coord_t bc_length = vSize(after - vertex);
    const coord_t ac_length = vSize(after - before);
    if (ab_length == 0 || ac_length == 0 || bc_length == 0)
    {
        return 0; // Either of the line segments is zero, so the deviation of one of the line segments doesn't matter (not printed). So effectively there is no deviation.
    }
    const coord_t width_diff = std::max(std::abs(vertex.w_ - before.w_), std::abs(after.w_ - vertex.w_));
    if (width_diff > 1)
    {
        // Adjust the width only if there is a difference, or else the rounding errors may produce the wrong
        // weighted average value.
        const coord_t ab_weight = (before.w_ + vertex.w_) / 2;
        const coord_t bc_weight = (vertex.w_ + after.w_) / 2;
        const coord_t weighted_average_width = (ab_length * ab_weight + bc_length * bc_weight) / ac_length;
        return std::abs(ab_weight - weighted_average_width) * ab_length + std::abs(bc_weight - weighted_average_width) * bc_length;
    }
    else
    {
        // If the width difference is very small, then select the width of the segment that is longer
        return ab_length > bc_length ? width_diff * bc_length : width_diff * ab_length;
    }
}

template LinesSet<OpenPolyline> Simplify::polyline(const LinesSet<OpenPolyline>& polylines) const;
template LinesSet<ClosedPolyline> Simplify::polyline(const LinesSet<ClosedPolyline>& polylines) const;

} // namespace cura
