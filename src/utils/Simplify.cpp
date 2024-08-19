// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/Simplify.h"

#include <limits>
#include <queue> //Priority queue to prioritise removing unimportant vertices.

#include "geometry/ClosedPolyline.h"
#include "geometry/MixedLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "settings/Settings.h" //To load the parameters from a Settings object.
#include "utils/ExtrusionLine.h"
#include "utils/linearAlg2D.h" //To calculate line deviations and intersecting lines.

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
        result.push_back(polygon(polygons[i]), CheckNonEmptyParam::OnlyIfNotEmpty);
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
        result.push_back(polyline(polylines[i]), CheckNonEmptyParam::OnlyIfNotEmpty);
    }
    return result;
}

MixedLinesSet Simplify::polyline(const MixedLinesSet& polylines) const
{
    MixedLinesSet result;
    for (const PolylinePtr& polyline_ptr : polylines)
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
ExtrusionLine Simplify::createEmpty(const ExtrusionLine& original)
{
    ExtrusionLine result(original.inset_idx_, original.is_odd_);
    result.is_closed_ = original.is_closed_;
    return result;
}

template<typename Polygonal>
Polygonal Simplify::createEmpty(const Polygonal& /*original*/)
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

template<typename Polygonal>
bool Simplify::detectSmall(const Polygonal& polygon, const coord_t& min_size) const
{
    if (polygon.size() < min_size) // For polygon, 2 or fewer vertices is degenerate. Delete it. For polyline, 1 vertex is degenerate.
    {
        return true;
    }
    if (polygon.size() == min_size)
    {
        const auto a = getPosition(polygon[0]);
        const auto b = getPosition(polygon[1]);
        const auto c = getPosition(polygon[polygon.size() - 1]);
        if (std::max(std::max(vSize2(b - a), vSize2(c - a)), vSize2(c - b)) < min_resolution * min_resolution)
        {
            // ... unless they are degenetate.
            return true;
        }
    }
    return false;
}

template<typename Polygonal>
Polygonal Simplify::simplify(const Polygonal& polygon, const bool is_closed) const
{
    const size_t min_size = is_closed ? 3 : 2;
    if (detectSmall(polygon, min_size))
    {
        return createEmpty(polygon);
    }
    if (polygon.size() == min_size) // For polygon, don't reduce below 3. For polyline, not below 2.
    {
        return polygon;
    }

    std::vector<bool> to_delete(polygon.size(), false);
    auto comparator = [](const std::pair<size_t, coord_t>& vertex_a, const std::pair<size_t, coord_t>& vertex_b)
    {
        return vertex_a.second > vertex_b.second || (vertex_a.second == vertex_b.second && vertex_a.first > vertex_b.first);
    };
    std::priority_queue<std::pair<size_t, coord_t>, std::vector<std::pair<size_t, coord_t>>, decltype(comparator)> by_importance(comparator);

    Polygonal result = polygon; // Make a copy so that we can also shift vertices.
    for (int64_t current_removed = -1; (polygon.size() - current_removed) > min_size && current_removed != 0;)
    {
        current_removed = 0;

        // Add the initial points.
        for (size_t i = 0; i < result.size(); ++i)
        {
            if (to_delete[i])
            {
                continue;
            }
            const coord_t vertex_importance = importance(result, to_delete, i, is_closed);
            by_importance.emplace(i, vertex_importance);
        }

        // Iteratively remove the least important point until a threshold.
        coord_t vertex_importance = 0;
        while (! by_importance.empty() && (polygon.size() - current_removed) > min_size)
        {
            std::pair<size_t, coord_t> vertex = by_importance.top();
            by_importance.pop();
            // The importance may have changed since this vertex was inserted. Re-compute it now.
            // If it doesn't change, it's safe to process.
            vertex_importance = importance(result, to_delete, vertex.first, is_closed);
            if (vertex_importance != vertex.second)
            {
                by_importance.emplace(vertex.first, vertex_importance); // Re-insert with updated importance.
                continue;
            }

            if (vertex_importance <= max_deviation_ * max_deviation_)
            {
                current_removed += remove(result, to_delete, vertex.first, vertex_importance, is_closed) ? 1 : 0;
            }
        }
    }

    // Now remove the marked vertices in one sweep.
    Polygonal filtered = createEmpty(polygon);
    for (size_t i = 0; i < result.size(); ++i)
    {
        if (! to_delete[i])
        {
            appendVertex(filtered, result[i]);
        }
    }

    if (detectSmall(filtered, min_size))
    {
        return createEmpty(filtered);
    }
    return filtered;
}

template<typename Polygonal>
coord_t Simplify::importance(const Polygonal& polygon, const std::vector<bool>& to_delete, const size_t index, const bool is_closed) const
{
    const size_t poly_size = polygon.size();
    if (! is_closed && (index == 0 || index == poly_size - 1))
    {
        return std::numeric_limits<coord_t>::max(); // Endpoints of the polyline must always be retained.
    }
    // From here on out we can safely look at the vertex neighbors and assume it's a polygon. We won't go out of bounds of the polyline.

    const Point2LL& vertex = getPosition(polygon[index]);
    const size_t before_index = previousNotDeleted(index, to_delete);
    const size_t after_index = nextNotDeleted(index, to_delete);

    const coord_t area_deviation = getAreaDeviation(polygon[before_index], polygon[index], polygon[after_index]);
    if (area_deviation > max_area_deviation_) // Removing this line causes the variable line width to get flattened out too much.
    {
        return std::numeric_limits<coord_t>::max();
    }

    const Point2LL& before = getPosition(polygon[before_index]);
    const Point2LL& after = getPosition(polygon[after_index]);
    const coord_t deviation2 = LinearAlg2D::getDist2FromLine(vertex, before, after);
    if (deviation2 <= min_resolution * min_resolution) // Deviation so small that it's always desired to remove them.
    {
        return deviation2;
    }
    if (vSize2(before - vertex) > max_resolution_ * max_resolution_ && vSize2(after - vertex) > max_resolution_ * max_resolution_)
    {
        return std::numeric_limits<coord_t>::max(); // Long line segments, no need to remove this one.
    }
    return deviation2;
}

template<typename Polygonal>
bool Simplify::remove(Polygonal& polygon, std::vector<bool>& to_delete, const size_t vertex, const coord_t deviation2, const bool is_closed) const
{
    if (deviation2 <= min_resolution * min_resolution)
    {
        // At less than the minimum resolution we're always allowed to delete the vertex.
        // Even if the adjacent line segments are very long.
        to_delete[vertex] = true;
        return true;
    }

    const size_t before = previousNotDeleted(vertex, to_delete);
    const size_t after = nextNotDeleted(vertex, to_delete);
    const Point2LL& vertex_position = getPosition(polygon[vertex]);
    const Point2LL& before_position = getPosition(polygon[before]);
    const Point2LL& after_position = getPosition(polygon[after]);
    const coord_t length2_before = vSize2(vertex_position - before_position);
    const coord_t length2_after = vSize2(vertex_position - after_position);

    if (length2_before <= max_resolution_ * max_resolution_ && length2_after <= max_resolution_ * max_resolution_) // Both adjacent line segments are short.
    {
        // Removing this vertex does little harm. No long lines will be shifted.
        to_delete[vertex] = true;
        return true;
    }

    // Otherwise, one edge next to this vertex is longer than max_resolution. The other is shorter.
    // In this case we want to remove the short edge by replacing it with a vertex where the two surrounding edges intersect.
    // Find the two line segments surrounding the short edge here ("before" and "after" edges).
    Point2LL before_from, before_to, after_from, after_to;
    if (length2_before <= length2_after) // Before is the shorter line.
    {
        if (! is_closed && before == 0) // No edge before the short edge.
        {
            return false; // Edge cannot be deleted without shifting a long edge. Don't remove anything.
        }
        const size_t before_before = previousNotDeleted(before, to_delete);
        before_from = getPosition(polygon[before_before]);
        before_to = getPosition(polygon[before]);
        after_from = getPosition(polygon[vertex]);
        after_to = getPosition(polygon[after]);
    }
    else
    {
        if (! is_closed && after == polygon.size() - 1) // No edge after the short edge.
        {
            return false; // Edge cannot be deleted without shifting a long edge. Don't remove anything.
        }
        const size_t after_after = nextNotDeleted(after, to_delete);
        before_from = getPosition(polygon[before]);
        before_to = getPosition(polygon[vertex]);
        after_from = getPosition(polygon[after]);
        after_to = getPosition(polygon[after_after]);
    }
    Point2LL intersection;
    const bool did_intersect = LinearAlg2D::lineLineIntersection(before_from, before_to, after_from, after_to, intersection);
    if (! did_intersect) // Lines are parallel.
    {
        return false; // Cannot remove edge without shifting a long edge. Don't remove anything.
    }
    const coord_t intersection_deviation = LinearAlg2D::getDist2FromLineSegment(before_to, intersection, after_from);
    if (intersection_deviation <= max_deviation_ * max_deviation_) // Intersection point doesn't deviate too much. Use it!
    {
        to_delete[vertex] = true;
        polygon[length2_before <= length2_after ? before : after] = createIntersection(polygon[before], intersection, polygon[after]);
        return true;
    }
    return false;
}

template OpenLinesSet Simplify::polyline(const OpenLinesSet& polylines) const;
template ClosedLinesSet Simplify::polyline(const ClosedLinesSet& polylines) const;

} // namespace cura
