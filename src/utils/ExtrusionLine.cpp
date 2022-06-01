//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm>

#include "ExtrusionLine.h"
#include "linearAlg2D.h"
#include "Simplify.h"

namespace cura
{

ExtrusionLine::ExtrusionLine(const size_t inset_idx, const bool is_odd)
: inset_idx(inset_idx)
, is_odd(is_odd)
, is_closed(false)
{}

coord_t ExtrusionLine::getLength() const
{
    if (junctions.empty())
    {
        return 0;
    }
    coord_t len = 0;
    ExtrusionJunction prev = junctions.front();
    for (const ExtrusionJunction& next : junctions)
    {
        len += vSize(next.p - prev.p);
        prev = next;
    }
    if (is_closed)
    {
        len += vSize(front().p - back().p);
    }
    return len;
}

coord_t ExtrusionLine::getMinimalWidth() const
{
    return std::min_element(junctions.cbegin(), junctions.cend(),
                            [](const ExtrusionJunction& l, const ExtrusionJunction& r)
                            {
                                return l.w < r.w;
                            })->w;
}

void ExtrusionLine::simplify(const coord_t smallest_line_segment_squared, const coord_t allowed_error_distance_squared, const coord_t maximum_extrusion_area_deviation)
{
    Simplify simplifier(std::sqrt(smallest_line_segment_squared), std::sqrt(allowed_error_distance_squared), std::sqrt(maximum_extrusion_area_deviation));
    ExtrusionLine result;
    if (vSize2(junctions.front().p - junctions.back().p) == 0)
    {
        result = simplifier.polygon(*this);
    }
    else
    {
        result = simplifier.polyline(*this);
    }
    junctions = result.junctions;
    
    /*const size_t min_path_size = is_closed ? 3 : 2;
    if (junctions.size() <= min_path_size)
    {
        return;
    }

    // TODO: allow for the first point to be removed in case of simplifying closed Extrusionlines.*/

    /* ExtrusionLines are treated as (open) polylines, so in case an ExtrusionLine is actually a closed polygon, its
     * starting and ending points will be equal (or almost equal). Therefore, the simplification of the ExtrusionLine
     * should not touch the first and last points. As a result, start simplifying from point at index 1.
     * */
    /*std::vector<ExtrusionJunction> new_junctions;
    // Starting junction should always exist in the simplified path
    new_junctions.emplace_back(junctions.front());*/

    /* Initially, previous_previous is always the same as previous because, for open ExtrusionLines the last junction
     * cannot be taken into consideration when checking the points at index 1. For closed ExtrusionLines, the first and
     * last junctions are anyway the same.
     * */
    //ExtrusionJunction previous_previous = junctions.front();
    //ExtrusionJunction previous = junctions.front();

    /* When removing a vertex, we check the height of the triangle of the area
     being removed from the original polygon by the simplification. However,
     when consecutively removing multiple vertices the height of the previously
     removed vertices w.r.t. the shortcut path changes.
     In order to not recompute the new height value of previously removed
     vertices we compute the height of a representative triangle, which covers
     the same amount of area as the area being cut off. We use the Shoelace
     formula to accumulate the area under the removed segments. This works by
     computing the area in a 'fan' where each of the blades of the fan go from
     the origin to one of the segments. While removing vertices the area in
     this fan accumulates. By subtracting the area of the blade connected to
     the short-cutting segment we obtain the total area of the cutoff region.
     From this area we compute the height of the representative triangle using
     the standard formula for a triangle area: A = .5*b*h
     */
    /*const ExtrusionJunction& initial = junctions.at(1);
    coord_t accumulated_area_removed = previous.p.X * initial.p.Y - previous.p.Y * initial.p.X; // Twice the Shoelace formula for area of polygon per line segment.

    for (size_t point_idx = 1; point_idx < junctions.size() - 1; point_idx++)
    {
        const ExtrusionJunction& current = junctions[point_idx];

        // Spill over in case of overflow, unless the [next] vertex will then be equal to [previous].
        const bool spill_over = point_idx + 1 == junctions.size() && new_junctions.size() > 1;
        ExtrusionJunction& next = spill_over ? new_junctions[0] : junctions[point_idx + 1];

        const coord_t removed_area_next = current.p.X * next.p.Y - current.p.Y * next.p.X; // Twice the Shoelace formula for area of polygon per line segment.
        const coord_t negative_area_closing = next.p.X * previous.p.Y - next.p.Y * previous.p.X; // Area between the origin and the short-cutting segment
        accumulated_area_removed += removed_area_next;

        const coord_t length2 = vSize2(current - previous);
        if (length2 < 25)
        {
            // We're allowed to always delete segments of less than 5 micron. The width in this case doesn't matter that much.
            continue;
        }

        const coord_t area_removed_so_far = accumulated_area_removed + negative_area_closing; // Close the shortcut area polygon
        const coord_t base_length_2 = vSize2(next - previous);

        if (base_length_2 == 0) // Two line segments form a line back and forth with no area.
        {
            continue; // Remove the junction (vertex).
        }
        //We want to check if the height of the triangle formed by previous, current and next vertices is less than allowed_error_distance_squared.
        //1/2 L = A           [actual area is half of the computed shoelace value] // Shoelace formula is .5*(...) , but we simplify the computation and take out the .5
        //A = 1/2 * b * h     [triangle area formula]
        //L = b * h           [apply above two and take out the 1/2]
        //h = L / b           [divide by b]
        //h^2 = (L / b)^2     [square it]
        //h^2 = L^2 / b^2     [factor the divisor]
        const coord_t height_2 = area_removed_so_far * area_removed_so_far / base_length_2;
        coord_t weighted_average_width;
        const coord_t extrusion_area_error = calculateExtrusionAreaDeviationError(previous, current, next, weighted_average_width);
        if ((height_2 <= 1 //Almost exactly colinear (barring rounding errors).
             && LinearAlg2D::getDistFromLine(current.p, previous.p, next.p) <= 1) // Make sure that height_2 is not small because of cancellation of positive and negative areas
            // We shouldn't remove middle junctions of colinear segments if the area changed for the C-P segment is exceeding the maximum allowed
             && extrusion_area_error <= maximum_extrusion_area_deviation)
        {
            // Remove the current junction (vertex).
            continue;
        }

        if (length2 < smallest_line_segment_squared
            && height_2 <= allowed_error_distance_squared) // Removing the junction (vertex) doesn't introduce too much error.
        {
            const coord_t next_length2 = vSize2(current - next);
            if (next_length2 > 4 * smallest_line_segment_squared)
            {
                // Special case; The next line is long. If we were to remove this, it could happen that we get quite noticeable artifacts.
                // We should instead move this point to a location where both edges are kept and then remove the previous point that we wanted to keep.
                // By taking the intersection of these two lines, we get a point that preserves the direction (so it makes the corner a bit more pointy).
                // We just need to be sure that the intersection point does not introduce an artifact itself.
                Point intersection_point;
                bool has_intersection = LinearAlg2D::lineLineIntersection(previous_previous.p, previous.p, current.p, next.p, intersection_point);
                if (!has_intersection
                    || LinearAlg2D::getDist2FromLine(intersection_point, previous.p, current.p) > allowed_error_distance_squared
                    || vSize2(intersection_point - previous.p) > smallest_line_segment_squared  // The intersection point is way too far from the 'previous'
                    || vSize2(intersection_point - next.p) > smallest_line_segment_squared)     // and 'next' points, so it shouldn't replace 'current'
                {
                    // We can't find a better spot for it, but the size of the line is more than 5 micron.
                    // So the only thing we can do here is leave it in...
                }
                else
                {
                    // New point seems like a valid one.
                    const ExtrusionJunction new_to_add = ExtrusionJunction(intersection_point, current.w, current.perimeter_index);
                    // If there was a previous point added, remove it.
                    if(!new_junctions.empty())
                    {
                        new_junctions.pop_back();
                        previous = previous_previous;
                    }

                    // The junction (vertex) is replaced by the new one.
                    accumulated_area_removed = removed_area_next; // So that in the next iteration it's the area between the origin, [previous] and [current]
                    previous_previous = previous;
                    previous = new_to_add; // Note that "previous" is only updated if we don't remove the junction (vertex).
                    new_junctions.push_back(new_to_add);
                    continue;
                }
            }
            else
            {
                continue; // Remove the junction (vertex).
            }
        }
        // The junction (vertex) isn't removed.
        accumulated_area_removed = removed_area_next; // So that in the next iteration it's the area between the origin, [previous] and [current]
        previous_previous = previous;
        previous = current; // Note that "previous" is only updated if we don't remove the junction (vertex).
        new_junctions.push_back(current);
    }

    // Ending junction (vertex) should always exist in the simplified path
    new_junctions.emplace_back(junctions.back());*/

    /* In case this is a closed polygon (instead of a poly-line-segments), the invariant that the first and last points are the same should be enforced.
     * Since one of them didn't move, and the other can't have been moved further than the constraints, if originally equal, they can simply be equated.
     */
    /*if (vSize2(junctions.front().p - junctions.back().p) == 0)
    {
        new_junctions.back().p = junctions.front().p;
    }

    junctions = new_junctions;*/
}

coord_t ExtrusionLine::calculateExtrusionAreaDeviationError(ExtrusionJunction A, ExtrusionJunction B, ExtrusionJunction C, coord_t& weighted_average_width)
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
    const coord_t ab_length = vSize(B - A);
    const coord_t bc_length = vSize(C - B);
    const coord_t width_diff = std::max(std::abs(B.w - A.w), std::abs(C.w - B.w));
    if (width_diff > 1)
    {
        // Adjust the width only if there is a difference, or else the rounding errors may produce the wrong
        // weighted average value.
        const coord_t ab_weight = (A.w + B.w) / 2;
        const coord_t bc_weight = (B.w + C.w) / 2;
        weighted_average_width = (ab_length * ab_weight + bc_length * bc_weight) / vSize(C - A);
        return std::abs(ab_weight - weighted_average_width) * ab_length + std::abs(bc_weight - weighted_average_width) * bc_length;
    }
    else
    {
        // If the width difference is very small, then select the width of the segment that is longer
        weighted_average_width = ab_length > bc_length ? A.w : B.w;
        return ab_length > bc_length ? width_diff * bc_length : width_diff * ab_length;
    }
}

}
