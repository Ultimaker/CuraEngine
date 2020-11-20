//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtrusionLine.h"
#include "linearAlg2D.h"

namespace cura
{

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
    return len;
}

void ExtrusionLine::appendJunctionsTo(LineJunctions& result) const
{
    result.insert(result.end(), junctions.begin(), junctions.end());
}


void ExtrusionLine::simplify(const coord_t smallest_line_segment_squared, const coord_t allowed_error_distance_squared, const coord_t maximum_extrusion_area_deviation)
{
    if (junctions.size() <= 3)
    {
        return;
    }

    /* ExtrusionLines are treated as (open) polylines, so in case an ExtrusionLine is actually a closed polygon, its
     * starting and ending points will be equal (or almost equal). Therefore, the simplification of the ExtrusionLine
     * should not touch the first and last points. As a result, start simplifying from point at index 1.
     * */
    std::vector<ExtrusionJunction> new_junctions;
    // Starting junction should always exist in the simplified path
    new_junctions.emplace_back(junctions.front());

    /* Initially, previous_previous is always the same as previous because, for open ExtrusionLines the last junction
     * cannot be taken into consideration when checking the points at index 1. For closed ExtrusionLines, the first and
     * last junctions are anyway the same.
     * */
    ExtrusionJunction previous_previous = junctions.front();
    ExtrusionJunction previous = junctions.front();
    ExtrusionJunction current = junctions.at(1);
    ExtrusionJunction next = junctions.at(2);

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
    coord_t accumulated_area_removed = previous.p.X * current.p.Y - previous.p.Y * current.p.X; // Twice the Shoelace formula for area of polygon per line segment.

    for (size_t point_idx = 1; point_idx < junctions.size() - 1; point_idx++)
    {
        current = junctions.at(point_idx);

        // don't spill over if the [next] vertex will then be equal to [previous]
        if (point_idx + 1 == junctions.size() && new_junctions.size() > 1)
        {
            next = new_junctions[0]; //Spill over to new polygon for checking removed area.
        }
        else
        {
            next = junctions.at(point_idx + 1);
        }
        const coord_t removed_area_next = current.p.X * next.p.Y - current.p.Y * next.p.X; // Twice the Shoelace formula for area of polygon per line segment.
        const coord_t negative_area_closing = next.p.X * previous.p.Y - next.p.Y * previous.p.X; // area between the origin and the short-cutting segment
        accumulated_area_removed += removed_area_next;

        const coord_t length2 = vSize2(current - previous);
        if (length2 < 25)
        {
            // We're allowed to always delete segments of less than 5 micron. The width in this case doesn't matter that
            // much.
            continue;
        }

        const coord_t area_removed_so_far = accumulated_area_removed + negative_area_closing; // close the shortcut area polygon
        const coord_t base_length_2 = vSize2(next - previous);

        if (base_length_2 == 0) //Two line segments form a line back and forth with no area.
        {
            continue; //Remove the vertex.
        }
        //We want to check if the height of the triangle formed by previous, current and next vertices is less than allowed_error_distance_squared.
        //1/2 L = A           [actual area is half of the computed shoelace value] // Shoelace formula is .5*(...) , but we simplify the computation and take out the .5
        //A = 1/2 * b * h     [triangle area formula]
        //L = b * h           [apply above two and take out the 1/2]
        //h = L / b           [divide by b]
        //h^2 = (L / b)^2     [square it]
        //h^2 = L^2 / b^2     [factor the divisor]
        const coord_t height_2 = area_removed_so_far * area_removed_so_far / base_length_2;
        const coord_t extrusion_area_error = extrusionAreaDeviationError(previous, current, next);
        if ((height_2 <= 1 //Almost exactly colinear (barring rounding errors).
             && LinearAlg2D::getDistFromLine(current.p, previous.p, next.p) <= 1) // make sure that height_2 is not small because of cancellation of positive and negative areas
            // We shouldn't remove middle junctions of colinear segments if the area changed for the C-P segment is exceeding the maximum allowed
             && extrusion_area_error <= maximum_extrusion_area_deviation)
        {
            // Adjust the width of the entire P-N line as a weighted average of the widths of the P-C and C-N lines and
            // then remove the current junction.
            if (extrusion_area_error > 0)
            {
                next.w = (vSize(current - previous) * current.w + vSize(next - current) * next.w) / vSize(next - previous);
            }
            continue;
        }

        if (length2 < smallest_line_segment_squared
            && height_2 <= allowed_error_distance_squared) // removing the vertex doesn't introduce too much error.)
        {
            const coord_t next_length2 = vSize2(current - next);
            if (next_length2 > smallest_line_segment_squared)
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
                    current = ExtrusionJunction(intersection_point, current.w, current.perimeter_index, current.region_id);
                    // If there was a previous point added, remove it.
                    if(!new_junctions.empty())
                    {
                        new_junctions.pop_back();
                        previous = previous_previous;
                    }
                }
            }
            else
            {
                continue; //Remove the vertex.
            }
        }
        //Don't remove the vertex.
        accumulated_area_removed = removed_area_next; // so that in the next iteration it's the area between the origin, [previous] and [current]
        previous_previous = previous;
        previous = current; //Note that "previous" is only updated if we don't remove the vertex.
        new_junctions.push_back(current);
    }

    // Ending junction should always exist in the simplified path
    new_junctions.emplace_back(junctions.back());
    junctions = new_junctions;
}

coord_t ExtrusionLine::extrusionAreaDeviationError(ExtrusionJunction A, ExtrusionJunction B, ExtrusionJunction C)
{
    /*
     * A             B                          C              A                                        C
     * ---------------                                         ************** deviation_error / 2
     * |             |---------------------------  B removed   ------------------------------------------
     * |             |                          |  --------->  |            |                           |
     * |             |---------------------------              ------------------------------------------
     * ---------------             ^                           **************     ^
     *       ^                    C.w                                            C.w
     *      B.w
     * */
    return llabs(B.w - C.w) * vSize(B - A);
}

}
