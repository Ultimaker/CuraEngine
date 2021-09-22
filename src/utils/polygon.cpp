//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "polygon.h"

#include <numeric>
#include <unordered_set>

#include "linearAlg2D.h" // pointLiesOnTheRightOfLine

#include "ListPolyIt.h"

namespace cura
{

size_t ConstPolygonRef::size() const
{
    return path->size();
}

bool ConstPolygonRef::empty() const
{
    return path->empty();
}

bool ConstPolygonRef::shorterThan(const coord_t check_length) const
{
    return cura::shorterThan(*this, check_length);
}

bool ConstPolygonRef::_inside(Point p, bool border_result) const
{
    const ConstPolygonRef thiss = *this;
    if (size() < 1)
    {
        return false;
    }

    int crossings = 0;
    Point p0 = back();
    for(unsigned int n=0; n<size(); n++)
    {
        Point p1 = thiss[n];
        // no tests unless the segment p0-p1 is at least partly at, or to right of, p.X
        short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
        if (comp == 1)
        {
            crossings++;
        }
        else if (comp == 0)
        {
            return border_result;
        }
        p0 = p1;
    }
    return (crossings % 2) == 1;
}


Polygons ConstPolygonRef::intersection(const ConstPolygonRef& other) const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPath(*path, ClipperLib::ptSubject, true);
    clipper.AddPath(*other.path, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, ret.paths);
    return ret;
}

bool Polygons::empty() const
{
    return paths.empty();
}

Polygons Polygons::approxConvexHull(int extra_outset)
{
    constexpr int overshoot = MM2INT(100); //10cm (hard-coded value).

    Polygons convex_hull;
    //Perform the offset for each polygon one at a time.
    //This is necessary because the polygons may overlap, in which case the offset could end up in an infinite loop.
    //See http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/_Body.htm
    for (const ClipperLib::Path& path : paths)
    {
        Polygons offset_result;
        ClipperLib::ClipperOffset offsetter(1.2, 10.0);
        offsetter.AddPath(path, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
        offsetter.Execute(offset_result.paths, overshoot);
        convex_hull.add(offset_result);
    }
    return convex_hull.unionPolygons().offset(-overshoot + extra_outset, ClipperLib::jtRound);
}

void Polygons::makeConvex()
{
    for (PolygonRef poly : *this)
    {
        Polygon convexified;
        Point a = poly.back();
        for (size_t i = 0; i < poly.size(); ++i)
        {
            const Point& b = poly[i];
            const Point& c = poly[(i + 1) % poly.size()];
            if (LinearAlg2D::pointIsLeftOfLine(b, a, c) < 0)
            {
                convexified.path->push_back(b);
                a = b;
            }
        }
        poly.path->swap(*convexified.path); //Due to vector's implementation, this is constant time.
    }
}

unsigned int Polygons::pointCount() const
{
    unsigned int count = 0;
    for (const ClipperLib::Path& path : paths)
    {
        count += path.size();
    }
    return count;
}

bool Polygons::inside(Point p, bool border_result) const
{
    int poly_count_inside = 0;
    for (const ClipperLib::Path& poly : *this)
    {
        const int is_inside_this_poly = ClipperLib::PointInPolygon(p, poly);
        if (is_inside_this_poly == -1)
        {
            return border_result;
        }
        poly_count_inside += is_inside_this_poly;
    }
    return (poly_count_inside % 2) == 1;
}

bool PolygonsPart::inside(Point p, bool border_result) const
{
    if (size() < 1)
    {
        return false;
    }
    if (!(*this)[0].inside(p, border_result))
    {
        return false;
    }
    for(unsigned int n = 1; n < paths.size(); n++)
    {
        if ((*this)[n].inside(p, border_result))
        {
            return false;
        }
    }
    return true;
}

bool Polygons::insideOld(Point p, bool border_result) const
{
    const Polygons& thiss = *this;
    if (size() < 1)
    {
        return false;
    }

    int crossings = 0;
    for (const ClipperLib::Path& poly : thiss)
    {
        Point p0 = poly.back();
        for (const Point& p1 : poly)
        {
            short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
            if (comp == 1)
            {
                crossings++;
            }
            else if (comp == 0)
            {
                return border_result;
            }
            p0 = p1;
        }
    }
    return (crossings % 2) == 1;
}

unsigned int Polygons::findInside(Point p, bool border_result)
{
    Polygons& thiss = *this;
    if (size() < 1)
    {
        return false;
    }

    // NOTE: Keep these vectors fixed-size, they replace an (non-standard, sized at runtime) arrays.
    std::vector<int64_t> min_x(size(), std::numeric_limits<int64_t>::max());
    std::vector<int64_t> crossings(size());

    for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
    {
        PolygonRef poly = thiss[poly_idx];
        Point p0 = poly.back();
        for(Point& p1 : poly)
        {
            short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
            if (comp == 1)
            {
                crossings[poly_idx]++;
                int64_t x;
                if (p1.Y == p0.Y)
                {
                    x = p0.X;
                }
                else
                {
                    x = p0.X + (p1.X-p0.X) * (p.Y-p0.Y) / (p1.Y-p0.Y);
                }
                if (x < min_x[poly_idx])
                {
                    min_x[poly_idx] = x;
                }
            }
            else if (border_result && comp == 0)
            {
                return poly_idx;
            }
            p0 = p1;
        }
    }

    int64_t min_x_uneven = std::numeric_limits<int64_t>::max();
    unsigned int ret = NO_INDEX;
    unsigned int n_unevens = 0;
    for (unsigned int array_idx = 0; array_idx < size(); array_idx++)
    {
        if (crossings[array_idx] % 2 == 1)
        {
            n_unevens++;
            if (min_x[array_idx] < min_x_uneven)
            {
                min_x_uneven = min_x[array_idx];
                ret = array_idx;
            }
        }
    }
    if (n_unevens % 2 == 0) { ret = NO_INDEX; }
    return ret;
}

Polygons Polygons::intersectionPolyLines(const Polygons& polylines) const
{
    ClipperLib::PolyTree result;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPaths(polylines.paths, ClipperLib::ptSubject, false);
    clipper.AddPaths(paths, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, result);
    Polygons ret;
    ret.addPolyTreeNodeRecursive(result);
    return ret;
}

Polygons& Polygons::cut(const Polygons& tool)
{
    ClipperLib::PolyTree interior_segments_tree;
    tool.lineSegmentIntersection(*this, interior_segments_tree);
    ClipperLib::Paths interior_segments;
    ClipperLib::OpenPathsFromPolyTree(interior_segments_tree, interior_segments);
    this->clear();
    for (const std::vector<ClipperLib::IntPoint>& interior_segment : interior_segments)
    {
        this->addLine(interior_segment[0], interior_segment[1]);
    }
    return *this;
}

coord_t Polygons::polyLineLength() const
{
    coord_t length = 0;
    for (unsigned int poly_idx = 0; poly_idx < paths.size(); poly_idx++)
    {
        Point p0 = paths[poly_idx][0];
        for (unsigned int point_idx = 1; point_idx < paths[poly_idx].size(); point_idx++)
        {
            Point p1 = paths[poly_idx][point_idx];
            length += vSize(p0 - p1);
            p0 = p1;
        }
    }
    return length;
}

Polygons Polygons::offset(int distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (distance == 0)
    {
        return *this;
    }
    Polygons ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    clipper.AddPaths(unionPolygons().paths, join_type, ClipperLib::etClosedPolygon);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret.paths, distance);
    return ret;
}

Polygons ConstPolygonRef::offset(int distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (distance == 0)
    {
        Polygons ret;
        ret.add(*this);
        return ret;
    }
    Polygons ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    clipper.AddPath(*path, join_type, ClipperLib::etClosedPolygon);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret.paths, distance);
    return ret;
}

void PolygonRef::removeColinearEdges(const AngleRadians max_deviation_angle)
{
    // TODO: Can be made more efficient (for example, use pointer-types for process-/skip-indices, so we can swap them without copy).

    size_t num_removed_in_iteration = 0;
    do
    {
        num_removed_in_iteration = 0;

        std::vector<bool> process_indices(path->size(), true);

        bool go = true;
        while (go)
        {
            go = false;

            const auto& rpath = *path;
            const size_t pathlen = rpath.size();
            if (pathlen <= 3)
            {
                return;
            }

            std::vector<bool> skip_indices(path->size(), false);

            ClipperLib::Path new_path;
            for (size_t point_idx = 0; point_idx < pathlen; ++point_idx)
            {
                // Don't iterate directly over process-indices, but do it this way, because there are points _in_ process-indices that should nonetheless be skipped:
                if (! process_indices[point_idx])
                {
                    new_path.push_back(rpath[point_idx]);
                    continue;
                }

                // Should skip the last point for this iteration if the old first was removed (which can be seen from the fact that the new first was skipped):
                if (point_idx == (pathlen - 1) && skip_indices[0])
                {
                    skip_indices[new_path.size()] = true;
                    go = true;
                    new_path.push_back(rpath[point_idx]);
                    break;
                }

                const Point& prev = rpath[(point_idx - 1 + pathlen) % pathlen];
                const Point& pt = rpath[point_idx];
                const Point& next = rpath[(point_idx + 1) % pathlen];

                float angle = LinearAlg2D::getAngleLeft(prev, pt, next);  // [0 : 2 * pi]
                if (angle >= M_PI) {angle -= M_PI;}  // map [pi : 2 * pi] to [0 : pi]

                // Check if the angle is within limits for the point to 'make sense', given the maximum deviation.
                // If the angle indicates near-parallel segments ignore the point 'pt'
                if (angle > max_deviation_angle && angle < M_PI - max_deviation_angle)
                {
                    new_path.push_back(pt);
                }
                else if (point_idx != (pathlen - 1))
                {
                    // Skip the next point, since the current one was removed:
                    skip_indices[new_path.size()] = true;
                    go = true;
                    new_path.push_back(next);
                    ++point_idx;
                }
            }
            *path = new_path;
            num_removed_in_iteration += pathlen - path->size();

            process_indices.clear();
            process_indices.insert(process_indices.end(), skip_indices.begin(), skip_indices.end());
        }
    }
    while (num_removed_in_iteration > 0);
}

void PolygonRef::simplify(const coord_t smallest_line_segment_squared, const coord_t allowed_error_distance_squared)
{
    if (size() < 3)
    {
        clear();
        return;
    }
    if (size() == 3)
    {
        return;
    }

    ClipperLib::Path new_path;
    Point previous = path->back();
    Point previous_previous = path->at(path->size() - 2);
    Point current = path->at(0);

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
    coord_t accumulated_area_removed = previous.X * current.Y - previous.Y * current.X; // Twice the Shoelace formula for area of polygon per line segment.

    for (size_t point_idx = 0; point_idx < size(); point_idx++)
    {
        current = path->at(point_idx % size());

        //Check if the accumulated area doesn't exceed the maximum.
        Point next;
        if (point_idx + 1 < size())
        {
            next = path->at(point_idx + 1);
        }
        else if (point_idx + 1 == size() && new_path.size() > 1)
        { // don't spill over if the [next] vertex will then be equal to [previous]
            next = new_path[0]; //Spill over to new polygon for checking removed area.
        }
        else
        {
            next = path->at((point_idx + 1) % size());
        }
        const coord_t removed_area_next = current.X * next.Y - current.Y * next.X; // Twice the Shoelace formula for area of polygon per line segment.
        const coord_t negative_area_closing = next.X * previous.Y - next.Y * previous.X; // area between the origin and the short-cutting segment
        accumulated_area_removed += removed_area_next;

        const coord_t length2 = vSize2(current - previous);
        if (length2 < 25)
        {
            // We're allowed to always delete segments of less than 5 micron.
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
        if ((height_2 <= 5 * 5 //Almost exactly colinear (barring rounding errors).
            && LinearAlg2D::getDistFromLine(current, previous, next) <= 5)) // make sure that height_2 is not small because of cancellation of positive and negative areas
        {
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
                bool has_intersection = LinearAlg2D::lineLineIntersection(previous_previous, previous, current, next, intersection_point);
                if (!has_intersection
                    || LinearAlg2D::getDist2FromLine(intersection_point, previous, current) > allowed_error_distance_squared
                    || vSize2(intersection_point - previous) > smallest_line_segment_squared  // The intersection point is way too far from the 'previous'
                    || vSize2(intersection_point - next) > smallest_line_segment_squared)     // and 'next' points, so it shouldn't replace 'current'
                {
                    // We can't find a better spot for it, but the size of the line is more than 5 micron.
                    // So the only thing we can do here is leave it in...
                }
                else
                {
                    // New point seems like a valid one.
                    current = intersection_point;
                    // If there was a previous point added, remove it.
                    if(!new_path.empty())
                    {
                        new_path.pop_back();
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
        new_path.push_back(current);
    }

    *path = new_path;
}

void PolygonRef::applyMatrix(const PointMatrix& matrix)
{
    for (unsigned int path_idx = 0; path_idx < path->size(); path_idx++)
    {
        (*path)[path_idx] = matrix.apply((*path)[path_idx]);
    }
}
void PolygonRef::applyMatrix(const Point3Matrix& matrix)
{
    for (unsigned int path_idx = 0; path_idx < path->size(); path_idx++)
    {
        (*path)[path_idx] = matrix.apply((*path)[path_idx]);
    }
}

Polygons Polygons::getOutsidePolygons() const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree poly_tree;
    constexpr bool paths_are_closed_polys = true;
    clipper.AddPaths(paths, ClipperLib::ptSubject, paths_are_closed_polys);
    clipper.Execute(ClipperLib::ctUnion, poly_tree);

    for (int outer_poly_idx = 0; outer_poly_idx < poly_tree.ChildCount(); outer_poly_idx++)
    {
        ClipperLib::PolyNode* child = poly_tree.Childs[outer_poly_idx];
        ret.emplace_back(child->Contour);
    }
    return ret;
}

Polygons Polygons::removeEmptyHoles() const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree poly_tree;
    constexpr bool paths_are_closed_polys = true;
    clipper.AddPaths(paths, ClipperLib::ptSubject, paths_are_closed_polys);
    clipper.Execute(ClipperLib::ctUnion, poly_tree);

    bool remove_holes = true;
    removeEmptyHoles_processPolyTreeNode(poly_tree, remove_holes, ret);
    return ret;
}

Polygons Polygons::getEmptyHoles() const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree poly_tree;
    constexpr bool paths_are_closed_polys = true;
    clipper.AddPaths(paths, ClipperLib::ptSubject, paths_are_closed_polys);
    clipper.Execute(ClipperLib::ctUnion, poly_tree);

    bool remove_holes = false;
    removeEmptyHoles_processPolyTreeNode(poly_tree, remove_holes, ret);
    return ret;
}

void Polygons::removeEmptyHoles_processPolyTreeNode(const ClipperLib::PolyNode& node, const bool remove_holes, Polygons& ret) const
{
    for (int outer_poly_idx = 0; outer_poly_idx < node.ChildCount(); outer_poly_idx++)
    {
        ClipperLib::PolyNode* child = node.Childs[outer_poly_idx];
        if (remove_holes)
        {
            ret.emplace_back(child->Contour);
        }
        for (int hole_node_idx = 0; hole_node_idx < child->ChildCount(); hole_node_idx++)
        {
            ClipperLib::PolyNode& hole_node = *child->Childs[hole_node_idx];
            if ((hole_node.ChildCount() > 0) == remove_holes)
            {
                ret.emplace_back(hole_node.Contour);
                removeEmptyHoles_processPolyTreeNode(hole_node, remove_holes, ret);
            }
        }
    }
}

void Polygons::removeSmallAreas(const double min_area_size, const bool remove_holes)
{
    auto new_end = paths.end();
    if(remove_holes)
    {
        for(auto it = paths.begin(); it < new_end; it++)
        {
            // All polygons smaller than target are removed by replacing them with a polygon from the back of the vector
            if(fabs(INT2MM2(ClipperLib::Area(*it))) < min_area_size)
            {
                new_end--;
                *it = std::move(*new_end);
                it--; // wind back the iterator such that the polygon just swaped in is checked next
            }
        }
    }
    else
    {
        // For each polygon, computes the signed area, move small outlines at the end of the vector and keep references on small holes
        std::vector<PolygonRef> small_holes;
        for(auto it = paths.begin(); it < new_end; it++) {
            double area = INT2MM2(ClipperLib::Area(*it));
            if (fabs(area) < min_area_size)
            {
                if(area >= 0)
                {
                    new_end--;
                    if(it < new_end) {
                        std::swap(*new_end, *it);
                        it--;
                    }
                    else
                    { // Don't self-swap the last Path
                        break;
                    }
                }
                else
                {
                    small_holes.push_back(*it);
                }
            }
        }

        // Removes small holes that have their first point inside one of the removed outlines
        // Iterating in reverse ensures that unprocessed small holes won't be moved
        const auto removed_outlines_start = new_end;
        for(auto hole_it = small_holes.rbegin(); hole_it < small_holes.rend(); hole_it++)
        {
            for(auto outline_it = removed_outlines_start; outline_it < paths.end() ; outline_it++)
            {
                if(PolygonRef(*outline_it).inside(*hole_it->begin())) {
                    new_end--;
                    **hole_it = std::move(*new_end);
                    break;
                }
            }
        }
    }
    paths.resize(new_end-paths.begin());
}

Polygons Polygons::toPolygons(ClipperLib::PolyTree& poly_tree)
{
    Polygons ret;
    ret.addPolyTreeNodeRecursive(poly_tree);
    return ret;
}


void Polygons::addPolyTreeNodeRecursive(const ClipperLib::PolyNode& node)
{
    for (int outer_poly_idx = 0; outer_poly_idx < node.ChildCount(); outer_poly_idx++)
    {
        ClipperLib::PolyNode* child = node.Childs[outer_poly_idx];
        paths.push_back(child->Contour);
        addPolyTreeNodeRecursive(*child);
    }
}

bool ConstPolygonRef::smooth_corner_complex(const Point p1, ListPolyIt& p0_it, ListPolyIt& p2_it, const int64_t shortcut_length)
{
    // walk away from the corner until the shortcut > shortcut_length or it would smooth a piece inward
    // - walk in both directions untill shortcut > shortcut_length
    // - stop walking in one direction if it would otherwise cut off a corner in that direction
    // - same in the other direction
    // - stop if both are cut off
    // walk by updating p0_it and p2_it
    int64_t shortcut_length2 = shortcut_length * shortcut_length;
    bool forward_is_blocked = false;
    bool forward_is_too_far = false;
    bool backward_is_blocked = false;
    bool backward_is_too_far = false;
    while (true)
    {
        const bool forward_has_converged = forward_is_blocked || forward_is_too_far;
        const bool backward_has_converged = backward_is_blocked || backward_is_too_far;
        if (forward_has_converged && backward_has_converged)
        {
            if (forward_is_too_far && backward_is_too_far && vSize2(p0_it.prev().p() - p2_it.next().p()) < shortcut_length2)
            {
                //         o
                //       /   \                                                  .
                //      o     o
                //      |     |
                //      \     /                                                 .
                //       |   |
                //       \   /                                                  .
                //        | |
                //        o o
                --p0_it;
                ++p2_it;
                forward_is_too_far = false; // invalidate data
                backward_is_too_far = false; // invalidate data
                continue;
            }
            else
            {
                break;
            }
        }
        smooth_outward_step(p1, shortcut_length2, p0_it, p2_it, forward_is_blocked, backward_is_blocked, forward_is_too_far, backward_is_too_far);
        if (p0_it.prev() == p2_it || p0_it == p2_it)
        { // stop if we went all the way around the polygon
            // this should only be the case for hole polygons (?)
            if (forward_is_too_far && backward_is_too_far)
            {
                // in case p0_it.prev() == p2_it :
                //     /                                                .
                //    /                /|
                //   |       becomes  | |
                //    \                \|
                //     \                                                .
                // in case p0_it == p2_it :
                //     /                                                .
                //    /    becomes     /|
                //    \                \|
                //     \                                                .
                break;
            }
            else
            {
                // this whole polygon can be removed
                return true;
            }
        }
    }

    const Point v02 = p2_it.p() - p0_it.p();
    const int64_t v02_size2 = vSize2(v02);
    // set the following:
    // p0_it = start point of line
    // p2_it = end point of line
    if (std::abs(v02_size2 - shortcut_length2) < shortcut_length * 10) // i.e. if (size2 < l * (l+10) && size2 > l * (l-10))
    { // v02 is approximately shortcut length
        // handle this separately to avoid rounding problems below in the getPointOnLineWithDist function
        // p0_it and p2_it are already correct
    }
    else if (!backward_is_blocked && !forward_is_blocked)
    { // introduce two new points
        //  1----b---->2
        //  ^   /
        //  |  /
        //  | /
        //  |/
        //  |a
        //  |
        //  0
        const int64_t v02_size = sqrt(v02_size2);

        const ListPolyIt p0_2_it = p0_it.prev();
        const ListPolyIt p2_2_it = p2_it.next();
        const Point p2_2 = p2_2_it.p();
        const Point p0_2 = p0_2_it.p();
        const Point v02_2 = p0_2 - p2_2;
        const int64_t v02_2_size = vSize(v02_2);
        float progress = std::min(1.0, INT2MM(shortcut_length - v02_size) / INT2MM(v02_2_size - v02_size)); // account for rounding error when v02_2_size is approx equal to v02_size
        assert(progress >= 0.0f && progress <= 1.0f && "shortcut length must be between last length and new length");
        const Point new_p0 = p0_it.p() + (p0_2 - p0_it.p()) * progress;
        p0_it = ListPolyIt::insertPointNonDuplicate(p0_2_it, p0_it, new_p0);
        const Point new_p2 = p2_it.p() + (p2_2 - p2_it.p()) * progress;
        p2_it = ListPolyIt::insertPointNonDuplicate(p2_it, p2_2_it, new_p2);
    }
    else if (!backward_is_blocked)
    { // forward is blocked, back is open
        //     |
        //  1->b
        //  ^  :
        //  | /
        //  0 :
        //  |/
        //  |a
        //  |
        //  0_2
        const ListPolyIt p0_2_it = p0_it.prev();
        const Point p0 = p0_it.p();
        const Point p0_2 = p0_2_it.p();
        const Point p2 = p2_it.p();
        Point new_p0;
        bool success = LinearAlg2D::getPointOnLineWithDist(p2, p0, p0_2, shortcut_length, new_p0);
        // shortcut length must be possible given that last length was ok and new length is too long
        if (success)
        {
#ifdef ASSERT_INSANE_OUTPUT
            assert(new_p0.X < 400000 && new_p0.Y < 400000);
#endif // #ifdef ASSERT_INSANE_OUTPUT
            p0_it = ListPolyIt::insertPointNonDuplicate(p0_2_it, p0_it, new_p0);
        }
        else
        { // if not then a rounding error occured
            if (vSize(p2 - p0_2) < vSize2(p2 - p0))
            {
                p0_it = p0_2_it; // start shortcut at 0
            }
        }
    }
    else if (!forward_is_blocked)
    { // backward is blocked, front is open
        //  1----2----b----------->2_2
        //  ^      ,-'
        //  |   ,-'
        //--0.-'
        //  a
        const ListPolyIt p2_2_it = p2_it.next();
        const Point p0 = p0_it.p();
        const Point p2 = p2_it.p();
        const Point p2_2 = p2_2_it.p();
        Point new_p2;
        bool success = LinearAlg2D::getPointOnLineWithDist(p0, p2, p2_2, shortcut_length, new_p2);
        // shortcut length must be possible given that last length was ok and new length is too long
        if (success)
        {
#ifdef ASSERT_INSANE_OUTPUT
            assert(new_p2.X < 400000 && new_p2.Y < 400000);
#endif // #ifdef ASSERT_INSANE_OUTPUT
            p2_it = ListPolyIt::insertPointNonDuplicate(p2_it, p2_2_it, new_p2);
        }
        else
        { // if not then a rounding error occured
            if (vSize(p2_2 - p0) < vSize2(p2 - p0))
            {
                p2_it = p2_2_it; // start shortcut at 0
            }
        }
    }
    else
    {
        //        |
        //      __|2
        //     | /  > shortcut cannot be of the desired length
        //  ___|/                                                       .
        //     0
        // both are blocked and p0_it and p2_it are already correct
    }
    // delete all cut off points
    while (p0_it.next() != p2_it)
    {
        p0_it.next().remove();
    }
    return false;
}

void ConstPolygonRef::smooth_outward_step(const Point p1, const int64_t shortcut_length2, ListPolyIt& p0_it, ListPolyIt& p2_it, bool& forward_is_blocked, bool& backward_is_blocked, bool& forward_is_too_far, bool& backward_is_too_far)
{
    const bool forward_has_converged = forward_is_blocked || forward_is_too_far;
    const bool backward_has_converged = backward_is_blocked || backward_is_too_far;
    const Point p0 = p0_it.p();
    const Point p2 = p2_it.p();
    bool walk_forward = !forward_has_converged && (backward_has_converged || (vSize2(p2 - p1) < vSize2(p0 - p1))); // whether to walk along the p1-p2 direction or in the p1-p0 direction

    if (walk_forward)
    {
        const ListPolyIt p2_2_it = p2_it.next();
        const Point p2_2 = p2_2_it.p();
        bool p2_is_left = LinearAlg2D::pointIsLeftOfLine(p2, p0, p2_2) >= 0;
        if (!p2_is_left)
        {
            forward_is_blocked = true;
            return;
        }

        const Point v02_2 = p2_2 - p0_it.p();
        if (vSize2(v02_2) > shortcut_length2)
        {
            forward_is_too_far = true;
            return;
        }

        p2_it = p2_2_it; // make one step in the forward direction
        backward_is_blocked = false; // invalidate data about backward walking
        backward_is_too_far = false;
        return;
    }
    else
    {
        const ListPolyIt p0_2_it = p0_it.prev();
        const Point p0_2 = p0_2_it.p();
        bool p0_is_left = LinearAlg2D::pointIsLeftOfLine(p0, p0_2, p2) >= 0;
        if (!p0_is_left)
        {
            backward_is_blocked = true;
            return;
        }

        const Point v02_2 = p2_it.p() - p0_2;
        if (vSize2(v02_2) > shortcut_length2)
        {
            backward_is_too_far = true;
            return;
        }

        p0_it = p0_2_it; // make one step in the backward direction
        forward_is_blocked = false; // invalidate data about forward walking
        forward_is_too_far = false;
        return;
    }
}

void ConstPolygonRef::smooth_corner_simple(const Point p0, const Point p1, const Point p2, const ListPolyIt p0_it, const ListPolyIt p1_it, const ListPolyIt p2_it, const Point v10, const Point v12, const Point v02, const int64_t shortcut_length, float cos_angle)
{
    //  1----b---->2
    //  ^   /
    //  |  /
    //  | /
    //  |/
    //  |a
    //  |
    //  0
    // ideally a1_size == b1_size
    if (vSize2(v02) <= shortcut_length * (shortcut_length + 10) // v02 is approximately shortcut length
        || (cos_angle > 0.9999 && LinearAlg2D::getDist2FromLine(p2, p0, p1) < 20 * 20)) // p1 is degenerate
    {
        // handle this separately to avoid rounding problems below in the getPointOnLineWithDist function
        p1_it.remove();
        // don't insert new elements
    }
    else
    {
        // compute the distance a1 == b1 to get vSize(ab)==shortcut_length with the given angle between v10 and v12
        //       1
        //      /|\                                                      .
        //     / | \                                                     .
        //    /  |  \                                                    .
        //   /   |   \                                                   .
        // a/____|____\b                                                 .
        //       m
        // use trigonometry on the right-angled triangle am1
        double a1m_angle = acos(cos_angle) / 2;
        const int64_t a1_size = shortcut_length / 2 / sin(a1m_angle);
        if (a1_size * a1_size < vSize2(v10) && a1_size * a1_size < vSize2(v12))
        {
            Point a = p1 + normal(v10, a1_size);
            Point b = p1 + normal(v12, a1_size);
#ifdef ASSERT_INSANE_OUTPUT
            assert(vSize(a) < 4000000);
            assert(vSize(b) < 4000000);
#endif // #ifdef ASSERT_INSANE_OUTPUT
            ListPolyIt::insertPointNonDuplicate(p0_it, p1_it, a);
            ListPolyIt::insertPointNonDuplicate(p1_it, p2_it, b);
            p1_it.remove();
        }
        else if (vSize2(v12) < vSize2(v10))
        {
            //     b
            //  1->2
            //  ^  |
            //  | /
            //  | |
            //  |/
            //  |a
            //  |
            //  0
            const Point& b = p2_it.p();
            Point a;
            bool success = LinearAlg2D::getPointOnLineWithDist(b, p1, p0, shortcut_length, a);
            // v02 has to be longer than ab!
            if (success)
            { // if not success then assume a is negligibly close to 0, but rounding errors caused a problem
#ifdef ASSERT_INSANE_OUTPUT
                assert(vSize(a) < 4000000);
#endif // #ifdef ASSERT_INSANE_OUTPUT
                ListPolyIt::insertPointNonDuplicate(p0_it, p1_it, a);
            }
            p1_it.remove();
        }
        else
        {
            //  1---------b----------->2
            //  ^      ,-'
            //  |   ,-'
            //  0.-'
            //  a
            const Point& a = p0_it.p();
            Point b;
            bool success = LinearAlg2D::getPointOnLineWithDist(a, p1, p2, shortcut_length, b);
            // v02 has to be longer than ab!
            if (success)
            { // if not success then assume b is negligibly close to 2, but rounding errors caused a problem
#ifdef ASSERT_INSANE_OUTPUT
                assert(vSize(b) < 4000000);
#endif // #ifdef ASSERT_INSANE_OUTPUT
                ListPolyIt::insertPointNonDuplicate(p1_it, p2_it, b);
            }
            p1_it.remove();
        }
    }
}

void ConstPolygonRef::smooth_outward(const AngleDegrees min_angle, int shortcut_length, PolygonRef result) const
{
// example of smoothed out corner:
//
//               6
//               ^
//               |
// inside        |     outside
//         2>3>4>5
//         ^    /                   .
//         |   /                    .
//         1  /                     .
//         ^ /                      .
//         |/                       .
//         |
//         |
//         0

    int shortcut_length2 = shortcut_length * shortcut_length;
    float cos_min_angle = cos(min_angle / 180 * M_PI);

    ListPolygon poly;
    ListPolyIt::convertPolygonToList(*this, poly);

    { // remove duplicate vertices
        ListPolyIt p1_it(poly, poly.begin());
        do
        {
            ListPolyIt next = p1_it.next();
            if (vSize2(p1_it.p() - next.p()) < 10 * 10)
            {
                p1_it.remove();
            }
            p1_it = next;
        } while (p1_it != ListPolyIt(poly, poly.begin()));
    }

    ListPolyIt p1_it(poly, poly.begin());
    do
    {
        const Point p1 = p1_it.p();
        ListPolyIt p0_it = p1_it.prev();
        ListPolyIt p2_it = p1_it.next();
        const Point p0 = p0_it.p();
        const Point p2 = p2_it.p();

        const Point v10 = p0 - p1;
        const Point v12 = p2 - p1;
        float cos_angle = INT2MM(INT2MM(dot(v10, v12))) / vSizeMM(v10) / vSizeMM(v12);
        bool is_left_angle = LinearAlg2D::pointIsLeftOfLine(p1, p0, p2) > 0;
        if (cos_angle > cos_min_angle && is_left_angle)
        {
            // angle is so sharp that it can be removed
            Point v02 = p2_it.p() - p0_it.p();
            if (vSize2(v02) >= shortcut_length2)
            {
                smooth_corner_simple(p0, p1, p2, p0_it, p1_it, p2_it, v10, v12, v02, shortcut_length, cos_angle);
            }
            else
            {
                bool remove_poly = smooth_corner_complex(p1, p0_it, p2_it, shortcut_length); // edits p0_it and p2_it!
                if (remove_poly)
                {
                    // don't convert ListPolygon into result
                    return;
                }
            }
            // update:
            p1_it = p2_it; // next point to consider for whether it's an internal corner
        }
        else
        {
            ++p1_it;
        }
    } while (p1_it != ListPolyIt(poly, poly.begin()));

    ListPolyIt::convertListPolygonToPolygon(poly, result);
}

Polygons Polygons::smooth_outward(const AngleDegrees max_angle, int shortcut_length)
{
    Polygons ret;
    for (unsigned int p = 0; p < size(); p++)
    {
        PolygonRef poly(paths[p]);
        if (poly.size() < 3)
        {
            continue;
        }
        if (poly.size() == 3)
        {
            ret.add(poly);
            continue;
        }
        poly.smooth_outward(max_angle, shortcut_length, ret.newPoly());
        if (ret.back().size() < 3)
        {
            ret.paths.resize(ret.paths.size() - 1);
        }
    }
    return ret;
}


void ConstPolygonRef::smooth(int remove_length, PolygonRef result) const
{
// a typical zigzag with the middle part to be removed by removing (1) :
//
//               3
//               ^
//               |
//               |
// inside        |     outside
//          1--->2
//          ^
//          |
//          |
//          |
//          0
    const ConstPolygonRef& thiss = *path;
    ClipperLib::Path* poly = result.path;
    if (size() > 0)
    {
        poly->push_back(thiss[0]);
    }
    auto is_zigzag = [remove_length](const int64_t v02_size, const int64_t v12_size, const int64_t v13_size, const int64_t dot1, const int64_t dot2)
    {
        if (v12_size > remove_length)
        { // v12 or v13 is too long
            return false;
        }
        const bool p1_is_left_of_v02 = dot1 < 0;
        if (!p1_is_left_of_v02)
        { // removing p1 wouldn't smooth outward
            return false;
        }
        const bool p2_is_left_of_v13 = dot2 > 0;
        if (p2_is_left_of_v13)
        { // l0123 doesn't constitute a zigzag ''|,,
            return false;
        }
        if (-dot1 <= v02_size * v12_size / 2)
        { // angle at p1 isn't sharp enough
            return false;
        }
        if (-dot2 <= v13_size * v12_size / 2)
        { // angle at p2 isn't sharp enough
            return false;
        }
        return true;
    };
    Point v02 = thiss[2] - thiss[0];
    Point v02T = turn90CCW(v02);
    int64_t v02_size = vSize(v02);
    bool force_push = false;
    for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
    {
        const Point& p1 = thiss[poly_idx];
        const Point& p2 = thiss[(poly_idx + 1) % size()];
        const Point& p3 = thiss[(poly_idx + 2) % size()];
        // v02 computed in last iteration
        // v02_size as well
        const Point v12 = p2 - p1;
        const int64_t v12_size = vSize(v12);
        const Point v13 = p3 - p1;
        const int64_t v13_size = vSize(v13);

        // v02T computed in last iteration
        const int64_t dot1 = dot(v02T, v12);
        const Point v13T = turn90CCW(v13);
        const int64_t dot2 = dot(v13T, v12);
        bool push_point = force_push || !is_zigzag(v02_size, v12_size, v13_size, dot1, dot2);
        force_push = false;
        if (push_point)
        {
            poly->push_back(p1);
        }
        else
        {
            // do not add the current one to the result
            force_push = true; // ensure the next point is added; it cannot also be a zigzag
        }
        v02T = v13T;
        v02 = v13;
        v02_size = v13_size;
    }
}

Polygons Polygons::smooth(int remove_length) const
{
    Polygons ret;
    for (unsigned int p = 0; p < size(); p++)
    {
        ConstPolygonRef poly(paths[p]);
        if (poly.size() < 3)
        {
            continue;
        }
        if (poly.size() == 3)
        {
            ret.add(poly);
            continue;
        }
        poly.smooth(remove_length, ret.newPoly());
        PolygonRef back = ret.back();
        if (back.size() < 3)
        {
            back.path->resize(back.path->size() - 1);
        }
    }
    return ret;
}

void ConstPolygonRef::smooth2(int remove_length, PolygonRef result) const
{
    const ConstPolygonRef& thiss = *this;
    ClipperLib::Path* poly = result.path;
    if (thiss.size() > 0)
    {
        poly->push_back(thiss[0]);
    }
    for (unsigned int poly_idx = 1; poly_idx < thiss.size(); poly_idx++)
    {
        const Point& last = thiss[poly_idx - 1];
        const Point& now = thiss[poly_idx];
        const Point& next = thiss[(poly_idx + 1) % thiss.size()];
        if (shorterThen(last - now, remove_length) && shorterThen(now - next, remove_length))
        {
            poly_idx++; // skip the next line piece (dont escalate the removal of edges)
            if (poly_idx < thiss.size())
            {
                poly->push_back(thiss[poly_idx]);
            }
        }
        else
        {
            poly->push_back(thiss[poly_idx]);
        }
    }
}

Polygons Polygons::smooth2(int remove_length, int min_area) const
{
    Polygons ret;
    for (unsigned int p = 0; p < size(); p++)
    {
        ConstPolygonRef poly(paths[p]);
        if (poly.size() == 0)
        {
            continue;
        }
        if (poly.area() < min_area || poly.size() <= 5) // when optimally removing, a poly with 5 pieces results in a triangle. Smaller polys dont have area!
        {
            ret.add(poly);
            continue;
        }
        if (poly.size() < 4)
        {
            ret.add(poly);
        }
        else
        {
            poly.smooth2(remove_length, ret.newPoly());
        }
    }
    return ret;
}

double Polygons::area() const
{
    double area = 0.0;
    for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
    {
        area += operator[](poly_idx).area();
        // note: holes already have negative area
    }
    return area;
}

std::vector<PolygonsPart> Polygons::splitIntoParts(bool unionAll) const
{
    std::vector<PolygonsPart> ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoParts_processPolyTreeNode(&resultPolyTree, ret);
    return ret;
}

void Polygons::splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<PolygonsPart>& ret) const
{
    for(int n=0; n<node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        PolygonsPart part;
        part.add(child->Contour);
        for(int i=0; i<child->ChildCount(); i++)
        {
            part.add(child->Childs[i]->Contour);
            splitIntoParts_processPolyTreeNode(child->Childs[i], ret);
        }
        ret.push_back(part);
    }
}

Polygons Polygons::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const
{
    return this->offset(outer_offset).difference(this->offset(-inner_offset));
}

unsigned int PartsView::getPartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) const
{
    const PartsView& partsView = *this;
    for (unsigned int part_idx_now = 0; part_idx_now < partsView.size(); part_idx_now++)
    {
        const std::vector<unsigned int>& partView = partsView[part_idx_now];
        if (partView.size() == 0) { continue; }
        std::vector<unsigned int>::const_iterator result = std::find(partView.begin(), partView.end(), poly_idx);
        if (result != partView.end())
        {
            if (boundary_poly_idx) { *boundary_poly_idx = partView[0]; }
            return part_idx_now;
        }
    }
    return NO_INDEX;
}

PolygonsPart PartsView::assemblePart(unsigned int part_idx) const
{
    const PartsView& partsView = *this;
    PolygonsPart ret;
    if (part_idx != NO_INDEX)
    {
        for (unsigned int poly_idx_ff : partsView[part_idx])
        {
            ret.add(polygons[poly_idx_ff]);
        }
    }
    return ret;
}

PolygonsPart PartsView::assemblePartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) const
{
    PolygonsPart ret;
    unsigned int part_idx = getPartContaining(poly_idx, boundary_poly_idx);
    if (part_idx != NO_INDEX)
    {
        return assemblePart(part_idx);
    }
    return ret;
}

PartsView Polygons::splitIntoPartsView(bool unionAll)
{
    Polygons reordered;
    PartsView partsView(*this);
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoPartsView_processPolyTreeNode(partsView, reordered, &resultPolyTree);

    (*this) = reordered;
    return partsView;
}

void Polygons::splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Polygons& reordered, ClipperLib::PolyNode* node) const
{
    for(int n=0; n<node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        partsView.emplace_back();
        unsigned int pos = partsView.size() - 1;
        partsView[pos].push_back(reordered.size());
        reordered.add(child->Contour); //TODO: should this steal the internal representation for speed?
        for(int i = 0; i < child->ChildCount(); i++)
        {
            partsView[pos].push_back(reordered.size());
            reordered.add(child->Childs[i]->Contour);
            splitIntoPartsView_processPolyTreeNode(partsView, reordered, child->Childs[i]);
        }
    }
}


void Polygons::ensureManifold()
{
    const Polygons& polys = *this;
    std::vector<Point> duplicate_locations;
    std::unordered_set<Point> poly_locations;
    for (size_t poly_idx = 0; poly_idx < polys.size(); poly_idx++)
    {
        ConstPolygonRef poly = polys[poly_idx];
        for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p = poly[point_idx];
            if (poly_locations.find(p) != poly_locations.end())
            {
                duplicate_locations.push_back(p);
            }
            poly_locations.emplace(p);
        }
    }
    Polygons removal_dots;
    for (Point p : duplicate_locations)
    {
        PolygonRef dot = removal_dots.newPoly();
        dot.add(p + Point(0,5));
        dot.add(p + Point(5,0));
        dot.add(p + Point(0,-5));
        dot.add(p + Point(-5,0));
    }
    if ( ! removal_dots.empty())
    {
        *this = polys.difference(removal_dots);
    }
}

}//namespace cura
