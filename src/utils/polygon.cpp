//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/polygon.h"

#include <numeric>
#include <unordered_set>

#include "utils/linearAlg2D.h" // pointLiesOnTheRightOfLine
#include "utils/Simplify.h"

#include "utils/ListPolyIt.h"

#include "utils/PolylineStitcher.h"

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
    for(PolygonRef poly : *this)
    {
        if(poly.size() <= 3)
        {
            continue; //Already convex.
        }

        Polygon convexified;

        //Start from a vertex that is known to be on the convex hull: The one with the lowest X.
        const size_t start_index = std::min_element(poly.begin(), poly.end(), [](Point a, Point b) { return a.X == b.X ? a.Y < b.Y : a.X < b.X; }) - poly.begin();
        convexified.path->push_back(poly[start_index]);

        for(size_t i = 1; i <= poly.size(); ++ i)
        {
            const Point& current = poly[(start_index + i) % poly.size()];

            //Track backwards to make sure we haven't been in a concave pocket for multiple vertices already.
            while(convexified.size() >= 2
                && (LinearAlg2D::pointIsLeftOfLine(convexified.path->back(), (*convexified.path)[convexified.size() - 2], current) >= 0
                ||  LinearAlg2D::pointIsLeftOfLine(convexified.path->back(), (*convexified.path)[convexified.size() - 2], convexified.path->front()) > 0))
            {
                convexified.path->pop_back();
            }
            convexified.path->push_back(current);
        }
        //remove last vertex as the starting vertex is added in the last iteration of the loop
        convexified.path->pop_back();

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

Polygons Polygons::intersectionPolyLines(const Polygons& polylines, bool restitch, const coord_t max_stitch_distance) const
{
    Polygons split_polylines = polylines.splitPolylinesIntoSegments();
    
    ClipperLib::PolyTree result;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPaths(split_polylines.paths, ClipperLib::ptSubject, false);
    clipper.AddPaths(paths, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, result);
    Polygons ret;
    ClipperLib::OpenPathsFromPolyTree(result, ret.paths);
    
    if (restitch)
    {
        Polygons result_lines, result_polygons;
        const coord_t snap_distance = 10_mu;
        PolylineStitcher<Polygons, Polygon, Point>::stitch(ret, result_lines, result_polygons, max_stitch_distance, snap_distance);
        ret = result_lines;
        // if polylines got stitched into polygons, split them back up into a polyline again, because the result only admits polylines
        for (PolygonRef poly : result_polygons)
        {
            if (poly.empty()) continue;
            if (poly.size() > 2)
            {
                poly.emplace_back(poly[0]);
            }
            ret.add(poly);
        }
    }

    return ret;
}

void Polygons::toPolylines()
{
    for (PolygonRef poly : *this)
    {
        if (poly.empty()) continue;
        poly.emplace_back(poly.front());
    }
}

void Polygons::splitPolylinesIntoSegments(Polygons& result) const
{
    for (ConstPolygonRef poly : *this)
    {
        poly.splitPolylineIntoSegments(result);
    }
}
Polygons Polygons::splitPolylinesIntoSegments() const
{
    Polygons ret;
    splitPolylinesIntoSegments(ret);
    return ret;
}

void Polygons::splitPolygonsIntoSegments(Polygons& result) const
{
    for (ConstPolygonRef poly : *this)
    {
        poly.splitPolygonIntoSegments(result);
    }
}
Polygons Polygons::splitPolygonsIntoSegments() const
{
    Polygons ret;
    splitPolygonsIntoSegments(ret);
    return ret;
}

coord_t Polygons::polyLineLength() const
{
    coord_t length = 0;
    for (ConstPolygonRef poly : *this)
    {
        length += poly.polylineLength();
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
    if (remove_holes)
    {
        for (auto it = paths.begin(); it < new_end;)
        {
            // All polygons smaller than target are removed by replacing them with a polygon from the back of the vector
            if (std::abs(INT2MM2(ClipperLib::Area(*it))) < min_area_size)
            {
                *it = std::move(*--new_end);
                continue;
            }
            it++; // Skipped on removal such that the polygon just swaped in is checked next
        }
    }
    else
    {
        // For each polygon, computes the signed area, move small outlines at the end of the vector and keep references on small holes
        std::vector<PolygonRef> small_holes;
        for (auto it = paths.begin(); it < new_end;)
        {
            double area = INT2MM2(ClipperLib::Area(*it));
            if (std::abs(area) < min_area_size)
            {
                if (area >= 0)
                {
                    --new_end;
                    if (it < new_end)
                    {
                        std::swap(*new_end, *it);
                        continue;
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
            it++; // Skipped on removal such that the polygon just swaped in is checked next
        }

        // Removes small holes that have their first point inside one of the removed outlines
        // Iterating in reverse ensures that unprocessed small holes won't be moved
        const auto removed_outlines_start = new_end;
        for (auto hole_it = small_holes.rbegin(); hole_it < small_holes.rend(); hole_it++)
        {
            for (auto outline_it = removed_outlines_start; outline_it < paths.end(); outline_it++)
            {
                if (PolygonRef(*outline_it).inside(*hole_it->begin()))
                {
                    **hole_it = std::move(*--new_end);
                    break;
                }
            }
        }
    }
    paths.resize(new_end - paths.begin());
}

void Polygons::removeDegenerateVerts()
{
    _removeDegenerateVerts(false);
}

void Polygons::removeDegenerateVertsPolyline()
{
    _removeDegenerateVerts(true);
}

void Polygons::_removeDegenerateVerts(const bool for_polyline)
{
    Polygons& thiss = *this;
    for(size_t poly_idx = 0; poly_idx < size(); poly_idx++)
    {
        PolygonRef poly = thiss[poly_idx];
        Polygon result;

        auto isDegenerate = [](const Point& last, const Point& now, const Point& next)
        {
            Point last_line = now - last;
            Point next_line = next - now;
            return dot(last_line, next_line) == -1 * vSize(last_line) * vSize(next_line);
        };

        //With polylines, skip the first and last vertex.
        const size_t start_vertex = for_polyline ? 1 : 0;
        const size_t end_vertex = for_polyline ? poly.size() - 1 : poly.size();
        for(size_t i = 0; i < start_vertex; ++i)
        {
            result.add(poly[i]); //Add everything before the start vertex.
        }

        bool isChanged = false;
        for(size_t idx = start_vertex; idx < end_vertex; idx++)
        {
            const Point& last = (result.size() == 0) ? poly.back() : result.back();
            if(idx + 1 >= poly.size() && result.size() == 0)
            {
                break;
            }
            const Point& next = (idx + 1 >= poly.size()) ? result[0] : poly[idx + 1];
            if(isDegenerate(last, poly[idx], next))
            { // lines are in the opposite direction
                // don't add vert to the result
                isChanged = true;
                while(result.size() > 1 && isDegenerate(result[result.size() - 2], result.back(), next))
                {
                    result.pop_back();
                }
            }
            else
            {
                result.add(poly[idx]);
            }
        }

        for(size_t i = end_vertex; i < poly.size(); ++i)
        {
            result.add(poly[i]); //Add everything after the end vertex.
        }

        if(isChanged)
        {
            if(for_polyline || result.size() > 2)
            {
                *poly = *result;
            }
            else
            {
                thiss.remove(poly_idx);
                poly_idx--; // effectively the next iteration has the same poly_idx (referring to a new poly which is not yet processed)
            }
        }
    }
}

Polygons Polygons::toPolygons(ClipperLib::PolyTree& poly_tree)
{
    Polygons ret;
    ClipperLib::PolyTreeToPaths(poly_tree, ret.paths);
    return ret;
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




void ConstPolygonRef::splitPolylineIntoSegments(Polygons& result) const 
{
    Point last = front();
    for (size_t idx = 1; idx < size(); idx++)
    {
        Point p = (*this)[idx];
        result.addLine(last, p);
        last = p;
    }
}

Polygons ConstPolygonRef::splitPolylineIntoSegments() const 
{
    Polygons ret;
    splitPolylineIntoSegments(ret);
    return ret;
}

void ConstPolygonRef::splitPolygonIntoSegments(Polygons& result) const
{
    splitPolylineIntoSegments(result);
    result.addLine(back(), front());
}

Polygons ConstPolygonRef::splitPolygonIntoSegments() const 
{
    Polygons ret;
    splitPolygonIntoSegments(ret);
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

std::vector<Polygons> Polygons::sortByNesting() const
{
    std::vector<Polygons> ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    sortByNesting_processPolyTreeNode(&resultPolyTree, 0, ret);
    return ret;
}

void Polygons::sortByNesting_processPolyTreeNode(ClipperLib::PolyNode* node, const size_t nesting_idx, std::vector<Polygons>& ret) const
{
    for (int n = 0; n < node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        if (nesting_idx >= ret.size())
        {
            ret.resize(nesting_idx + 1);
        }
        ret[nesting_idx].add(child->Contour);
        sortByNesting_processPolyTreeNode(child, nesting_idx + 1, ret);
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
