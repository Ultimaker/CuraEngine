/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "polygon.h"

#include "linearAlg2D.h" // pointLiesOnTheRightOfLine

#include "ListPolyIt.h"

namespace cura 
{

bool PolygonRef::shorterThan(int64_t check_length) const
{
    const PolygonRef& polygon = *this;
    const Point* p0 = &polygon.back();
    int64_t length = 0;
    for (const Point& p1 : polygon)
    {
        length += vSize(*p0 - p1);
        if (length >= check_length)
        {
            return false;
        }
        p0 = &p1;
    }
    return true;
}

bool PolygonRef::_inside(Point p, bool border_result)
{
    PolygonRef thiss = *this;
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

bool Polygons::inside(Point p, bool border_result) const
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
    
    int64_t min_x[size()];
    std::fill_n(min_x, size(), std::numeric_limits<int64_t>::max());  // initialize with int.max
    int crossings[size()];
    std::fill_n(crossings, size(), 0);  // initialize with zeros
    
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

Polygons PolygonRef::offset(int distance, ClipperLib::JoinType joinType, double miter_limit) const
{
    Polygons ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    clipper.AddPath(*path, joinType, ClipperLib::etClosedPolygon);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret.paths, distance);
    return ret;
}

Polygon Polygons::convexHull() const
{
    // Implements Andrew's monotone chain convex hull algorithm
    // See https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain

    using IntPoint = ClipperLib::IntPoint;

    size_t num_points = 0U;
    for (const ClipperLib::Path& path : paths)
    {
        num_points += path.size();
    }

    std::vector<IntPoint> all_points;
    all_points.reserve(num_points);
    for (const ClipperLib::Path& path : paths)
    {
        for (const IntPoint& point : path)
        {
            all_points.push_back(point);
        }
    }

    struct HullSort
    {
        bool operator()(const IntPoint& a,
                        const IntPoint& b)
        {
            return (a.X < b.X) ||
                (a.X == b.X && a.Y < b.Y);
        }
    };

    std::sort(all_points.begin(), all_points.end(), HullSort());

    // positive for left turn, 0 for straight, negative for right turn
    auto ccw = [](const IntPoint& p0, const IntPoint& p1, const IntPoint& p2) -> int64_t {
        IntPoint v01(p1.X - p0.X, p1.Y - p0.Y);
        IntPoint v12(p2.X - p1.X, p2.Y - p1.Y);

        return
            static_cast<int64_t>(v01.X) * v12.Y -
            static_cast<int64_t>(v01.Y) * v12.X;
    };

    Polygon hull_poly;
    ClipperLib::Path& hull_points = *hull_poly;
    hull_points.resize(num_points+1);
    // index to insert next hull point, also number of valid hull points
    size_t hull_idx = 0;

    // Build lower hull
    for (size_t pt_idx = 0U; pt_idx != num_points; ++pt_idx)
    {
        while (hull_idx >= 2 &&
               ccw(hull_points[hull_idx-2], hull_points[hull_idx-1],
                   all_points[pt_idx]) <= 0)
        {
            --hull_idx;
        }
        hull_points[hull_idx] = all_points[pt_idx];
        ++hull_idx;
    }

    // Build upper hull
    size_t min_upper_hull_chain_end_idx = hull_idx+1;
    for (int pt_idx = num_points-2; pt_idx >= 0; --pt_idx)
    {
        while (hull_idx >= min_upper_hull_chain_end_idx &&
               ccw(hull_points[hull_idx-2], hull_points[hull_idx-1],
                   all_points[pt_idx]) <= 0)
        {
            --hull_idx;
        }
        hull_points[hull_idx] = all_points[pt_idx];
        ++hull_idx;
    }

    assert(hull_idx <= hull_points.size());

    // Last point is duplicted with first.  It is removed in the resize.
    hull_points.resize(hull_idx - 2);

    return hull_poly;
}

void PolygonRef::simplify(int smallest_line_segment_squared, int allowed_error_distance_squared){

    if (size() < 3)
    {
        clear();
        return;
    }

// Algorithm Explanation:
// initial: 1
// after stage I: 2
// after first step of stage II: 3
//
//                       111111111111111111111111111
//                      1         22 332222         1
//                     1       222  3  3   222       1
//                    1      22    3   3      222     1
//                    1   222      3    3        2222 1
//                   1 222        3      3           221
//                  122          3       3              12
//                 12            3        3              1
//                 12           3          3             1
//                1 2          3            3            21
//               1  2          3            3            21
//               1  2         3              3          2  1
//              1    2        3               3         2  1
//              1    2       3                 3        2   1
//             1     2      3                  3        2   1
//              1    2      3                   3       2    1
//               1    2    3                     3      2    1
//                1   2   3                      3     2    1
//                 1  2   3                       3    2   1
//                  1  2 3                         3   2  1
//                   1 23                           3  2 1
//                    123                           3  21
//                     1                             321
//                                                    1
//
// this is neccesary in order to avoid the case where a long segment is followed by a lot of small segments would get simplified to a long segment going to the wrong end point
//  .......                _                 _______
// |                      /                 |
// |     would become    /    instead of    |
// |                    /                   |
//
// therefore every time a vert is removed the next is kept (for the moment)
//
// .......            ._._._._.         .___.___.          .________
// |                  |                 |                  |
// |    will  become  |    will become  |     will become  |
// |                  |                 |                  |
//

    ClipperLib::Path this_path = *path;
    coord_t min_length_2 = std::min(smallest_line_segment_squared, allowed_error_distance_squared);

    ListPolygon result_list_poly;
    result_list_poly.emplace_back(path->front());

    std::vector<ListPolyIt> skipped_verts;
    // add the first point as starting point for the algorithm
    // whether the first point has to be removed is checked separately afterwards
    skipped_verts.emplace_back(result_list_poly, result_list_poly.begin());

    char here_is_beyond_line = 0;
    { // stage I: convert to a ListPolygon and remove verts, but don't remove verts just after removed verts (i.e. skip them)
        Point prev = this_path[0];
        auto skip = [this, &result_list_poly, &this_path, &prev, &skipped_verts](unsigned int& poly_idx)
        {
            poly_idx++;
            if (poly_idx >= size())
            { // don't wrap around, the first point has already been added
                return;
            }
            result_list_poly.emplace_back(this_path[poly_idx]);
            prev = result_list_poly.back();
            skipped_verts.emplace_back(result_list_poly, --result_list_poly.end());
        };

        for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
        {
            const Point& here = this_path[poly_idx];
            const Point& next = this_path[(poly_idx + 1) % size()];
            if (here == next)
            { // disregard duplicate points without skipping the next point
                continue;
            }
            if ( vSize2(here - prev) < min_length_2 && vSize2(next - here) < min_length_2 )
            {
                // don't add [here] to the result
                // skip checking whether the next point has to be removed for now
                skip(poly_idx);
                continue;
            }
            int64_t error2 = LinearAlg2D::getDist2FromLineSegment(prev, here, next, &here_is_beyond_line);
            if (here_is_beyond_line == 0 && error2 < allowed_error_distance_squared)
            {
                // don't add [here] to the result
                // skip checking whether the next point has to be removed for now
                skip(poly_idx);
            }
            else
            {
                result_list_poly.emplace_back(here);
                prev = result_list_poly.back();
            }
        }
    }

    { // stage II: keep removing skipped verts (and skip the next vert if it was a skipped vert)
        auto skip = [&skipped_verts, &result_list_poly](unsigned int& skipped_vert_idx, const ListPolyIt skipped_vert, std::vector<ListPolyIt>& new_skipped_verts)
        {
            unsigned int next_skipped_vert_idx = skipped_vert_idx + 1;
            if (next_skipped_vert_idx < skipped_verts.size() && skipped_vert.next() == skipped_verts[next_skipped_vert_idx])
            {
                skipped_vert_idx++;
                new_skipped_verts.emplace_back(skipped_verts[next_skipped_vert_idx]);
            }
            result_list_poly.erase(skipped_vert.it);
        };
        while (!skipped_verts.empty() && result_list_poly.size() >= 3)
        {
            std::vector<ListPolyIt> new_skipped_verts;

            for (unsigned int skipped_vert_idx = 0; skipped_vert_idx < skipped_verts.size(); skipped_vert_idx++)
            {
                ListPolyIt skipped_vert = skipped_verts[skipped_vert_idx];
                const Point& here = skipped_vert.p();
                const Point& prev = skipped_vert.prev().p();

                const Point& next = skipped_vert.next().p();
                if ( vSize2(here - prev) < min_length_2 && vSize2(next - here) < min_length_2 )
                {
                    // skip checking whether the next point has to be removed for now
                    skip(skipped_vert_idx, skipped_vert, new_skipped_verts);
                    continue;
                }
                int64_t error2 = LinearAlg2D::getDist2FromLineSegment(prev, here, next, &here_is_beyond_line);
                if (here_is_beyond_line == 0 && error2 < allowed_error_distance_squared)
                {
                    // skip checking whether the next point has to be removed for now
                    skip(skipped_vert_idx, skipped_vert, new_skipped_verts);
                }
                else
                {
                    // keep the point
                }
            }
            skipped_verts = new_skipped_verts;
        }
    }

    clear();
    if (result_list_poly.size() < 3)
    {
        return;
    }
    ListPolyIt::convertListPolygonToPolygon(result_list_poly, *this);
}

void PolygonRef::smooth_outward_step(const Point p1, const int64_t shortcut_length, ListPolyIt& p0_it, ListPolyIt& p2_it, bool& forward_is_blocked, bool& backward_is_blocked, bool& forward_is_too_far, bool& backward_is_too_far)
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
        if (vSize2(v02_2) > shortcut_length * shortcut_length)
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
        if (vSize2(v02_2) > shortcut_length * shortcut_length)
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

void PolygonRef::smooth_corner_simple(ListPolygon& poly, const Point p0, const Point p1, const Point p2, const ListPolyIt p0_it, const ListPolyIt p1_it, const ListPolyIt p2_it, const Point v10, const Point v12, const Point v02, const int64_t shortcut_length, float cos_angle)
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
            p1_it.remove();
            poly.insert(p2_it.it, a);
            poly.insert(p2_it.it, b);
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
            assert(success && "v02 has to be longer than ab!");
            poly.insert(p1_it.it, a);
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
            assert(success && "v02 has to be longer than ab!");
            p1_it.remove();
            poly.insert(p2_it.it, b);
        }
    }
}

void PolygonRef::smooth_outward(float min_angle, int shortcut_length, PolygonRef result) const
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
            if (vSize2(v02) >= shortcut_length * shortcut_length)
            {
                smooth_corner_simple(poly, p0, p1, p2, p0_it, p1_it, p2_it, v10, v12, v02, shortcut_length, cos_angle);
                // update:
                p1_it = p2_it; // next point to consider for whether it's an internal corner
            }
            else
            { // walk away from the corner until the shortcut > shortcut_length or it would smooth a piece inward
                // - walk in both directions untill shortcut > shortcut_length 
                // - stop walking in one direction if it would otherwise cut off a corner in that direction
                // - same in the other direction
                // - stop if both are cut off
                // walk by updating p0_it and p2_it
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
                        break;
                    }
                    smooth_outward_step(p1, shortcut_length, p0_it, p2_it, forward_is_blocked, backward_is_blocked, forward_is_too_far, backward_is_too_far);
                }
//                 set the following:
//                 p0_it = start point of line
//                 p2_it = end point of line
                if (std::abs(vSize2(v02) - shortcut_length * shortcut_length) < shortcut_length * 10) // i.e. if (size2 < l * (l+10) && size2 > l * (l-10))
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
                    const int64_t v02_size = vSize(v02);

                    const ListPolyIt p0_2_it = p0_it.prev();
                    const ListPolyIt p2_2_it = p2_it.next();
                    const Point p2_2 = p2_2_it.p();
                    const Point p0_2 = p0_2_it.p();
                    const Point v02_2 = p0_2 - p2_2;
                    const int64_t v02_2_size = vSize(v02_2);
                    float progress = INT2MM(shortcut_length - v02_size) / INT2MM(v02_2_size - v02_size);
                    assert(progress >= 0.0f && progress <= 1.0f && "shortcut length must be between last length and new length");
                    const Point new_p0 = p0_it.p() + (p0_2 - p0_it.p()) * progress;
                    p0_it = ListPolyIt(poly, poly.insert(p0_it.it, new_p0));
                    const Point new_p2 = p2_it.p() + (p2_2 - p2_it.p()) * progress;
                    p2_it = ListPolyIt(poly, poly.insert(p2_2_it.it, new_p2));
                }
                else if (!backward_is_blocked)
                { // forward is blocked, back is open
                    //     |
                    //  1->b
                    //  ^  :
                    //  | /
                    //  | :
                    //  |/
                    //  |a
                    //  |
                    //  0
                    Point new_p0;
                    bool success = LinearAlg2D::getPointOnLineWithDist(p2_it.p(), p0_it.p(), p0_it.prev().p(), shortcut_length, new_p0);
                    assert(success && "shortcut length must be possible given that last length was ok and new length is too long");
                    p0_it = ListPolyIt(poly, poly.insert(p0_it.it, new_p0));
                }
                else if (!forward_is_blocked)
                { // backward is blocked, front is open
                    //  1---------b----------->2
                    //  ^      ,-'
                    //  |   ,-'
                    //--0.-'
                    //  a
                    Point new_p2;
                    bool success = LinearAlg2D::getPointOnLineWithDist(p0_it.p(), p2_it.p(), p2_it.next().p(), shortcut_length, new_p2);
                    assert(success && "shortcut length must be possible given that last length was ok and new length is too long");
                    p2_it = ListPolyIt(poly, poly.insert(p2_it.next().it, new_p2));
                }
                else
                {
                    //        |
                    //      __|
                    //     | /  > shortcut cannot be of the desired length
                    //  ___|/                                                           .
                    // both are blocked and p0_it and p2_it are already correct
                }
                // delete all cut off points
                while (p0_it.next() != p2_it)
                {
                    p0_it.next().remove();
                }
                // update:
                p1_it = p2_it; // next point to consider for whether it's an internal corner
            }
        }
        else
        {
            ++p1_it;
        }
    } while (p1_it != ListPolyIt(poly, poly.begin()));

    ListPolyIt::convertListPolygonToPolygon(poly, result);
}

Polygons Polygons::smooth_outward(float max_angle, int shortcut_length)
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

void PolygonRef::smooth(int remove_length, PolygonRef result)
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
    PolygonRef& thiss = *this;
    ClipperLib::Path* poly = result.path;
    if (size() > 0)
    {
        poly->push_back(thiss[0]);
    }
    auto is_zigzag = [remove_length](const Point v02, const int64_t v02_size, const Point v12, const int64_t v12_size, const Point v13, const int64_t v13_size, const int64_t dot1, const int64_t dot2)
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
        bool push_point = force_push || !is_zigzag(v02, v02_size, v12, v12_size, v13, v13_size, dot1, dot2);
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

Polygons Polygons::smooth(int remove_length)
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
        poly.smooth(remove_length, ret.newPoly());
        PolygonRef back = ret.back();
        if (back.size() < 3)
        {
            back.path->resize(back.path->size() - 1);
        }
    }
    return ret;
}

void PolygonRef::smooth2(int remove_length, PolygonRef result)
{
    PolygonRef& thiss = *this;
    ClipperLib::Path* poly = result.path;
    if (size() > 0)
    {
        poly->push_back(thiss[0]);
    }
    for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
    {
        Point& last = thiss[poly_idx - 1];
        Point& now = thiss[poly_idx];
        Point& next = thiss[(poly_idx + 1) % size()];
        if (shorterThen(last - now, remove_length) && shorterThen(now - next, remove_length)) 
        {
            poly_idx++; // skip the next line piece (dont escalate the removal of edges)
            if (poly_idx < size())
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

Polygons Polygons::smooth2(int remove_length, int min_area)
{
    Polygons ret;
    for (unsigned int p = 0; p < size(); p++)
    {
        PolygonRef poly(paths[p]);
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

unsigned int PartsView::getPartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) 
{
    PartsView& partsView = *this;
    for (unsigned int part_idx_now = 0; part_idx_now < partsView.size(); part_idx_now++)
    {
        std::vector<unsigned int>& partView = partsView[part_idx_now];
        if (partView.size() == 0) { continue; }
        std::vector<unsigned int>::iterator result = std::find(partView.begin(), partView.end(), poly_idx);
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

PolygonsPart PartsView::assemblePartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) 
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
        reordered.add(child->Contour);
        for(int i = 0; i < child->ChildCount(); i++)
        {
            partsView[pos].push_back(reordered.size());
            reordered.add(child->Childs[i]->Contour);
            splitIntoPartsView_processPolyTreeNode(partsView, reordered, child->Childs[i]);
        }
    }
}



}//namespace cura
