// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/Polygon.h"

#include <cstddef>
#include <numbers>

#include "geometry/Point3Matrix.h"
#include "geometry/Shape.h"
#include "utils/ListPolyIt.h"
#include "utils/linearAlg2D.h"

namespace cura
{

Shape Polygon::intersection(const Polygon& other) const
{
    ClipperLib::Paths ret_paths;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPath(getPoints(), ClipperLib::ptSubject, true);
    clipper.AddPath(other.getPoints(), ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, ret_paths);
    return Shape{ std::move(ret_paths) };
}

void Polygon::smooth2(int remove_length, Polygon& result) const
{
    if (! empty())
    {
        result.push_back(front());
    }

    for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
    {
        const Point2LL& last = getPoints()[poly_idx - 1];
        const Point2LL& now = getPoints()[poly_idx];
        const Point2LL& next = getPoints()[(poly_idx + 1) % size()];
        if (shorterThen(last - now, remove_length) && shorterThen(now - next, remove_length))
        {
            poly_idx++; // skip the next line piece (dont escalate the removal of edges)
            if (poly_idx < size())
            {
                result.push_back(getPoints()[poly_idx]);
            }
        }
        else
        {
            result.push_back(getPoints()[poly_idx]);
        }
    }
}

void Polygon::smooth(int remove_length, Polygon& result) const
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
    if (! empty())
    {
        result.push_back(front());
    }
    auto is_zigzag = [remove_length](const int64_t v02_size, const int64_t v12_size, const int64_t v13_size, const int64_t dot1, const int64_t dot2)
    {
        if (v12_size > remove_length)
        { // v12 or v13 is too long
            return false;
        }
        const bool p1_is_left_of_v02 = dot1 < 0;
        if (! p1_is_left_of_v02)
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
    Point2LL v02 = getPoints()[2] - getPoints()[0];
    Point2LL v02_t = turn90CCW(v02);
    int64_t v02_size = vSize(v02);
    bool force_push = false;
    for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
    {
        const Point2LL& p1 = getPoints().at(poly_idx);
        const Point2LL& p2 = getPoints().at((poly_idx + 1) % size());
        const Point2LL& p3 = getPoints().at((poly_idx + 2) % size());
        // v02 computed in last iteration
        // v02_size as well
        const Point2LL v12 = p2 - p1;
        const int64_t v12_size = vSize(v12);
        const Point2LL v13 = p3 - p1;
        const int64_t v13_size = vSize(v13);

        // v02T computed in last iteration
        const int64_t dot1 = dot(v02_t, v12);
        const Point2LL v13_t = turn90CCW(v13);
        const int64_t dot2 = dot(v13_t, v12);
        bool push_point = force_push || ! is_zigzag(v02_size, v12_size, v13_size, dot1, dot2);
        force_push = false;
        if (push_point)
        {
            result.push_back(p1);
        }
        else
        {
            // do not add the current one to the result
            force_push = true; // ensure the next point is added; it cannot also be a zigzag
        }
        v02_t = v13_t;
        v02 = v13;
        v02_size = v13_size;
    }
}

void Polygon::smoothOutward(const AngleDegrees min_angle, int shortcut_length, Polygon& result) const
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
    double cos_min_angle = std::cos(min_angle / 180 * std::numbers::pi);

    ListPolygon poly;
    ListPolyIt::convertPolygonToList(*this, poly);

    { // remove duplicate vertices
        ListPolyIt p1_it(poly, poly.begin());
        do
        {
            ListPolyIt next = p1_it.next();
            if (vSize2(p1_it.p() - next.p()) < 100LL)
            {
                p1_it.remove();
            }
            p1_it = next;
        } while (p1_it != ListPolyIt(poly, poly.begin()));
    }

    ListPolyIt p1_it(poly, poly.begin());
    do
    {
        const Point2LL p1 = p1_it.p();
        ListPolyIt p0_it = p1_it.prev();
        ListPolyIt p2_it = p1_it.next();
        const Point2LL p0 = p0_it.p();
        const Point2LL p2 = p2_it.p();

        const Point2LL v10 = p0 - p1;
        const Point2LL v12 = p2 - p1;
        double cos_angle = INT2MM(INT2MM(dot(v10, v12))) / vSizeMM(v10) / vSizeMM(v12);
        bool is_left_angle = LinearAlg2D::pointIsLeftOfLine(p1, p0, p2) > 0;
        if (cos_angle > cos_min_angle && is_left_angle)
        {
            // angle is so sharp that it can be removed
            Point2LL v02 = p2_it.p() - p0_it.p();
            if (vSize2(v02) >= shortcut_length2)
            {
                smoothCornerSimple(p0, p1, p2, p0_it, p1_it, p2_it, v10, v12, v02, shortcut_length, cos_angle);
            }
            else
            {
                bool remove_poly = smoothCornerComplex(p1, p0_it, p2_it, shortcut_length); // edits p0_it and p2_it!
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

void Polygon::smoothCornerSimple(
    const Point2LL& p0,
    const Point2LL& p1,
    const Point2LL& p2,
    const ListPolyIt& p0_it,
    const ListPolyIt& p1_it,
    const ListPolyIt& p2_it,
    const Point2LL& v10,
    const Point2LL& v12,
    const Point2LL& v02,
    const int64_t shortcut_length,
    double cos_angle)
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
        const int64_t a1_size = shortcut_length / 2 / std::sin(a1m_angle);
        if (a1_size * a1_size < vSize2(v10) && a1_size * a1_size < vSize2(v12))
        {
            Point2LL a = p1 + normal(v10, a1_size);
            Point2LL b = p1 + normal(v12, a1_size);
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
            const Point2LL& b = p2_it.p();
            Point2LL a;
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
            const Point2LL& a = p0_it.p();
            Point2LL b;
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

void Polygon::smoothOutwardStep(
    const Point2LL& p1,
    const int64_t shortcut_length2,
    ListPolyIt& p0_it,
    ListPolyIt& p2_it,
    bool& forward_is_blocked,
    bool& backward_is_blocked,
    bool& forward_is_too_far,
    bool& backward_is_too_far)
{
    const bool forward_has_converged = forward_is_blocked || forward_is_too_far;
    const bool backward_has_converged = backward_is_blocked || backward_is_too_far;
    const Point2LL p0 = p0_it.p();
    const Point2LL p2 = p2_it.p();
    bool walk_forward
        = ! forward_has_converged && (backward_has_converged || (vSize2(p2 - p1) < vSize2(p0 - p1))); // whether to walk along the p1-p2 direction or in the p1-p0 direction

    if (walk_forward)
    {
        const ListPolyIt p2_2_it = p2_it.next();
        const Point2LL p2_2 = p2_2_it.p();
        bool p2_is_left = LinearAlg2D::pointIsLeftOfLine(p2, p0, p2_2) >= 0;
        if (! p2_is_left)
        {
            forward_is_blocked = true;
            return;
        }

        const Point2LL v02_2 = p2_2 - p0_it.p();
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
    const ListPolyIt p0_2_it = p0_it.prev();
    const Point2LL p0_2 = p0_2_it.p();
    bool p0_is_left = LinearAlg2D::pointIsLeftOfLine(p0, p0_2, p2) >= 0;
    if (! p0_is_left)
    {
        backward_is_blocked = true;
        return;
    }

    const Point2LL v02_2 = p2_it.p() - p0_2;
    if (vSize2(v02_2) > shortcut_length2)
    {
        backward_is_too_far = true;
        return;
    }

    p0_it = p0_2_it; // make one step in the backward direction
    forward_is_blocked = false; // invalidate data about forward walking
    forward_is_too_far = false;
}

bool Polygon::smoothCornerComplex(const Point2LL& p1, ListPolyIt& p0_it, ListPolyIt& p2_it, const int64_t shortcut_length)
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
            break;
        }
        smoothOutwardStep(p1, shortcut_length2, p0_it, p2_it, forward_is_blocked, backward_is_blocked, forward_is_too_far, backward_is_too_far);
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
            // this whole polygon can be removed
            return true;
        }
    }

    const Point2LL v02 = p2_it.p() - p0_it.p();
    const int64_t v02_size2 = vSize2(v02);
    // set the following:
    // p0_it = start point of line
    // p2_it = end point of line
    if (std::abs(v02_size2 - shortcut_length2) < shortcut_length * 10) // i.e. if (size2 < l * (l+10) && size2 > l * (l-10))
    { // v02 is approximately shortcut length
      // handle this separately to avoid rounding problems below in the getPointOnLineWithDist function
      // p0_it and p2_it are already correct
    }
    else if (! backward_is_blocked && ! forward_is_blocked)
    { // introduce two new points
        //  1----b---->2
        //  ^   /
        //  |  /
        //  | /
        //  |/
        //  |a
        //  |
        //  0
        const auto v02_size = static_cast<int64_t>(std::sqrt(v02_size2));

        const ListPolyIt p0_2_it = p0_it.prev();
        const ListPolyIt p2_2_it = p2_it.next();
        const Point2LL p2_2 = p2_2_it.p();
        const Point2LL p0_2 = p0_2_it.p();
        const Point2LL v02_2 = p0_2 - p2_2;
        const int64_t v02_2_size = vSize(v02_2);
        double progress
            = std::min(1.0, INT2MM(shortcut_length - v02_size) / INT2MM(v02_2_size - v02_size)); // account for rounding error when v02_2_size is approx equal to v02_size
        assert(progress >= 0.0f && progress <= 1.0f && "shortcut length must be between last length and new length");
        const Point2LL new_p0 = p0_it.p() + (p0_2 - p0_it.p()) * progress;
        p0_it = ListPolyIt::insertPointNonDuplicate(p0_2_it, p0_it, new_p0);
        const Point2LL new_p2 = p2_it.p() + (p2_2 - p2_it.p()) * progress;
        p2_it = ListPolyIt::insertPointNonDuplicate(p2_it, p2_2_it, new_p2);
    }
    else if (! backward_is_blocked)
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
        const Point2LL p0 = p0_it.p();
        const Point2LL p0_2 = p0_2_it.p();
        const Point2LL p2 = p2_it.p();
        Point2LL new_p0;
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
    else if (! forward_is_blocked)
    { // backward is blocked, front is open
        //  1----2----b----------->2_2
        //  ^      ,-'
        //  |   ,-'
        //--0.-'
        //  a
        const ListPolyIt p2_2_it = p2_it.next();
        const Point2LL p0 = p0_it.p();
        const Point2LL p2 = p2_it.p();
        const Point2LL p2_2 = p2_2_it.p();
        Point2LL new_p2;
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

Point2LL Polygon::centerOfMass() const
{
    if (! empty())
    {
        Point2LL p0 = getPoints()[0];
        if (size() > 1)
        {
            double x{ 0 };
            double y{ 0 };
            for (size_t n = 1; n <= size(); n++)
            {
                Point2LL p1 = getPoints()[n % size()];
                auto second_factor = static_cast<double>((p0.X * p1.Y) - (p1.X * p0.Y));

                x += static_cast<double>(p0.X + p1.X) * second_factor;
                y += static_cast<double>(p0.Y + p1.Y) * second_factor;
                p0 = p1;
            }

            double current_area = area();

            x = x / 6 / current_area;
            y = y / 6 / current_area;

            return { std::llrint(x), std::llrint(y) };
        }
        return p0;
    }
    return {};
}

Shape Polygon::offset(int distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (distance == 0)
    {
        return Shape({ *this });
    }
    ClipperLib::Paths ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    clipper.AddPath(getPoints(), join_type, ClipperLib::etClosedPolygon);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret, distance);
    return Shape{ std::move(ret) };
}

} // namespace cura
