#ifndef CURA_INFILL_MATH_H
#define CURA_INFILL_MATH_H

#include <cassert>
#include <cstdint>
#include <limits>
#ifndef NDEBUG
# include <iostream>
#endif // NDEBUG

#include "../../libs/clipper/clipper.hpp"
#include "../utils/polygon.h"


namespace cura {

/*
 * Calculates the distance between the two given points (with integer coordinates)
 */
static inline double p2p_dist(const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& p2)
{
    return std::sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y));
}


static inline void compute_unit_vector(
    double& vec_x, double& vec_y,
    const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& p2)
{
    vec_x = p2.X - p1.X;
    vec_y = p2.Y - p1.Y;
    const double length = p2p_dist(p1, p2);
    vec_x /= length;
    vec_y /= length;
}


static inline double get_path_length(const ClipperLib::Path& path)
{
    double length = 0;
    for (uint64_t i = 1; i < path.size(); ++i)
    {
        length += p2p_dist(path[i - 1], path[i]);
    }
    length += p2p_dist(path[path.size() - 1], path[0]);
    return length;
}


static inline double compute_path_length(const ClipperLib::Path& path, uint32_t start_idx, uint32_t end_idx)
{
    double length = 0;
    for (uint32_t i = start_idx + 1; i <= end_idx; ++i)
    {
        length += p2p_dist(path[i - 1], path[i]);
    }
    return length;
}


static inline ClipperLib::IntPoint compute_polygon_center(const ClipperLib::Path& path)
{
    ClipperLib::IntPoint center(0, 0);
    for (auto itr_pt = path.begin(); itr_pt != path.end(); ++itr_pt)
    {
        center.X += (*itr_pt).X;
        center.Y += (*itr_pt).Y;
    }
    center.X = std::round(static_cast<double>(center.X) / path.size());
    center.Y = std::round(static_cast<double>(center.Y) / path.size());

    return center;
}


static inline compute_line_kb(double& k, double& b, const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& p2)
{
    k = static_cast<double>(p2.Y - p1.Y) / (p2.X - p1.X);
    b = p1.Y - k * p1.X;
}


static inline void compute_line_intersection(ClipperLib::IntPoint& intersection,
    const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& q1,
    const ClipperLib::IntPoint& p2, const ClipperLib::IntPoint& q2)
{
    double k1, b1;
    double k2, b2;
    compute_line_kb(k1, b1, p1, q1);
    compute_line_kb(k2, b2, p2, q2);

    double x = (b1 - b2) / (k2 - k1);
    double y = k1 * x + b1;

    intersection.X = std::round(x);
    intersection.Y = std::round(y);
}


static inline bool is_point_in_path(const ClipperLib::IntPoint& point, const ClipperLib::Path& path)
{
    bool on_path = false;
    for (auto itr_point = path.begin(); itr_point != path.end(); ++itr_point)
    {
        if (point.X == (*itr_point).X and point.Y == (*itr_point).Y)
        {
            on_path = true;
            break;
        }
    }
    return on_path;
}


static inline bool get_point_idx_in_path(uint64_t index, const ClipperLib::IntPoint& point, const ClipperLib::Path& path)
{
    bool found_point = false;
    index = 0;
    for (auto itr_point = path.begin(); itr_point != path.end(); ++itr_point)
    {
        if (point == *itr_point)
        {
            found_point = true;
            break;
        }
        ++index;
    }

    return found_point;
}


static inline double p2l_distance(const ClipperLib::IntPoint& point, const ClipperLib::IntPoint& lp1, const ClipperLib::IntPoint& lp2)
{
    double a = lp2.Y - lp1.Y;
    double b = lp2.X - lp1.X;
    double c = lp1.X * lp2.Y - lp2.X * lp1.Y;

    return std::abs(a * point.X + b * point.Y + c) / std::sqrt(a * a + b * b);
}


/*
 * Computes the direction of the given path. This only works with non-convex polygons.
 * Returns a positive number if it's clockwise, a negative number if it's counter-clockwise.
 */
static inline int32_t compute_path_direction(const ClipperLib::Path& path)
{
    int64_t sum = 0;
    for (auto itr_pt = path.begin(); itr_pt != path.end(); ++itr_pt)
    {
        if (itr_pt + 1 == path.end())
        {
            break;
        }

        const ClipperLib::IntPoint& p1 = *itr_pt;
        const ClipperLib::IntPoint& p2 = *(itr_pt + 1);

        sum += (p2.X - p1.X) * (p2.Y + p1.Y);
    }

    // last edge
    const ClipperLib::IntPoint& p1 = path[path.size() - 1];
    const ClipperLib::IntPoint& p2 = path[0];
    sum += (p2.X - p1.X) * (p2.Y + p1.Y); 

    if (sum > 0)
    {
        sum = 1;
    }
    else if (sum < 0)
    {
        sum = -1;
    }
    assert(sum != 0);
    return sum;
}


/*
 * Reverse the direction of the given path and save it to the result path.
 */
static inline void reverse_path_direction(ClipperLib::Path& result_path, const ClipperLib::Path& original_path)
{
    for (auto itr_pt = original_path.crbegin(); itr_pt != original_path.crend(); ++itr_pt)
    {
        result_path << *itr_pt;
        assert((*itr_pt).X >= 0);
        assert((*itr_pt).Y >= 0);
    }
    assert(result_path.size() == original_path.size());
}


static inline bool is_point_on_line_segment(
    const ClipperLib::IntPoint& point,
    const ClipperLib::IntPoint& p,
    const ClipperLib::IntPoint& q)
{
    if (point.X >= std::min(p.X, q.X) and point.X <= std::max(p.X, q.X) and
        point.Y <= std::max(p.Y, q.Y) and point.Y <= std::max(p.Y, q.Y))
    {
        return true;
    }
    return false;
}


static inline bool calculate_closest_distance_on_path_from_point(
    double& closest_distance,
    const ClipperLib::IntPoint& from_point,
    const ClipperLib::Path& to_path)
{
    uint64_t current_index = 0;
    closest_distance = std::numeric_limits<double>::max();
    bool found_closest_distance = false;
    for (auto itr_pt = to_path.begin(); itr_pt != to_path.end(); ++itr_pt)
    {
        if (current_index + 1 >= to_path.size())
        {
            break;
        }

        const struct ClipperLib::IntPoint& p1 = *itr_pt;
        const struct ClipperLib::IntPoint& p2 = *(itr_pt + 1);

        struct ClipperLib::IntPoint vec1;
        vec1.X = p2.X - p1.X;
        vec1.Y = p2.Y - p1.Y;
        struct ClipperLib::IntPoint vec2;
        vec2.X = from_point.X - p1.X;
        vec2.Y = from_point.Y - p1.Y;

        // v[proj] = (v1 * v2 ) / |v1|^2 * v1
        ClipperLib::IntPoint vec_proj;
        double factor = (vec1.X * vec2.X) + (vec1.Y * vec2.Y);
        double v1_len_2 = vec1.X * vec1.X + vec1.Y * vec1.Y;
        factor = factor / v1_len_2;
        double x = vec1.X * factor;
        double y = vec1.Y * factor;

        // get vector from p1 -> projected point
        vec_proj.X = std::round(x);
        vec_proj.Y = std::round(y);

        // get projected point
        ClipperLib::IntPoint projected_point;
        projected_point.X = p1.X + vec_proj.X;
        projected_point.Y = p1.Y + vec_proj.Y;

        // get the vector from fp -> projected point
        ClipperLib::IntPoint vec_inward;
        vec_inward.X = projected_point.X - from_point.X;
        vec_inward.Y = projected_point.Y - from_point.Y;

        // make a line following the direction from fp -> projected point but goes further
        // for detecting intersection.
        ClipperLib::IntPoint other_point;
        other_point.X = from_point.X + vec_inward.X * 2;
        other_point.Y = from_point.Y + vec_inward.Y * 2;

        double distance = std::sqrt((vec_inward.X * vec_inward.X) + (vec_inward.Y * vec_inward.Y));
        if (distance < closest_distance and is_point_on_line_segment(projected_point, p1, p2))
        {
            closest_distance = distance;
            found_closest_distance = true;
        }

        ++current_index;
    }

    return found_closest_distance;
}


/*
 * Gets the nearest point on the given path to the given point.
 */
static inline bool get_nearest_point_on_path_from_point(
    ClipperLib::IntPoint& nearest_point,
    int64_t& start_index,
    int64_t& end_index,
    const ClipperLib::IntPoint& from_point,
    const ClipperLib::Path& to_path)
{
    double closest_distance = std::numeric_limits<double>::max();
    start_index = 0;
    end_index = 1;

    uint64_t current_index = 0;
    bool found_nearest_point = false;

    for (auto itr_pt = to_path.begin(); itr_pt != to_path.end(); ++itr_pt)
    {
        if (current_index + 1 >= to_path.size())
        {
            break;
        }

        const ClipperLib::IntPoint& p1 = *itr_pt;
        const ClipperLib::IntPoint& p2 = *(itr_pt + 1);

        ClipperLib::IntPoint vec1;
        vec1.X = p2.X - p1.X;
        vec1.Y = p2.Y - p1.Y;

        // compute projection of vector p1 -> from_point on vec1
        ClipperLib::IntPoint v2;
        v2.X = from_point.X - p1.X;
        v2.Y = from_point.Y - p1.Y;

        double tmp = (v2.X * vec1.X) + (v2.Y * vec1.Y);
        double v1_len_2 = vec1.X * vec1.X + vec1.Y * vec1.Y;
        tmp = tmp / v1_len_2;
        double x = vec1.X * tmp;
        double y = vec1.Y * tmp;

        ClipperLib::IntPoint projection_vector;
        projection_vector.X = std::round(x);
        projection_vector.Y = std::round(y);

        ClipperLib::IntPoint projected_point;
        projected_point.X = p1.X + projection_vector.X;
        projected_point.Y = p1.Y + projection_vector.Y;

        ClipperLib::IntPoint vec_inward;
        vec_inward.X = projected_point.X - from_point.X;
        vec_inward.Y = projected_point.Y - from_point.Y;

        ClipperLib::IntPoint other_point;
        other_point.X = from_point.X + vec_inward.X * 2;
        other_point.Y = from_point.Y + vec_inward.Y * 2;

        // if two lines don't intersect, skip
        double distance = std::sqrt((vec_inward.X * vec_inward.X) + (vec_inward.Y * vec_inward.Y));
        if (distance >= closest_distance)
        {
#ifndef NDEBUG
            std::cout << "!!!!! distance too long, skip" << std::endl;
#endif // NDEBUG
        }
        else if (is_point_on_line_segment(projected_point, p1, p2))
        {
#ifndef NDEBUG
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! ok!" << std::endl;
#endif // NDEBUG
            // set the nearest point data
            nearest_point = projected_point;
            start_index = current_index;
            end_index = current_index + 1;
            found_nearest_point = true;
        }
        else
        {
#ifndef NDEBUG
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! not intersecting, skip" << std::endl;
            std::cout << " -- p1 = " << p1 << " , p2 = " << p2 << std::endl;
            std::cout << " -- from = " << from_point << std::endl;
            std::cout << " -- projection vector = " << projection_vector << std::endl;
            std::cout << " -- projection point = " << projected_point << std::endl;
            std::cout << " -- inward vector = " << vec_inward << std::endl;
            std::cout << " -- other point = " << other_point << std::endl;
#endif // NDEBUG
        }

        ++current_index;
    }
#ifndef NDEBUG
    std::cout << "!!!!! path size = " << to_path.size() << std::endl;
#endif // NDEBUG

    {
        const ClipperLib::IntPoint& p1 = to_path[to_path.size() - 1];
        const ClipperLib::IntPoint& p2 = to_path[0];
        ClipperLib::IntPoint vec1;
        vec1.X = p2.X - p1.X;
        vec1.Y = p2.Y - p1.Y;

        // compute projection of vector p1 -> from_point on vec1
        ClipperLib::IntPoint v2;
        v2.X = from_point.X - p1.X;
        v2.Y = from_point.Y - p1.Y;

        double tmp = (v2.X * vec1.X) + (v2.Y * vec1.Y);
        double v1_len_2 = vec1.X * vec1.X + vec1.Y * vec1.Y;
        tmp = tmp / v1_len_2;
        double x = vec1.X * tmp;
        double y = vec1.Y * tmp;

        ClipperLib::IntPoint projection_vector;
        projection_vector.X = std::round(x);
        projection_vector.Y = std::round(y);

        ClipperLib::IntPoint projected_point;
        projected_point.X = p1.X + projection_vector.X;
        projected_point.Y = p1.Y + projection_vector.Y;

        ClipperLib::IntPoint vec_inward;
        vec_inward.X = projected_point.X - from_point.X;
        vec_inward.Y = projected_point.Y - from_point.Y;

        ClipperLib::IntPoint other_point;
        other_point.X = from_point.X + vec_inward.X * 2;
        other_point.Y = from_point.Y + vec_inward.Y * 2;

        // if two lines don't intersect, skip
        double distance = std::sqrt((vec_inward.X * vec_inward.X) + (vec_inward.Y * vec_inward.Y));
        if (distance >= closest_distance)
        {
#ifndef NDEBUG
            std::cout << "!!!!! distance too long, skip" << std::endl;
#endif // NDEBUG
        }
        else if (is_point_on_line_segment(projected_point, p1, p2))
        {
#ifndef NDEBUG
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! ok!" << std::endl;
#endif // NDEBUG
            // set the nearest point data
            nearest_point = projected_point;
            start_index = to_path.size() - 1; // TODO: should fix
            end_index = 0;
            found_nearest_point = true;
        }
        else
        {
#ifndef NDEBUG
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! not intersecting, skip" << std::endl;
            std::cout << " -- p1 = " << p1 << " , p2 = " << p2 << std::endl;
            std::cout << " -- from = " << from_point << std::endl;
            std::cout << " -- projection vector = " << projection_vector << std::endl;
            std::cout << " -- projection point = " << projected_point << std::endl;
            std::cout << " -- inward vector = " << vec_inward << std::endl;
            std::cout << " -- other point = " << other_point << std::endl;
#endif // NDEBUG
        }
    }

    return found_nearest_point;
}


static inline bool get_point_on_path_after_distance(
    ClipperLib::IntPoint& result_point,
    uint64_t& range_begin_index,
    uint64_t& range_end_index,
    const ClipperLib::Path& path,
    uint64_t start_point_index,
    int64_t distance)
{
    assert(start_point_index < path.size());
    if (start_point_index >= path.size())
    {
        start_point_index = start_point_index % path.size();
    }

    bool got_point = false;
    double accumulated_distance = 0;
    range_begin_index = start_point_index;
    range_end_index = range_begin_index + 1;

    while (!got_point)
    {
        for (uint64_t i = start_point_index; i < path.size(); ++i)
        {
            if (i + 1 >= path.size())
            {
                break;
            }
            const ClipperLib::IntPoint& start_point = path[i];
            const ClipperLib::IntPoint& end_point = path[i + 1];

            const double line_length = p2p_dist(start_point, end_point);
            accumulated_distance += line_length;
            if (std::round(accumulated_distance) >= distance)
            {
                // chop off
                const double distance_to_go = accumulated_distance - distance;

                const double unit_a = (end_point.X - start_point.X) / line_length;
                const double unit_b = (end_point.Y - start_point.Y) / line_length;
                const int64_t delta_x = std::round(distance_to_go * unit_a);
                const int64_t delta_y = std::round(distance_to_go * unit_b);

                result_point.X = delta_x;
                result_point.Y = delta_y;

                range_begin_index = i;
                range_end_index = i + 1;
                got_point = true;
                break;
            }
        }

        if (got_point)
        {
            break;
        }
    }

    return got_point;
}

// ----------------------------------

 
// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
static inline bool onSegment(const ClipperLib::IntPoint& p, const ClipperLib::IntPoint& q, const ClipperLib::IntPoint& r)
{
    if (q.X <= std::max(p.X, r.X) && q.X >= std::min(p.X, r.X) &&
        q.Y <= std::max(p.Y, r.Y) && q.Y >= std::min(p.Y, r.Y))
       return true;
 
    return false;
}
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
static inline int orientation(const ClipperLib::IntPoint& p, const ClipperLib::IntPoint& q, const ClipperLib::IntPoint& r)
{
    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    int val = (q.Y - p.Y) * (r.X - q.X) -
              (q.X - p.X) * (r.Y - q.Y);
 
    if (val == 0) return 0;  // colinear
 
    return (val > 0) ? 1: 2; // clock or counterclock wise
}
 
// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
static inline bool doIntersect(const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& q1, const ClipperLib::IntPoint& p2, const ClipperLib::IntPoint& q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 and o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 and onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 and onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 and onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 and onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}


static inline void get_closest_point_on_path_from_point(
    ClipperLib::IntPoint& result_point,
    uint64_t& result_point_index,
    const ClipperLib::IntPoint& from_point,
    const ClipperLib::Path& to_path)
{
    double closest_distance = std::numeric_limits<double>::max();
    uint64_t current_index = 0;
    for (auto itr_pt = to_path.begin(); itr_pt != to_path.end(); ++itr_pt)
    {
        double distance = p2p_dist(from_point, *itr_pt);
        if (distance < closest_distance)
        {
            closest_distance = distance;
            result_point = *itr_pt;
            result_point_index = current_index;
        }
        ++current_index;
    }
}



} // namespace cura

#endif // CURA_INFILL_MATH_H
