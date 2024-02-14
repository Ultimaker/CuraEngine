// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_POLYGON_H
#define UTILS_POLYGON_H

#include <assert.h>
#include <cmath>
#include <limits>
#include <list>
#include <numeric>
#include <polyclipping/clipper.hpp>
#include <vector>

#include "../settings/types/Angle.h"
#include "../settings/types/Ratio.h"
#include "Point2LL.h"
#include "utils/ListPolyIt.h"
#include "utils/ShapeType.h"
#include "utils/linearAlg2D.h"

#define CHECK_POLY_ACCESS
#ifdef CHECK_POLY_ACCESS
#define POLY_ASSERT(e) assert(e)
#else
#define POLY_ASSERT(e) \
    do \
    { \
    } while (0)
#endif

namespace cura
{

using point_t = ClipperLib::IntPoint;
using path_t = ClipperLib::Path;
using paths_t = ClipperLib::Paths;

template<typename T>
bool shorterThan(const T& shape, const coord_t check_length)
{
    const auto* p0 = &shape.back();
    int64_t length = 0;
    for (const auto& p1 : shape)
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

class PartsView;
class Polygons;
class OpenPolyline;
class ListPolyIt;

const static int clipper_init = (0);
#define NO_INDEX (std::numeric_limits<size_t>::max())

class PointsSet : public std::vector<point_t>
{
public:
    PointsSet() = default;

    PointsSet(const std::initializer_list<point_t>& initializer)
        : std::vector<point_t>(initializer)
    {
    }

    PointsSet(const std::vector<point_t>& points)
        : std::vector<point_t>(points)
    {
    }

    PointsSet(std::vector<point_t>&& points)
        : std::vector<point_t>(std::move(points))
    {
    }

    /*PointsSet& operator=(const PointsSet& other)
    {
        std::vector<point_t>::operator=(other);
        return *this;
    }*/

    Point2LL min() const;

    Point2LL max() const;

    Point2LL closestPointTo(Point2LL p) const;

    /*!
     * Translate the whole polygon in some direction.
     *
     * \param translation The direction in which to move the polygon
     */
    void translate(Point2LL translation)
    {
        for (Point2LL& p : *this)
        {
            p += translation;
        }
    }

    /*!
     * Apply a matrix to each vertex in this set
     */
    void applyMatrix(const PointMatrix& matrix);
    void applyMatrix(const Point3Matrix& matrix);
};

// Transitory structure used to iterate over segments within a polygon/polyline
template<bool IsConst>
struct Segment
{
    using PointType = typename std::conditional<IsConst, const point_t, point_t>::type;

    PointType& start;
    PointType& end;
};

// Custom iterator to loop over the segments of an existing polygon/polyline
template<bool IsConst>
struct SegmentIterator
{
    using iterator_category = std::random_access_iterator_tag;
    using value_type = Segment<IsConst>;
    using difference_type = std::ptrdiff_t;
    using pointer = Segment<IsConst>*;
    using reference = Segment<IsConst>&;
    using source_iterator_type = typename std::conditional<IsConst, typename std::vector<point_t>::const_iterator, typename std::vector<point_t>::iterator>::type;

private:
    source_iterator_type current_pos_;
    source_iterator_type begin_;
    source_iterator_type before_end_;

public:
    SegmentIterator(source_iterator_type pos, source_iterator_type begin, source_iterator_type end)
        : current_pos_(pos)
        , begin_(begin)
        , before_end_(end != begin ? std::prev(end) : end)
    {
    }

    Segment<IsConst> operator*() const
    {
        if (current_pos_ == before_end_)
        {
            return Segment<IsConst>{ *current_pos_, *begin_ };
        }
        else
        {
            return Segment<IsConst>{ *current_pos_, *std::next(current_pos_) };
        }
    }

    SegmentIterator& operator++()
    {
        current_pos_++;
        return *this;
    }

    bool operator==(const SegmentIterator& other) const
    {
        return current_pos_ == other.current_pos_;
    }

    bool operator!=(const SegmentIterator& other) const
    {
        return ! (*this == other);
    }

    friend difference_type operator-(const SegmentIterator<IsConst>& iterator1, const SegmentIterator<IsConst>& iterator2)
    {
        return iterator1.current_pos_ - iterator2.current_pos_;
    }
};

template<ShapeType ShapeTypeVal>
class Polyline : public PointsSet
{
    friend class Polygons;

public:
    static constexpr ShapeType shape_type_ = ShapeTypeVal;

public:
    using segments_iterator = SegmentIterator<false>;
    using const_segments_iterator = SegmentIterator<true>;

    Polyline() = default;

    Polyline(const std::initializer_list<point_t>& initializer)
        : PointsSet(initializer)
    {
    }

    Polyline(const std::vector<point_t>& points)
        : PointsSet(points)
    {
    }

    Polyline(std::vector<point_t>&& points)
        : PointsSet(points)
    {
    }

    /*Polyline& operator=(const Polyline& other)
    {
        std::vector<point_t>::operator=(other);
        return *this;
    }*/

    const_segments_iterator beginSegments() const;

    const_segments_iterator endSegments() const;

    segments_iterator beginSegments();

    segments_iterator endSegments();

    /*!
     * Split these poly line objects into several line segment objects consisting of only two verts
     * and store them in the \p result
     */
    void splitIntoSegments(std::vector<OpenPolyline>& result) const;
    std::vector<OpenPolyline> splitIntoSegments() const;

    /*!
     * On Y-axis positive upward displays, Orientation will return true if the polygon's orientation is counter-clockwise.
     *
     * from http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/Orientation.htm
     */
    bool orientation() const
    {
        return ClipperLib::Orientation(*this);
    }

    coord_t length() const;

    bool shorterThan(const coord_t check_length) const;

    void reverse()
    {
        ClipperLib::ReversePath(*this);
    }

    void removeColinearEdges(const AngleRadians max_deviation_angle);

    /*!
     * Removes consecutive line segments with same orientation and changes this polygon.
     *
     * 1. Removes verts which are connected to line segments which are too small.
     * 2. Removes verts which detour from a direct line from the previous and next vert by a too small amount.
     * 3. Moves a vert when a small line segment is connected to a much longer one. in order to maintain the outline of the object.
     * 4. Don't remove a vert when the impact on the outline of the object is too great.
     *
     * Note that the simplify is a best effort algorithm. It does not guarantee that no lines below the provided smallest_line_segment_squared are left.
     *
     * The following example (Two very long line segments (" & , respectively) that are connected by a very small line segment (i) is unsimplifable by this
     * function, even though the actual area change of removing line segment i is very small. The reason for this is that in the case of long lines, even a small
     * deviation from it's original direction is very noticeable in the final result, especially if the polygons above make a slightly different choice.
     *
     * """"""""""""""""""""""""""""""""i,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

     *
     * \param smallest_line_segment_squared maximal squared length of removed line segments
     * \param allowed_error_distance_squared The square of the distance of the middle point to the line segment of the consecutive and previous point for which the middle point is
     removed
     */
    void simplify(const coord_t smallest_line_segment_squared = MM2INT(0.01) * MM2INT(0.01), const coord_t allowed_error_distance_squared = 25);

    /*!
     * See simplify(.)
     */
#warning This can probably be merge with simplify ?
    void simplifyPolyline(const coord_t smallest_line_segment_squared = 100, const coord_t allowed_error_distance_squared = 25);

private:
    /*!
     * Private implementation for both simplify and simplifyPolygons.
     *
     * Made private to avoid accidental use of the wrong function.
     */
    void _simplify(const coord_t smallest_line_segment_squared = 100, const coord_t allowed_error_distance_squared = 25, bool processing_polylines = false);
};

template<ShapeType ShapeTypeVal = ShapeType::Closed>
class _ClosedPolyline : public Polyline<ShapeTypeVal>
{
    friend class Polygons;

public:
    _ClosedPolyline() = default;

    _ClosedPolyline(const std::initializer_list<point_t>& initializer)
        : Polyline<ShapeTypeVal>(initializer)
    {
    }

    _ClosedPolyline(const std::vector<point_t>& points)
        : Polyline<ShapeTypeVal>(points)
    {
    }

    _ClosedPolyline& operator=(const _ClosedPolyline& other)
    {
        Polyline<ShapeTypeVal>::operator=(other);
        return *this;
    }

    /*!
     * Check if we are inside the polygon. We do this by tracing from the point towards the positive X direction,
     * every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
     * Care needs to be taken, if p.Y exactly matches a vertex to the right of p, then we need to count 1 intersect if the
     * outline passes vertically past; and 0 (or 2) intersections if that point on the outline is a 'top' or 'bottom' vertex.
     * The easiest way to do this is to break out two cases for increasing and decreasing Y ( from p0 to p1 ).
     * A segment is tested if pa.Y <= p.Y < pb.Y, where pa and pb are the points (from p0,p1) with smallest & largest Y.
     * When both have the same Y, no intersections are counted but there is a special test to see if the point falls
     * exactly on the line.
     *
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     *
     * \deprecated This function is no longer used, since the Clipper function is used by the function PolygonRef::inside(.)
     *
     * \param p The point for which to check if it is inside this polygon
     * \param border_result What to return when the point is exactly on the border
     * \return Whether the point \p p is inside this polygon (or \p border_result when it is on the border)
     */
    // bool _inside(Point2LL p, bool border_result = false) const;

    /*!
     * Clipper function.
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     *
     * http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PointInPolygon.htm
     */
    bool inside(Point2LL p, bool border_result = false) const
    {
        int res = ClipperLib::PointInPolygon(p, *this);
        if (res == -1)
        {
            return border_result;
        }
        return res == 1;
    }

    bool inside(const auto& polygon) const
    {
        for (const auto& point : *this)
        {
            if (! ClipperLib::PointInPolygon(point, polygon))
            {
                return false;
            }
        }
        return true;
    }
};

using ClosedPolyline = _ClosedPolyline<ShapeType::Closed>;

class Polygon : public _ClosedPolyline<ShapeType::Filled>
{
    friend class Polygons;

public:
    Polygon() = default;

    Polygon(const Polygon& other) = default;

    Polygon(Polygon&& other) = default;

    Polygon(const std::initializer_list<point_t>& initializer)
        : _ClosedPolyline<ShapeType::Filled>(initializer)
    {
    }

    Polygon(const std::vector<point_t>& points)
        : _ClosedPolyline<ShapeType::Filled>(points)
    {
    }

    Polygon& operator=(const Polygon& other)
    {
        _ClosedPolyline::operator=(other);
        return *this;
    }

    /*!
     * Compute the morphological intersection between this polygon and another.
     *
     * Note that the result may consist of multiple polygons, if you have bad
     * luck.
     *
     * \param other The polygon with which to intersect this polygon.
     */
    Polygons intersection(const Polygon& other) const;

    double area() const
    {
        return ClipperLib::Area(*this);
    }

    Point2LL centerOfMass() const;

    Polygons offset(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    /*!
     * Smooth out small perpendicular segments and store the result in \p result.
     * Smoothing is performed by removing the inner most vertex of a line segment smaller than \p remove_length
     * which has an angle with the next and previous line segment smaller than roughly 150*
     *
     * Note that in its current implementation this function doesn't remove line segments with an angle smaller than 30*
     * Such would be the case for an N shape.
     *
     * \param remove_length The length of the largest segment removed
     * \param result (output) The result polygon, assumed to be empty
     */
    void smooth(int remove_length, Polygon& result) const;

    /*!
     * Smooth out sharp inner corners, by taking a shortcut which bypasses the corner
     *
     * \param angle The maximum angle of inner corners to be smoothed out
     * \param shortcut_length The desired length of the shortcut line segment introduced (shorter shortcuts may be unavoidable)
     * \param result The resulting polygon
     */
    void smooth_outward(const AngleDegrees angle, int shortcut_length, Polygon& result) const;

    /*!
     * Smooth out the polygon and store the result in \p result.
     * Smoothing is performed by removing vertices for which both connected line segments are smaller than \p remove_length
     *
     * \param remove_length The length of the largest segment removed
     * \param result (output) The result polygon, assumed to be empty
     */
    void smooth2(int remove_length, Polygon& result) const;

    /*!
     * Smooth out a simple corner consisting of two linesegments.
     *
     * Auxiliary function for \ref smooth_outward
     *
     * \param p0 The point before the corner
     * \param p1 The corner
     * \param p2 The point after the corner
     * \param p0_it Iterator to the point before the corner
     * \param p1_it Iterator to the corner
     * \param p2_it Iterator to the point after the corner
     * \param v10 Vector from \p p1 to \p p0
     * \param v12 Vector from \p p1 to \p p2
     * \param v02 Vector from \p p0 to \p p2
     * \param shortcut_length The desired length ofthe shortcutting line
     * \param cos_angle The cosine on the angle in L 012
     */
    static void smooth_corner_simple(
        const Point2LL p0,
        const Point2LL p1,
        const Point2LL p2,
        const ListPolyIt p0_it,
        const ListPolyIt p1_it,
        const ListPolyIt p2_it,
        const Point2LL v10,
        const Point2LL v12,
        const Point2LL v02,
        const int64_t shortcut_length,
        double cos_angle);

    /*!
     * Smooth out a complex corner where the shortcut bypasses more than two line segments
     *
     * Auxiliary function for \ref smooth_outward
     *
     * \warning This function might try to remove the whole polygon
     * Error code -1 means the whole polygon should be removed (which means it is a hole polygon)
     *
     * \param p1 The corner point
     * \param[in,out] p0_it Iterator to the last point checked before \p p1 to consider cutting off
     * \param[in,out] p2_it Iterator to the last point checked after \p p1 to consider cutting off
     * \param shortcut_length The desired length ofthe shortcutting line
     * \return Whether this whole polygon whould be removed by the smoothing
     */
    static bool smooth_corner_complex(const Point2LL p1, ListPolyIt& p0_it, ListPolyIt& p2_it, const int64_t shortcut_length);

    /*!
     * Try to take a step away from the corner point in order to take a bigger shortcut.
     *
     * Try to take the shortcut from a place as far away from the corner as the place we are taking the shortcut to.
     *
     * Auxiliary function for \ref smooth_outward
     *
     * \param[in] p1 The corner point
     * \param[in] shortcut_length2 The square of the desired length ofthe shortcutting line
     * \param[in,out] p0_it Iterator to the previously checked point somewhere beyond \p p1. Updated for the next iteration.
     * \param[in,out] p2_it Iterator to the previously checked point somewhere before \p p1. Updated for the next iteration.
     * \param[in,out] forward_is_blocked Whether trying another step forward is blocked by the smoothing outward condition. Updated for the next iteration.
     * \param[in,out] backward_is_blocked Whether trying another step backward is blocked by the smoothing outward condition. Updated for the next iteration.
     * \param[in,out] forward_is_too_far Whether trying another step forward is blocked by the shortcut length condition. Updated for the next iteration.
     * \param[in,out] backward_is_too_far Whether trying another step backward is blocked by the shortcut length condition. Updated for the next iteration.
     */
    static void smooth_outward_step(
        const Point2LL p1,
        const int64_t shortcut_length2,
        ListPolyIt& p0_it,
        ListPolyIt& p2_it,
        bool& forward_is_blocked,
        bool& backward_is_blocked,
        bool& forward_is_too_far,
        bool& backward_is_too_far);
};

class OpenPolyline : public Polyline<ShapeType::Open>
{
public:
    OpenPolyline() = default;

    OpenPolyline(const OpenPolyline& other) = default;

    OpenPolyline(OpenPolyline&& other) = default;

    OpenPolyline(const std::initializer_list<point_t>& initializer)
        : Polyline<ShapeType::Open>(initializer)
    {
    }

    OpenPolyline(const std::vector<point_t>& points)
        : Polyline<ShapeType::Open>(points)
    {
    }

    OpenPolyline(std::vector<point_t>&& points)
        : Polyline<ShapeType::Open>(points)
    {
    }

    OpenPolyline& operator=(const OpenPolyline& other)
    {
        Polyline<ShapeType::Open>::operator=(other);
        return *this;
    }

    OpenPolyline& operator=(OpenPolyline&& other)
    {
        Polyline<ShapeType::Open>::operator=(other);
        return *this;
    }
};

#if 0
/*!
 * Outer polygons should be counter-clockwise,
 * inner hole polygons should be clockwise.
 * (When negative X is to the left and negative Y is downward.)
 */
class ConstPolygonRef
{

};

class PolygonsPart;

class Polygons
{

    Polygons& operator=(const Polygons& other)
    {
        paths = other.paths;
        return *this;
    }
    Polygons& operator=(Polygons&& other)
    {
        if (this != &other)
        {
            paths = std::move(other.paths);
        }
        return *this;
    }  
};
#endif

// ###########################################################
// Definitions of templated methods
// ###########################################################
#if 0
seems to be unused
template<ShapeType ShapeTypeVal>
bool SurfaceContainer<ShapeTypeVal>::_inside(Point2LL p, bool border_result) const
{
    if (size() < 1)
    {
        return false;
    }

    int crossings = 0;
    Point2LL p0 = back();
    for (unsigned int n = 0; n < size(); n++)
    {
        Point2LL p1 = (*this)[n];
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
#endif

template<ShapeType ShapeTypeVal>
void Polyline<ShapeTypeVal>::removeColinearEdges(const AngleRadians max_deviation_angle)
{
    // TODO: Can be made more efficient (for example, use pointer-types for process-/skip-indices, so we can swap them without copy).

    size_t num_removed_in_iteration = 0;
    do
    {
        num_removed_in_iteration = 0;

        std::vector<bool> process_indices(size(), true);

        bool go = true;
        while (go)
        {
            go = false;

            const path_t& rpath = *this;
            const size_t pathlen = rpath.size();
            if (pathlen <= 3)
            {
                return;
            }

            std::vector<bool> skip_indices(size(), false);

            Polyline<ShapeTypeVal> new_path;
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

                const Point2LL& prev = rpath[(point_idx - 1 + pathlen) % pathlen];
                const Point2LL& pt = rpath[point_idx];
                const Point2LL& next = rpath[(point_idx + 1) % pathlen];

                double angle = LinearAlg2D::getAngleLeft(prev, pt, next); // [0 : 2 * pi]
                if (angle >= std::numbers::pi)
                {
                    angle -= std::numbers::pi;
                } // map [pi : 2 * pi] to [0 : pi]

                // Check if the angle is within limits for the point to 'make sense', given the maximum deviation.
                // If the angle indicates near-parallel segments ignore the point 'pt'
                if (angle > max_deviation_angle && angle < std::numbers::pi - max_deviation_angle)
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
            (*this) = new_path;
            num_removed_in_iteration += pathlen - size();

            process_indices.clear();
            process_indices.insert(process_indices.end(), skip_indices.begin(), skip_indices.end());
        }
    } while (num_removed_in_iteration > 0);
}

template<ShapeType ShapeTypeVal>
Polyline<ShapeTypeVal>::const_segments_iterator Polyline<ShapeTypeVal>::beginSegments() const
{
    return const_segments_iterator(begin(), begin(), end());
}

template<ShapeType ShapeTypeVal>
Polyline<ShapeTypeVal>::const_segments_iterator Polyline<ShapeTypeVal>::endSegments() const
{
    if constexpr (shape_type_ == ShapeType::Closed || shape_type_ == ShapeType::Filled)
    {
        return const_segments_iterator(end(), begin(), end());
    }
    else
    {
        return const_segments_iterator(size() > 1 ? std::prev(end()) : end(), begin(), end());
    }
}

template<ShapeType ShapeTypeVal>
Polyline<ShapeTypeVal>::segments_iterator Polyline<ShapeTypeVal>::beginSegments()
{
    return segments_iterator(begin(), begin(), end());
}

template<ShapeType ShapeTypeVal>
Polyline<ShapeTypeVal>::segments_iterator Polyline<ShapeTypeVal>::endSegments()
{
    if constexpr (shape_type_ == ShapeType::Closed || shape_type_ == ShapeType::Filled)
    {
        return segments_iterator(end(), begin(), end());
    }
    else
    {
        return segments_iterator(size() > 1 ? std::prev(end()) : end(), begin(), end());
    }
}

template<ShapeType ShapeTypeVal>
coord_t Polyline<ShapeTypeVal>::length() const
{
    return std::accumulate(
        beginSegments(),
        endSegments(),
        0,
        [](coord_t total, const const_segments_iterator::value_type& segment)
        {
            return total + vSize(segment.end - segment.start);
        });
}

template<ShapeType ShapeTypeVal>
bool Polyline<ShapeTypeVal>::shorterThan(const coord_t check_length) const
{
    coord_t length = 0;
    auto iterator_segment = std::find_if(
        beginSegments(),
        endSegments(),
        [&length, &check_length](const const_segments_iterator::value_type& segment)
        {
            length += vSize(segment.end - segment.start);
            if (length >= check_length)
            {
                return true;
            }
        });
    return iterator_segment == endSegments();
}

template<ShapeType ShapeTypeVal>
void Polyline<ShapeTypeVal>::splitIntoSegments(std::vector<OpenPolyline>& result) const
{
    for (auto it = beginSegments(); it != endSegments(); ++it)
    {
        result.emplace_back(std::initializer_list<point_t>{ (*it).start, (*it).end });
    }
}

template<ShapeType ShapeTypeVal>
std::vector<OpenPolyline> Polyline<ShapeTypeVal>::splitIntoSegments() const
{
    std::vector<OpenPolyline> result;
    splitIntoSegments(result);
    return result;
}

} // namespace cura

namespace std
{
#if 0
template<>
struct hash<cura::Polygon*>
{
    size_t operator()(const cura::PolygonPointer& poly) const
    {
        const cura::ConstPolygonRef ref = *static_cast<cura::PolygonPointer>(poly);
        return std::hash<const ClipperLib::Path*>()(&*ref);
    }
};
#endif
} // namespace std

#endif // UTILS_POLYGON_H
