// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_POLYGON_H
#define UTILS_POLYGON_H

#include "../settings/types/Angle.h" //For angles between vertices.
#include "../settings/types/Ratio.h"
#include "IntPoint.h"

#include <algorithm>
#include <algorithm> // std::reverse, fill_n array
#include <assert.h>
#include <cmath> // fabs
#include <float.h>
#include <initializer_list>
#include <limits> // int64_t.min
#include <list>
#include <polyclipping/clipper.hpp>
#include <unordered_map>
#include <vector>

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
class Polygon;
class PolygonRef;

class ListPolyIt;

typedef std::list<Point> ListPolygon; //!< A polygon represented by a linked list instead of a vector
typedef std::vector<ListPolygon> ListPolygons; //!< Polygons represented by a vector of linked lists instead of a vector of vectors

const static int clipper_init = (0);
#define NO_INDEX (std::numeric_limits<unsigned int>::max())

class ConstPolygonPointer;

/*!
 * Outer polygons should be counter-clockwise,
 * inner hole polygons should be clockwise.
 * (When negative X is to the left and negative Y is downward.)
 */
class ConstPolygonRef
{
    friend class Polygons;
    friend class Polygon;
    friend class PolygonRef;
    friend class ConstPolygonPointer;

protected:
    ClipperLib::Path* path;

public:
    ConstPolygonRef(const ClipperLib::Path& polygon)
        : path(const_cast<ClipperLib::Path*>(&polygon))
    {
    }

    ConstPolygonRef() = delete; // you cannot have a reference without an object!

    virtual ~ConstPolygonRef()
    {
    }

    bool operator==(ConstPolygonRef& other) const = delete; // polygon comparison is expensive and probably not what you want when you use the equality operator

    ConstPolygonRef& operator=(const ConstPolygonRef& other) = delete; // Cannot assign to a const object

    /*!
     * Gets the number of vertices in this polygon.
     * \return The number of vertices in this polygon.
     */
    size_t size() const;

    /*!
     * Returns whether there are any vertices in this polygon.
     * \return ``true`` if the polygon has no vertices at all, or ``false`` if
     * it does have vertices.
     */
    bool empty() const;

    const Point& operator[](unsigned int index) const
    {
        POLY_ASSERT(index < size());
        return (*path)[index];
    }

    const ClipperLib::Path& operator*() const
    {
        return *path;
    }

    ClipperLib::Path::const_iterator begin() const
    {
        return path->begin();
    }

    ClipperLib::Path::const_iterator end() const
    {
        return path->end();
    }

    ClipperLib::Path::const_reverse_iterator rbegin() const
    {
        return path->rbegin();
    }

    ClipperLib::Path::const_reverse_iterator rend() const
    {
        return path->rend();
    }

    ClipperLib::Path::const_reference front() const
    {
        return path->front();
    }

    ClipperLib::Path::const_reference back() const
    {
        return path->back();
    }

    const void* data() const
    {
        return path->data();
    }


    /*!
     * On Y-axis positive upward displays, Orientation will return true if the polygon's orientation is counter-clockwise.
     *
     * from http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/Orientation.htm
     */
    bool orientation() const
    {
        return ClipperLib::Orientation(*path);
    }

    Polygons offset(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    coord_t polygonLength() const
    {
        return polylineLength() + vSize(path->front() - path->back());
    }

    coord_t polylineLength() const
    {
        coord_t length = 0;
        Point p0 = path->front();
        for (unsigned int n = 1; n < path->size(); n++)
        {
            Point p1 = (*path)[n];
            length += vSize(p0 - p1);
            p0 = p1;
        }
        return length;
    }

    /*!
     * Split these poly line objects into several line segment objects consisting of only two verts
     * and store them in the \p result
     */
    void splitPolylineIntoSegments(Polygons& result) const;
    Polygons splitPolylineIntoSegments() const;

    /*!
     * Split these polygon objects into several line segment objects consisting of only two verts
     * and store them in the \p result
     */
    void splitPolygonIntoSegments(Polygons& result) const;
    Polygons splitPolygonIntoSegments() const;

    bool shorterThan(const coord_t check_length) const;

    Point min() const
    {
        Point ret = Point(POINT_MAX, POINT_MAX);
        for (Point p : *path)
        {
            ret.X = std::min(ret.X, p.X);
            ret.Y = std::min(ret.Y, p.Y);
        }
        return ret;
    }

    Point max() const
    {
        Point ret = Point(POINT_MIN, POINT_MIN);
        for (Point p : *path)
        {
            ret.X = std::max(ret.X, p.X);
            ret.Y = std::max(ret.Y, p.Y);
        }
        return ret;
    }

    double area() const
    {
        return ClipperLib::Area(*path);
    }

    Point centerOfMass() const
    {
        double x = 0, y = 0;
        Point p0 = (*path)[path->size() - 1];
        for (unsigned int n = 0; n < path->size(); n++)
        {
            Point p1 = (*path)[n];
            double second_factor = (p0.X * p1.Y) - (p1.X * p0.Y);

            x += double(p0.X + p1.X) * second_factor;
            y += double(p0.Y + p1.Y) * second_factor;
            p0 = p1;
        }

        double area = Area(*path);

        x = x / 6 / area;
        y = y / 6 / area;

        return Point(x, y);
    }

    Point closestPointTo(Point p) const
    {
        Point ret = p;
        float bestDist = FLT_MAX;
        for (unsigned int n = 0; n < path->size(); n++)
        {
            float dist = vSize2f(p - (*path)[n]);
            if (dist < bestDist)
            {
                ret = (*path)[n];
                bestDist = dist;
            }
        }
        return ret;
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
    bool _inside(Point p, bool border_result = false) const;

    /*!
     * Clipper function.
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     *
     * http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PointInPolygon.htm
     */
    bool inside(Point p, bool border_result = false) const
    {
        int res = ClipperLib::PointInPolygon(p, *path);
        if (res == -1)
        {
            return border_result;
        }
        return res == 1;
    }

    bool inside(const auto& polygon) const
    {
        for (const auto& point : *path)
        {
            if (! ClipperLib::PointInPolygon(point, *polygon.path))
            {
                return false;
            }
        }
        return true;
    }

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
    void smooth(int remove_length, PolygonRef result) const;

    /*!
     * Smooth out sharp inner corners, by taking a shortcut which bypasses the corner
     *
     * \param angle The maximum angle of inner corners to be smoothed out
     * \param shortcut_length The desired length of the shortcut line segment introduced (shorter shortcuts may be unavoidable)
     * \param result The resulting polygon
     */
    void smooth_outward(const AngleDegrees angle, int shortcut_length, PolygonRef result) const;

    /*!
     * Smooth out the polygon and store the result in \p result.
     * Smoothing is performed by removing vertices for which both connected line segments are smaller than \p remove_length
     *
     * \param remove_length The length of the largest segment removed
     * \param result (output) The result polygon, assumed to be empty
     */
    void smooth2(int remove_length, PolygonRef result) const;

    /*!
     * Compute the morphological intersection between this polygon and another.
     *
     * Note that the result may consist of multiple polygons, if you have bad
     * luck.
     *
     * \param other The polygon with which to intersect this polygon.
     */
    Polygons intersection(const ConstPolygonRef& other) const;


private:
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
        const Point p0,
        const Point p1,
        const Point p2,
        const ListPolyIt p0_it,
        const ListPolyIt p1_it,
        const ListPolyIt p2_it,
        const Point v10,
        const Point v12,
        const Point v02,
        const int64_t shortcut_length,
        float cos_angle);

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
    static bool smooth_corner_complex(const Point p1, ListPolyIt& p0_it, ListPolyIt& p2_it, const int64_t shortcut_length);

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
        const Point p1,
        const int64_t shortcut_length2,
        ListPolyIt& p0_it,
        ListPolyIt& p2_it,
        bool& forward_is_blocked,
        bool& backward_is_blocked,
        bool& forward_is_too_far,
        bool& backward_is_too_far);
};


class PolygonPointer;

class PolygonRef : public ConstPolygonRef
{
    friend class PolygonPointer;
    friend class Polygons;
    friend class PolygonsPart;

public:
    PolygonRef(ClipperLib::Path& polygon)
        : ConstPolygonRef(polygon)
    {
    }

    PolygonRef(const PolygonRef& other)
        : ConstPolygonRef(*other.path)
    {
    }

    PolygonRef() = delete; // you cannot have a reference without an object!

    virtual ~PolygonRef()
    {
    }

    /*!
     * Reserve a number of polygons to prevent reallocation and breakage of pointers.
     * \param min_size The minimum size the new underlying array should have.
     */
    void reserve(size_t min_size)
    {
        path->reserve(min_size);
    }

    template<class iterator>
    ClipperLib::Path::iterator insert(ClipperLib::Path::const_iterator pos, iterator first, iterator last)
    {
        return path->insert(pos, first, last);
    }

    PolygonRef& operator=(const ConstPolygonRef& other) = delete; // polygon assignment is expensive and probably not what you want when you use the assignment operator

    PolygonRef& operator=(ConstPolygonRef& other) = delete; // polygon assignment is expensive and probably not what you want when you use the assignment operator
    //     { path = other.path; return *this; }

    PolygonRef& operator=(PolygonRef&& other)
    {
        *path = std::move(*other.path);
        return *this;
    }

    Point& operator[](unsigned int index)
    {
        POLY_ASSERT(index < size());
        return (*path)[index];
    }

    const Point& operator[](unsigned int index) const
    {
        POLY_ASSERT(index < size());
        return (*path)[index];
    }

    ClipperLib::Path::iterator begin()
    {
        return path->begin();
    }

    ClipperLib::Path::iterator end()
    {
        return path->end();
    }

    ClipperLib::Path::reference front()
    {
        return path->front();
    }

    ClipperLib::Path::reference back()
    {
        return path->back();
    }

    void* data()
    {
        return path->data();
    }

    void add(const Point p)
    {
        path->push_back(p);
    }

    ClipperLib::Path& operator*()
    {
        return *path;
    }

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        path->emplace_back(args...);
    }

    void remove(unsigned int index)
    {
        POLY_ASSERT(index < size() && index <= static_cast<unsigned int>(std::numeric_limits<int>::max()));
        path->erase(path->begin() + index);
    }

    void insert(size_t index, Point p)
    {
        POLY_ASSERT(index < size() && index <= static_cast<size_t>(std::numeric_limits<int>::max()));
        path->insert(path->begin() + index, p);
    }

    void clear()
    {
        path->clear();
    }

    void reverse()
    {
        ClipperLib::ReversePath(*path);
    }

    /*!
     * Translate the whole polygon in some direction.
     *
     * \param translation The direction in which to move the polygon
     */
    void translate(Point translation)
    {
        for (Point& p : *this)
        {
            p += translation;
        }
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
    void simplifyPolyline(const coord_t smallest_line_segment_squared = 100, const coord_t allowed_error_distance_squared = 25);

protected:
    /*!
     * Private implementation for both simplify and simplifyPolygons.
     *
     * Made private to avoid accidental use of the wrong function.
     */
    void _simplify(const coord_t smallest_line_segment_squared = 100, const coord_t allowed_error_distance_squared = 25, bool processing_polylines = false);

public:
    void pop_back()
    {
        path->pop_back();
    }

    /*!
     * Apply a matrix to each vertex in this polygon
     */
    void applyMatrix(const PointMatrix& matrix);
    void applyMatrix(const Point3Matrix& matrix);
};

class ConstPolygonPointer
{
protected:
    const ClipperLib::Path* path;

public:
    ConstPolygonPointer()
        : path(nullptr)
    {
    }
    ConstPolygonPointer(const ConstPolygonRef* ref)
        : path(ref->path)
    {
    }
    ConstPolygonPointer(const ConstPolygonRef& ref)
        : path(ref.path)
    {
    }

    ConstPolygonRef operator*() const
    {
        assert(path);
        return ConstPolygonRef(*path);
    }
    const ClipperLib::Path* operator->() const
    {
        assert(path);
        return path;
    }

    operator bool() const
    {
        return path;
    }

    bool operator==(const ConstPolygonPointer& rhs) const
    {
        return path == rhs.path;
    }
};

class PolygonPointer : public ConstPolygonPointer
{
public:
    PolygonPointer()
        : ConstPolygonPointer(nullptr)
    {
    }
    PolygonPointer(PolygonRef* ref)
        : ConstPolygonPointer(ref)
    {
    }

    PolygonPointer(PolygonRef& ref)
        : ConstPolygonPointer(ref)
    {
    }

    PolygonRef operator*()
    {
        assert(path);
        return PolygonRef(*const_cast<ClipperLib::Path*>(path));
    }

    ConstPolygonRef operator*() const
    {
        assert(path);
        return ConstPolygonRef(*path);
    }

    ClipperLib::Path* operator->()
    {
        assert(path);
        return const_cast<ClipperLib::Path*>(path);
    }

    const ClipperLib::Path* operator->() const
    {
        assert(path);
        return path;
    }

    operator bool() const
    {
        return path;
    }
};

} // namespace cura


namespace std
{
template<>
struct hash<cura::ConstPolygonRef>
{
    size_t operator()(const cura::ConstPolygonRef& poly) const
    {
        return std::hash<const ClipperLib::Path*>()(&*poly);
    }
};
template<>
struct hash<cura::ConstPolygonPointer>
{
    size_t operator()(const cura::ConstPolygonPointer& poly) const
    {
        return std::hash<const ClipperLib::Path*>()(&**poly);
    }
};
template<>
struct hash<cura::PolygonPointer>
{
    size_t operator()(const cura::PolygonPointer& poly) const
    {
        const cura::ConstPolygonRef ref = *static_cast<cura::PolygonPointer>(poly);
        return std::hash<const ClipperLib::Path*>()(&*ref);
    }
};
} // namespace std

namespace cura
{

class Polygon : public PolygonRef
{
public:
    ClipperLib::Path poly;

    Polygon()
        : PolygonRef(poly)
    {
    }

    Polygon(const ConstPolygonRef& other)
        : PolygonRef(poly)
        , poly(*other.path)
    {
    }

    Polygon(const Polygon& other)
        : PolygonRef(poly)
        , poly(*other.path)
    {
    }

    Polygon(Polygon&& moved)
        : PolygonRef(poly)
        , poly(std::move(moved.poly))
    {
    }

    virtual ~Polygon()
    {
    }

    Polygon& operator=(const ConstPolygonRef& other) = delete; // copying a single polygon is generally not what you want
    //     {
    //         path = other.path;
    //         poly = *other.path;
    //         return *this;
    //     }

    Polygon& operator=(Polygon&& other) //!< move assignment
    {
        poly = std::move(other.poly);
        return *this;
    }
};

class PolygonsPart;

class Polygons
{
    friend class Polygon;
    friend class PolygonRef;
    friend class ConstPolygonRef;
    friend class PolygonUtils;

public:
    ClipperLib::Paths paths;

    unsigned int size() const
    {
        return paths.size();
    }

    void reserve(size_t new_cap)
    {
        paths.reserve(new_cap);
    }

    /*!
     * Convenience function to check if the polygon has no points.
     *
     * \return `true` if the polygon has no points, or `false` if it does.
     */
    bool empty() const;

    unsigned int pointCount() const; //!< Return the amount of points in all polygons

    PolygonRef operator[](unsigned int index)
    {
        POLY_ASSERT(index < size() && index <= static_cast<unsigned int>(std::numeric_limits<int>::max()));
        return paths[index];
    }
    ConstPolygonRef operator[](unsigned int index) const
    {
        POLY_ASSERT(index < size() && index <= static_cast<unsigned int>(std::numeric_limits<int>::max()));
        return paths[index];
    }
    ClipperLib::Paths::iterator begin()
    {
        return paths.begin();
    }
    ClipperLib::Paths::const_iterator begin() const
    {
        return paths.begin();
    }
    ClipperLib::Paths::iterator end()
    {
        return paths.end();
    }
    ClipperLib::Paths::const_iterator end() const
    {
        return paths.end();
    }
    /*!
     * Remove a polygon from the list and move the last polygon to its place
     *
     * \warning changes the order of the polygons!
     */
    void remove(unsigned int index)
    {
        POLY_ASSERT(index < size() && index <= static_cast<unsigned int>(std::numeric_limits<int>::max()));
        if (index < paths.size() - 1)
        {
            paths[index] = std::move(paths.back());
        }
        paths.resize(paths.size() - 1);
    }

    void pop_back()
    {
        paths.pop_back();
    }

    /*!
     * Remove a range of polygons
     */
    void erase(ClipperLib::Paths::iterator start, ClipperLib::Paths::iterator end)
    {
        paths.erase(start, end);
    }
    void clear()
    {
        paths.clear();
    }
    void add(ConstPolygonRef& poly)
    {
        paths.push_back(*poly.path);
    }
    void add(const ConstPolygonRef& poly)
    {
        paths.push_back(*poly.path);
    }
    void add(Polygon&& other_poly)
    {
        paths.emplace_back(std::move(*other_poly));
    }
    void add(const Polygons& other)
    {
        std::copy(other.paths.begin(), other.paths.end(), std::back_inserter(paths));
    }
    void addIfNotEmpty(ConstPolygonRef& poly)
    {
        if (! poly.empty())
        {
            paths.push_back(*poly.path);
        }
    }
    void addIfNotEmpty(const ConstPolygonRef& poly)
    {
        if (! poly.empty())
        {
            paths.push_back(*poly.path);
        }
    }
    void addIfNotEmpty(Polygon&& other_poly)
    {
        if (! other_poly.empty())
        {
            paths.emplace_back(std::move(*other_poly));
        }
    }
    /*!
     * Add a 'polygon' consisting of two points
     */
    void addLine(const Point from, const Point to)
    {
        paths.emplace_back(ClipperLib::Path{ from, to });
    }

    void emplace_back(const Polygon& poly)
    {
        paths.emplace_back(*poly.path);
    }

    void emplace_back(const ConstPolygonRef& poly)
    {
        paths.emplace_back(*poly.path);
    }

    void emplace_back(const PolygonRef& poly)
    {
        paths.emplace_back(*poly.path);
    }

    template<typename... Args>
    void emplace_back(Args... args)
    {
        paths.emplace_back(args...);
    }

    PolygonRef newPoly()
    {
        paths.emplace_back();
        return PolygonRef(paths.back());
    }
    PolygonRef front()
    {
        return PolygonRef(paths.front());
    }
    ConstPolygonRef front() const
    {
        return ConstPolygonRef(paths.front());
    }
    PolygonRef back()
    {
        return PolygonRef(paths.back());
    }
    ConstPolygonRef back() const
    {
        return ConstPolygonRef(paths.back());
    }

    Polygons()
    {
    }

    Polygons(const Polygons& other)
    {
        paths = other.paths;
    }
    Polygons(Polygons&& other)
    {
        paths = std::move(other.paths);
    }
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

    bool operator==(const Polygons& other) const = delete;

    /*!
     * Convert ClipperLib::PolyTree to a Polygons object,
     * which uses ClipperLib::Paths instead of ClipperLib::PolyTree
     */
    static Polygons toPolygons(ClipperLib::PolyTree& poly_tree);

    Polygons difference(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
        clipper.Execute(ClipperLib::ctDifference, ret.paths);
        return ret;
    }
    Polygons unionPolygons(const Polygons& other, ClipperLib::PolyFillType fill_type = ClipperLib::pftNonZero) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.AddPaths(other.paths, ClipperLib::ptSubject, true);
        clipper.Execute(ClipperLib::ctUnion, ret.paths, fill_type, fill_type);
        return ret;
    }
    /*!
     * Union all polygons with each other (When polygons.add(polygon) has been called for overlapping polygons)
     */
    Polygons unionPolygons() const
    {
        return unionPolygons(Polygons());
    }
    Polygons intersection(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
        clipper.Execute(ClipperLib::ctIntersection, ret.paths);
        return ret;
    }


    /*!
     * Intersect polylines with this area Polygons object.
     *
     * \note Due to a clipper bug with polylines with nearly collinear segments, the polylines are cut up into separate polylines, and restitched back together at the end.
     *
     * \param polylines The (non-closed!) polylines to limit to the area of this Polygons object
     * \param restitch Whether to stitch the resulting segments into longer polylines, or leave every segment as a single segment
     * \param max_stitch_distance The maximum distance for two polylines to be stitched together with a segment
     * \return The resulting polylines limited to the area of this Polygons object
     */
    Polygons intersectionPolyLines(const Polygons& polylines, bool restitch = true, const coord_t max_stitch_distance = 10_mu) const;

    /*!
     * Add the front to each polygon so that the polygon is represented as a polyline
     */
    void toPolylines();

    /*!
     * Split this poly line object into several line segment objects
     * and store them in the \p result
     */
    void splitPolylinesIntoSegments(Polygons& result) const;
    Polygons splitPolylinesIntoSegments() const;

    /*!
     * Split this polygon object into several line segment objects
     * and store them in the \p result
     */
    void splitPolygonsIntoSegments(Polygons& result) const;
    Polygons splitPolygonsIntoSegments() const;

    Polygons xorPolygons(const Polygons& other, ClipperLib::PolyFillType pft = ClipperLib::pftEvenOdd) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
        clipper.Execute(ClipperLib::ctXor, ret.paths, pft);
        return ret;
    }

    Polygons execute(ClipperLib::PolyFillType pft = ClipperLib::pftEvenOdd) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.Execute(ClipperLib::ctXor, ret.paths, pft);
        return ret;
    }

    Polygons offset(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    Polygons offsetPolyLine(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, bool inputPolyIsClosed = false) const
    {
        Polygons ret;
        double miterLimit = 1.2;
        ClipperLib::EndType end_type;
        if (inputPolyIsClosed)
        {
            end_type = ClipperLib::etClosedLine;
        }
        else if (joinType == ClipperLib::jtMiter)
        {
            end_type = ClipperLib::etOpenSquare;
        }
        else
        {
            end_type = ClipperLib::etOpenRound;
        }
        ClipperLib::ClipperOffset clipper(miterLimit, 10.0);
        clipper.AddPaths(paths, joinType, end_type);
        clipper.MiterLimit = miterLimit;
        clipper.Execute(ret.paths, distance);
        return ret;
    }

    /*!
     * Check if we are inside the polygon.
     *
     * We do this by counting the number of polygons inside which this point lies.
     * An odd number is inside, while an even number is outside.
     *
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return \p border_result.
     *
     * \param p The point for which to check if it is inside this polygon
     * \param border_result What to return when the point is exactly on the border
     * \return Whether the point \p p is inside this polygon (or \p border_result when it is on the border)
     */
    bool inside(Point p, bool border_result = false) const;

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
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return \p border_result.
     *
     * \deprecated This function is old and no longer used. instead use \ref Polygons::inside
     *
     * \param p The point for which to check if it is inside this polygon
     * \param border_result What to return when the point is exactly on the border
     * \return Whether the point \p p is inside this polygon (or \p border_result when it is on the border)
     */
    bool insideOld(Point p, bool border_result = false) const;

    /*!
     * Find the polygon inside which point \p p resides.
     *
     * We do this by tracing from the point towards the positive X direction,
     * every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
     * We then find the polygon with an uneven number of crossings which is closest to \p p.
     *
     * If \p border_result, we return the first polygon which is exactly on \p p.
     *
     * \param p The point for which to check in which polygon it is.
     * \param border_result Whether a point exactly on a polygon counts as inside
     * \return The index of the polygon inside which the point \p p resides
     */
    unsigned int findInside(Point p, bool border_result = false);

    /*!
     * Approximates the convex hull of the polygons.
     * \p extra_outset Extra offset outward
     * \return the convex hull (approximately)
     *
     */
    Polygons approxConvexHull(int extra_outset = 0);

    /*!
     * Make each of the polygons convex
     */
    void makeConvex();

    /*!
     * Compute the area enclosed within the polygons (minus holes)
     *
     * \return The area in square micron
     */
    double area() const;

    /*!
     * Smooth out small perpendicular segments
     * Smoothing is performed by removing the inner most vertex of a line segment smaller than \p remove_length
     * which has an angle with the next and previous line segment smaller than roughly 150*
     *
     * Note that in its current implementation this function doesn't remove line segments with an angle smaller than 30*
     * Such would be the case for an N shape.
     *
     * \param remove_length The length of the largest segment removed
     * \return The smoothed polygon
     */
    Polygons smooth(int remove_length) const;

    /*!
     * Smooth out sharp inner corners, by taking a shortcut which bypasses the corner
     *
     * \param angle The maximum angle of inner corners to be smoothed out
     * \param shortcut_length The desired length of the shortcut line segment introduced (shorter shortcuts may be unavoidable)
     * \return The resulting polygons
     */
    Polygons smooth_outward(const AngleDegrees angle, int shortcut_length);

    Polygons smooth2(int remove_length, int min_area) const; //!< removes points connected to small lines

    void removeColinearEdges(const AngleRadians max_deviation_angle = AngleRadians(0.0005))
    {
        Polygons& thiss = *this;
        for (size_t p = 0; p < size(); p++)
        {
            thiss[p].removeColinearEdges(max_deviation_angle);
            if (thiss[p].size() < 3)
            {
                remove(p);
                p--;
            }
        }
    }

public:
    void scale(const Ratio& ratio)
    {
        if (ratio == 1.)
        {
            return;
        }

        for (auto& points : *this)
        {
            for (auto& pt : points)
            {
                pt = pt * static_cast<double>(ratio);
            }
        }
    }

    void translate(const Point vec)
    {
        if (vec.X == 0 && vec.Y == 0)
        {
            return;
        }

        for (PolygonRef poly : *this)
        {
            poly.translate(vec);
        }
    }

    /*!
     * Remove all but the polygons on the very outside.
     * Exclude holes and parts within holes.
     * \return the resulting polygons.
     */
    Polygons getOutsidePolygons() const;

    /*!
     * Exclude holes which have no parts inside of them.
     * \return the resulting polygons.
     */
    Polygons removeEmptyHoles() const;

    /*!
     * Return hole polygons which have no parts inside of them.
     * \return the resulting polygons.
     */
    Polygons getEmptyHoles() const;

    /*!
     * Split up the polygons into groups according to the even-odd rule.
     * Each PolygonsPart in the result has an outline as first polygon, whereas the rest are holes.
     */
    std::vector<PolygonsPart> splitIntoParts(bool unionAll = false) const;

    /*!
     * Sort the polygons into bins where each bin has polygons which are contained within one of the polygons in the previous bin.
     *
     * \warning When polygons are crossing each other the result is undefined.
     */
    std::vector<Polygons> sortByNesting() const;

    /*!
     * Utility method for creating the tube (or 'donut') of a shape.
     * \param inner_offset Offset relative to the original shape-outline towards the inside of the shape. Sort-of like a negative normal offset, except it's the offset part that's
     * kept, not the shape. \param outer_offset Offset relative to the original shape-outline towards the outside of the shape. Comparable to normal offset. \return The resulting
     * polygons.
     */
    Polygons tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;

private:
    /*!
     * recursive part of \ref Polygons::removeEmptyHoles and \ref Polygons::getEmptyHoles
     * \param node The node of the polygons part to process
     * \param remove_holes Whether to remove empty holes or everything but the empty holes
     * \param ret Where to store polygons which are not empty holes
     */
    void removeEmptyHoles_processPolyTreeNode(const ClipperLib::PolyNode& node, const bool remove_holes, Polygons& ret) const;
    void splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<PolygonsPart>& ret) const;
    void sortByNesting_processPolyTreeNode(ClipperLib::PolyNode* node, const size_t nesting_idx, std::vector<Polygons>& ret) const;

public:
    /*!
     * Split up the polygons into groups according to the even-odd rule.
     * Each vector in the result has the index to an outline as first index, whereas the rest are indices to holes.
     *
     * \warning Note that this function reorders the polygons!
     */
    PartsView splitIntoPartsView(bool unionAll = false);

private:
    void splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Polygons& reordered, ClipperLib::PolyNode* node) const;

public:
    /*!
     * Removes polygons with area smaller than \p min_area_size (note that min_area_size is in mm^2, not in micron^2).
     * Unless \p remove_holes is true, holes are not removed even if their area is below \p min_area_size.
     * However, holes that are contained within outlines whose area is below the threshold are removed though.
     */
    void removeSmallAreas(const double min_area_size, const bool remove_holes = false);

    /*!
     * Removes polygons with circumference smaller than \p min_circumference_size (in micron).
     * Unless \p remove_holes is true, holes are not removed even if their circumference is below \p min_circumference_size.
     * However, holes that are contained within outlines whose circumference is below the threshold are removed though.
     */
    [[maybe_unused]] void removeSmallCircumference(const coord_t min_circumference_size, const bool remove_holes = false);

    /*!
     * Removes polygons with circumference smaller than \p min_circumference_size (in micron) _and_
     * an area smaller then \p min_area_size (note that min_area_size is in mm^2, not in micron^2).
     * Unless \p remove_holes is true, holes are not removed even if their circumference is
     * below \p min_circumference_size and their area smaller then \p min_area_size.
     * However, holes that are contained within outlines whose circumference is below the threshold are removed though.
     */
    [[maybe_unused]] void removeSmallAreaCircumference(const double min_area_size, const coord_t min_circumference_size, const bool remove_holes = false);

    /*!
     * Removes overlapping consecutive line segments which don't delimit a
     * positive area.
     *
     * This function is meant to work on polygons, not polylines. When misused
     * on polylines, it may cause too many vertices to be removed.
     * See \ref removeDegenerateVertsPolyline for a version that works on
     * polylines.
     */
    void removeDegenerateVerts();

    /*!
     * Removes overlapping consecutive line segments which don't delimit a
     * positive area.
     *
     * This version is meant to work on polylines, not polygons. It leaves the
     * endpoints of the polyline untouched. When misused on polygons, it may
     * leave some degenerate vertices in.
     * See \ref removeDegenerateVerts for a version that works on polygons.
     */
    void removeDegenerateVertsPolyline();

    /*!
     * Removes overlapping consecutive line segments which don't delimit a
     * positive area.
     * \param for_polyline Indicate that we're removing degenerate vertices from
     * a polyline, causing the endpoints of the polyline to be left untouched.
     * When removing vertices from a polygon, the start and end can be
     * considered for removal too, but when processing a polyline, removing
     * those would cause the polyline to become shorter.
     */
    void _removeDegenerateVerts(const bool for_polyline = false);

    /*!
     * Removes the same polygons from this set (and also empty polygons).
     * Polygons are considered the same if all points lie within [same_distance] of their counterparts.
     */
    Polygons remove(const Polygons& to_be_removed, int same_distance = 0) const
    {
        Polygons result;
        for (unsigned int poly_keep_idx = 0; poly_keep_idx < size(); poly_keep_idx++)
        {
            ConstPolygonRef poly_keep = (*this)[poly_keep_idx];
            bool should_be_removed = false;
            if (poly_keep.size() > 0)
                //             for (int hole_poly_idx = 0; hole_poly_idx < to_be_removed.size(); hole_poly_idx++)
                for (ConstPolygonRef poly_rem : to_be_removed)
                {
                    //                 PolygonRef poly_rem = to_be_removed[hole_poly_idx];
                    if (poly_rem.size() != poly_keep.size() || poly_rem.size() == 0)
                        continue;

                    // find closest point, supposing this point aligns the two shapes in the best way
                    int closest_point_idx = 0;
                    int smallestDist2 = -1;
                    for (unsigned int point_rem_idx = 0; point_rem_idx < poly_rem.size(); point_rem_idx++)
                    {
                        int dist2 = vSize2(poly_rem[point_rem_idx] - poly_keep[0]);
                        if (dist2 < smallestDist2 || smallestDist2 < 0)
                        {
                            smallestDist2 = dist2;
                            closest_point_idx = point_rem_idx;
                        }
                    }
                    bool poly_rem_is_poly_keep = true;
                    // compare the two polygons on all points
                    if (smallestDist2 > same_distance * same_distance)
                        continue;
                    for (unsigned int point_idx = 0; point_idx < poly_rem.size(); point_idx++)
                    {
                        int dist2 = vSize2(poly_rem[(closest_point_idx + point_idx) % poly_rem.size()] - poly_keep[point_idx]);
                        if (dist2 > same_distance * same_distance)
                        {
                            poly_rem_is_poly_keep = false;
                            break;
                        }
                    }
                    if (poly_rem_is_poly_keep)
                    {
                        should_be_removed = true;
                        break;
                    }
                }
            if (! should_be_removed)
                result.add(poly_keep);
        }
        return result;
    }

    Polygons processEvenOdd(ClipperLib::PolyFillType poly_fill_type = ClipperLib::PolyFillType::pftEvenOdd) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.Execute(ClipperLib::ctUnion, ret.paths, poly_fill_type);
        return ret;
    }

    /*!
     * Ensure the polygon is manifold, by removing small areas where the polygon touches itself.
     *  ____                  ____
     * |    |                |    |
     * |    |____     ==>    |   / ____
     *  """"|    |            """ /    |
     *      |____|                |____|
     *
     */
    void ensureManifold();

    coord_t polygonLength() const
    {
        coord_t length = 0;
        for (ConstPolygonRef poly : *this)
        {
            length += poly.polygonLength();
        }
        return length;
    }

    coord_t polyLineLength() const;

    Point min() const
    {
        Point ret = Point(POINT_MAX, POINT_MAX);
        for (const ClipperLib::Path& polygon : paths)
        {
            for (Point p : polygon)
            {
                ret.X = std::min(ret.X, p.X);
                ret.Y = std::min(ret.Y, p.Y);
            }
        }
        return ret;
    }

    Point max() const
    {
        Point ret = Point(POINT_MIN, POINT_MIN);
        for (const ClipperLib::Path& polygon : paths)
        {
            for (Point p : polygon)
            {
                ret.X = std::max(ret.X, p.X);
                ret.Y = std::max(ret.Y, p.Y);
            }
        }
        return ret;
    }

    void applyMatrix(const PointMatrix& matrix)
    {
        for (unsigned int i = 0; i < paths.size(); i++)
        {
            for (unsigned int j = 0; j < paths[i].size(); j++)
            {
                paths[i][j] = matrix.apply(paths[i][j]);
            }
        }
    }

    void applyMatrix(const Point3Matrix& matrix)
    {
        for (unsigned int i = 0; i < paths.size(); i++)
        {
            for (unsigned int j = 0; j < paths[i].size(); j++)
            {
                paths[i][j] = matrix.apply(paths[i][j]);
            }
        }
    }

    Polygons offset(const std::vector<coord_t>& offset_dists) const;
};

/*!
 * A single area with holes. The first polygon is the outline, while the rest are holes within this outline.
 *
 * This class has little more functionality than Polygons, but serves to show that a specific instance is ordered such that the first Polygon is the outline and the rest are holes.
 */
class PolygonsPart : public Polygons
{
public:
    PolygonRef outerPolygon()
    {
        return paths[0];
    }
    ConstPolygonRef outerPolygon() const
    {
        return paths[0];
    }

    /*!
     * Tests whether the given point is inside this polygon part.
     * \param p The point to test whether it is inside.
     * \param border_result If the point is exactly on the border, this will be
     * returned instead.
     */
    bool inside(Point p, bool border_result = false) const;
};

/*!
 * Extension of vector<vector<unsigned int>> which is similar to a vector of PolygonParts, except the base of the container is indices to polygons into the original Polygons,
 * instead of the polygons themselves
 */
class PartsView : public std::vector<std::vector<unsigned int>>
{
public:
    Polygons& polygons;
    PartsView(Polygons& polygons)
        : polygons(polygons)
    {
    }
    /*!
     * Get the index of the PolygonsPart of which the polygon with index \p poly_idx is part.
     *
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The PolygonsPart containing the polygon with index \p poly_idx
     */
    unsigned int getPartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx = nullptr) const;
    /*!
     * Assemble the PolygonsPart of which the polygon with index \p poly_idx is part.
     *
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The PolygonsPart containing the polygon with index \p poly_idx
     */
    PolygonsPart assemblePartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx = nullptr) const;
    /*!
     * Assemble the PolygonsPart of which the polygon with index \p poly_idx is part.
     *
     * \param part_idx The index of the part
     * \return The PolygonsPart with index \p poly_idx
     */
    PolygonsPart assemblePart(unsigned int part_idx) const;
};

} // namespace cura

#endif // UTILS_POLYGON_H
