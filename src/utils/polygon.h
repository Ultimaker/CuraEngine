#ifndef UTILS_POLYGON_H
#define UTILS_POLYGON_H

#include <vector>
#include <assert.h>
#include <float.h>
#include <clipper/clipper.hpp>

#include <algorithm>    // std::reverse, fill_n array
#include <cmath> // fabs
#include <limits> // int64_t.min
#include <list>

#include "intpoint.h"

//#define CHECK_POLY_ACCESS
#ifdef CHECK_POLY_ACCESS
#define POLY_ASSERT(e) assert(e)
#else
#define POLY_ASSERT(e) do {} while(0)
#endif

namespace cura {


class PartsView;
class Polygons;

class ListPolyIt;

typedef std::list<Point> ListPolygon; //!< A polygon represented by a linked list instead of a vector
typedef std::vector<ListPolygon> ListPolygons; //!< Polygons represented by a vector of linked lists instead of a vector of vectors

const static int clipper_init = (0);
#define NO_INDEX (std::numeric_limits<unsigned int>::max())

class PolygonRef
{
    ClipperLib::Path* path;
    PolygonRef()
    : path(nullptr)
    {}
public:
    PolygonRef(ClipperLib::Path& polygon)
    : path(&polygon)
    {}
    
    unsigned int size() const
    {
        return path->size();
    }

    Point& operator[] (unsigned int index) const
    {
        POLY_ASSERT(index < size());
        return (*path)[index];
    }

    void* data()
    {
        return path->data();
    }

    const void* data() const
    {
        return path->data();
    }

    void add(const Point p)
    {
        path->push_back(p);
    }
    
    PolygonRef& operator=(const PolygonRef& other) { path = other.path; return *this; }

    bool operator==(const PolygonRef& other) const =delete;

    ClipperLib::Path& operator*() { return *path; }
    
    template <typename... Args>
    void emplace_back(Args&&... args)
    {
        path->emplace_back(args...);
    }

    void remove(unsigned int index)
    {
        POLY_ASSERT(index < size());
        path->erase(path->begin() + index);
    }

    void clear()
    {
        path->clear();
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

    void reverse()
    {
        ClipperLib::ReversePath(*path);
    }

    Polygons offset(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    int64_t polygonLength() const
    {
        int64_t length = 0;
        Point p0 = (*path)[path->size()-1];
        for(unsigned int n=0; n<path->size(); n++)
        {
            Point p1 = (*path)[n];
            length += vSize(p0 - p1);
            p0 = p1;
        }
        return length;
    }
    
    bool shorterThan(int64_t check_length) const;

    Point min() const
    {
        Point ret = Point(POINT_MAX, POINT_MAX);
        for(Point p : *path)
        {
            ret.X = std::min(ret.X, p.X);
            ret.Y = std::min(ret.Y, p.Y);
        }
        return ret;
    }
    
    Point max() const
    {
        Point ret = Point(POINT_MIN, POINT_MIN);
        for(Point p : *path)
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

    Point centerOfMass() const
    {
        double x = 0, y = 0;
        Point p0 = (*path)[path->size()-1];
        for(unsigned int n=0; n<path->size(); n++)
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

    Point closestPointTo(Point p)
    {
        Point ret = p;
        float bestDist = FLT_MAX;
        for(unsigned int n=0; n<path->size(); n++)
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
    bool _inside(Point p, bool border_result = false);

    /*!
     * Clipper function.
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     * 
     * http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PointInPolygon.htm
     */
    bool inside(Point p, bool border_result = false)
    {
        int res = ClipperLib::PointInPolygon(p, *path);
        if (res == -1)
        {
            return border_result;
        }
        return res == 1;
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
    void smooth(int remove_length, PolygonRef result);

    /*!
     * Smooth out sharp inner corners, by taking a shortcut which bypasses the corner
     * 
     * \param angle The maximal angle of inner corners to be smoothed out
     * \param shortcut_length The desired length of the shortcut line segment introduced (shorter shortcuts may be unavoidable)
     * \param result The resulting polygon
     */
    void smooth_outward(float angle, int shortcut_length, PolygonRef result) const;

    /*!
     * Smooth out the polygon and store the result in \p result.
     * Smoothing is performed by removing vertices for which both connected line segments are smaller than \p remove_length
     * 
     * \param remove_length The length of the largest segment removed
     * \param result (output) The result polygon, assumed to be empty
     */
    void smooth2(int remove_length, PolygonRef result);

    /*! 
     * Removes consecutive line segments with same orientation and changes this polygon.
     * 
     * Removes verts which are connected to line segments which are both too small.
     * Removes verts which detour from a direct line from the previous and next vert by a too small amount.
     * 
     * \param smallest_line_segment_squared maximal squared length of removed line segments
     * \param allowed_error_distance_squared The square of the distance of the middle point to the line segment of the consecutive and previous point for which the middle point is removed
     */
    void simplify(int smallest_line_segment_squared = 100, int allowed_error_distance_squared = 25);

    void pop_back()
    { 
        path->pop_back();
    }
    
    ClipperLib::Path::reference back() const
    {
        return path->back();
    }
    
    ClipperLib::Path::iterator begin()
    {
        return path->begin();
    }

    ClipperLib::Path::iterator end()
    {
        return path->end();
    }

    ClipperLib::Path::const_iterator begin() const
    {
        return path->begin();
    }

    ClipperLib::Path::const_iterator end() const
    {
        return path->end();
    }

    friend class Polygons;
    friend class Polygon;

private:
    /*!
     * Smooth out a simple corner consisting of two linesegments.
     * 
     * Auxiliary function for \ref smooth_outward
     * 
     * \param poly The polygon in which to find the corner
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
    static void smooth_corner_simple(ListPolygon& poly, const Point p0, const Point p1, const Point p2, const ListPolyIt p0_it, const ListPolyIt p1_it, const ListPolyIt p2_it, const Point v10, const Point v12, const Point v02, const int64_t shortcut_length, float cos_angle);

    /*!
     * Smooth out a complex corner where the shortcut bypasses more than two line segments
     * 
     * Auxiliary function for \ref smooth_outward
     * 
     * \param poly The polygon in which to find the corner
     * \param p1 The corner point
     * \param v02 The vector from p0 to p2
     * \param[in,out] p0_it Iterator to the last point checked before \p p1 to consider cutting off
     * \param[in,out] p2_it Iterator to the last point checked after \p p1 to consider cutting off
     * \param shortcut_length The desired length ofthe shortcutting line
     */
    static void smooth_corner_complex(ListPolygon& poly, const Point p1, const Point v02, ListPolyIt& p0_it, ListPolyIt& p2_it, const int64_t shortcut_length);

    /*!
     * Try to take a step away from the corner point in order to take a bigger shortcut.
     * 
     * Try to take the shortcut from a place as far away from the corner as the place we are taking the shortcut to.
     * 
     * Auxiliary function for \ref smooth_outward
     * 
     * \param[in] p1 The corner point
     * \param[in] shortcut_length The desired length ofthe shortcutting line
     * \param[in,out] p0_it Iterator to the previously checked point somewhere beyond \p p1. Updated for the next iteration.
     * \param[in,out] p2_it Iterator to the previously checked point somewhere before \p p1. Updated for the next iteration.
     * \param[in,out] forward_is_blocked Whether trying another step forward is blocked by the smoothing outward condition. Updated for the next iteration.
     * \param[in,out] backward_is_blocked Whether trying another step backward is blocked by the smoothing outward condition. Updated for the next iteration.
     * \param[in,out] forward_is_too_far Whether trying another step forward is blocked by the shortcut length condition. Updated for the next iteration.
     * \param[in,out] backward_is_too_far Whether trying another step backward is blocked by the shortcut length condition. Updated for the next iteration.
     */
    static void smooth_outward_step(const Point p1, const int64_t shortcut_length, ListPolyIt& p0_it, ListPolyIt& p2_it, bool& forward_is_blocked, bool& backward_is_blocked, bool& forward_is_too_far, bool& backward_is_too_far);
};

class Polygon : public PolygonRef
{
    ClipperLib::Path poly;
public:
    Polygon()
    : PolygonRef(poly)
    {
    }

    Polygon(const PolygonRef& other)
    : PolygonRef(poly)
    {
        poly = *other.path;
    }
};

class PolygonsPart;

class Polygons
{
    friend class Polygon;
    friend class PolygonRef;
protected:
    ClipperLib::Paths paths;
public:
    unsigned int size() const
    {
        return paths.size();
    }

    PolygonRef operator[] (unsigned int index)
    {
        POLY_ASSERT(index < size());
        return PolygonRef(paths[index]);
    }
    const PolygonRef operator[] (unsigned int index) const
    {
        return const_cast<Polygons*>(this)->operator[](index);
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
    void remove(unsigned int index)
    {
        POLY_ASSERT(index < size());
        paths.erase(paths.begin() + index);
    }
    void erase(ClipperLib::Paths::iterator start, ClipperLib::Paths::iterator end)
    {
        paths.erase(start, end);
    }
    void clear()
    {
        paths.clear();
    }
    void add(const PolygonRef& poly)
    {
        paths.push_back(*poly.path);
    }
    void add(Polygon&& other_poly)
    {
        paths.emplace_back(std::move(*other_poly));
    }
    void add(const Polygons& other)
    {
        for(unsigned int n=0; n<other.paths.size(); n++)
            paths.push_back(other.paths[n]);
    }
    PolygonRef newPoly()
    {
        paths.emplace_back();
        return PolygonRef(paths.back());
    }
    PolygonRef back()
    {
        return PolygonRef(paths.back());
    }

    Polygons() {}

    Polygons(const Polygons& other) { paths = other.paths; }
    Polygons& operator=(const Polygons& other) { paths = other.paths; return *this; }

    bool operator==(const Polygons& other) const =delete;

    Polygons difference(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
        clipper.Execute(ClipperLib::ctDifference, ret.paths);
        return ret;
    }
    Polygons unionPolygons(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.AddPaths(other.paths, ClipperLib::ptSubject, true);
        clipper.Execute(ClipperLib::ctUnion, ret.paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
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
    Polygons xorPolygons(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
        clipper.Execute(ClipperLib::ctXor, ret.paths);
        return ret;
    }
    Polygons offset(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const
    {
        Polygons ret;
        ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
        clipper.AddPaths(paths, joinType, ClipperLib::etClosedPolygon);
        clipper.MiterLimit = miter_limit;
        clipper.Execute(ret.paths, distance);
        return ret;
    }
    
    Polygons offsetPolyLine(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter) const
    {
        Polygons ret;
        double miterLimit = 1.2;
        ClipperLib::ClipperOffset clipper(miterLimit, 10.0);
        clipper.AddPaths(paths, joinType, ClipperLib::etOpenSquare);
        clipper.MiterLimit = miterLimit;
        clipper.Execute(ret.paths, distance);
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
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return \p border_result.
     * 
     * \param p The point for which to check if it is inside this polygon
     * \param border_result What to return when the point is exactly on the border
     * \return Whether the point \p p is inside this polygon (or \p border_result when it is on the border)
     */
    bool inside(Point p, bool border_result = false) const;
    
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
    Polygons approxConvexHull(int extra_outset = 0)
    {
        int overshoot = 100000; // 10 cm (hardcoded value)
        
        return offset(overshoot, ClipperLib::jtRound).offset(-overshoot+extra_outset, ClipperLib::jtRound);
    }
    
    /*!
     * Convex hull of all the points in the polygons.
     * \return the convex hull
     *
     */
    Polygon convexHull() const;

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
    Polygons smooth(int remove_length);

    /*!
     * Smooth out sharp inner corners, by taking a shortcut which bypasses the corner
     * 
     * \param angle The maximal angle of inner corners to be smoothed out
     * \param shortcut_length The desired length of the shortcut line segment introduced (shorter shortcuts may be unavoidable)
     * \return The resulting polygons
     */
    Polygons smooth_outward(float angle, int shortcut_length);

    Polygons smooth2(int remove_length, int min_area); //!< removes points connected to small lines
    
    /*!
     * removes points connected to similarly oriented lines
     * 
     * \param smallest_line_segment maximal length of removed line segments
     * \param allowed_error_distance The distance of the middle point to the line segment of the consecutive and previous point for which the middle point is removed
     */
    void simplify(int smallest_line_segment = 10, int allowed_error_distance = 5) 
    {
        int allowed_error_distance_squared = allowed_error_distance * allowed_error_distance;
        int smallest_line_segment_squared = smallest_line_segment * smallest_line_segment;
        Polygons& thiss = *this;
        for (unsigned int p = 0; p < size(); p++)
        {
            thiss[p].simplify(smallest_line_segment_squared, allowed_error_distance_squared);
            if (thiss[p].size() < 3)
            {
                remove(p);
                p--;
            }
        }
    }
    
    /*!
     * Split up the polygons into groups according to the even-odd rule.
     * Each PolygonsPart in the result has an outline as first polygon, whereas the rest are holes.
     */
    std::vector<PolygonsPart> splitIntoParts(bool unionAll = false) const;
private:
    void splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<PolygonsPart>& ret) const;
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
     * Removes polygons with area smaller than \p minAreaSize (note that minAreaSize is in mm^2, not in micron^2).
     */
    void removeSmallAreas(double minAreaSize)
    {               
        Polygons& thiss = *this;
        for(unsigned int i=0; i<size(); i++)
        {
            double area = INT2MM(INT2MM(fabs(thiss[i].area())));
            if (area < minAreaSize) // Only create an up/down skin if the area is large enough. So you do not create tiny blobs of "trying to fill"
            {
                remove(i);
                i -= 1;
            }
        }
    }
    /*!
     * Removes overlapping consecutive line segments which don't delimit a positive area.
     */
    void removeDegenerateVerts()
    {
        Polygons& thiss = *this;
        for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
        {
            PolygonRef poly = thiss[poly_idx];
            Polygon result;
            
            auto isDegenerate = [](Point& last, Point& now, Point& next)
            {
                Point last_line = now - last;
                Point next_line = next - now;
                return dot(last_line, next_line) == -1 * vSize(last_line) * vSize(next_line);
            };
            bool isChanged = false;
            for (unsigned int idx = 0; idx < poly.size(); idx++)
            {
                Point& last = (result.size() == 0) ? poly.back() : result.back();
                if (idx+1 == poly.size() && result.size() == 0) { break; }
                Point& next = (idx+1 == poly.size())? result[0] : poly[idx+1];
                if ( isDegenerate(last, poly[idx], next) )
                { // lines are in the opposite direction
                    // don't add vert to the result
                    isChanged = true;
                    while (result.size() > 1 && isDegenerate(result[result.size()-2], result.back(), next) )
                    {
                        result.pop_back();
                    }
                }
                else 
                {
                    result.add(poly[idx]);
                }
            }
            
            if (isChanged)
            {
                if (result.size() > 2) 
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
    /*!
     * Removes the same polygons from this set (and also empty polygons).
     * Polygons are considered the same if all points lie within [same_distance] of their counterparts.
     */
    Polygons remove(Polygons& to_be_removed, int same_distance = 0)
    {
        Polygons result;
        for (unsigned int poly_keep_idx = 0; poly_keep_idx < size(); poly_keep_idx++)
        {
            PolygonRef poly_keep = (*this)[poly_keep_idx];
            bool should_be_removed = false;
            if (poly_keep.size() > 0) 
//             for (int hole_poly_idx = 0; hole_poly_idx < to_be_removed.size(); hole_poly_idx++)
            for (PolygonRef poly_rem : to_be_removed)
            {
//                 PolygonRef poly_rem = to_be_removed[hole_poly_idx];
                if (poly_rem.size() != poly_keep.size() || poly_rem.size() == 0) continue;
                
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
            if (!should_be_removed)
                result.add(poly_keep);
            
        }
        return result;
    }

    Polygons processEvenOdd() const
    {
        Polygons ret;
        ClipperLib::Clipper clipper(clipper_init);
        clipper.AddPaths(paths, ClipperLib::ptSubject, true);
        clipper.Execute(ClipperLib::ctUnion, ret.paths);
        return ret;
    }

    int64_t polygonLength() const
    {
        int64_t length = 0;
        for(unsigned int i=0; i<paths.size(); i++)
        {
            Point p0 = paths[i][paths[i].size()-1];
            for(unsigned int n=0; n<paths[i].size(); n++)
            {
                Point p1 = paths[i][n];
                length += vSize(p0 - p1);
                p0 = p1;
            }
        }
        return length;
    }
    
    Point min() const
    {
        Point ret = Point(POINT_MAX, POINT_MAX);
        for(const ClipperLib::Path& polygon : paths)
        {
            for(Point p : polygon)
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
        for(const ClipperLib::Path& polygon : paths)
        {
            for(Point p : polygon)
            {
                ret.X = std::max(ret.X, p.X);
                ret.Y = std::max(ret.Y, p.Y);
            }
        }
        return ret;
    }

    void applyMatrix(const PointMatrix& matrix)
    {
        for(unsigned int i=0; i<paths.size(); i++)
        {
            for(unsigned int j=0; j<paths[i].size(); j++)
            {
                paths[i][j] = matrix.apply(paths[i][j]);
            }
        }
    }
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
        Polygons& thiss = *this;
        return thiss[0];
    }
    
    bool inside(Point p)
    {
        if (size() < 1)
            return false;
        if (!(*this)[0].inside(p))
            return false;
        for(unsigned int n=1; n<paths.size(); n++)
        {
            if ((*this)[n].inside(p))
                return false;
        }
        return true;
    }
};

/*!
 * Extension of vector<vector<unsigned int>> which is similar to a vector of PolygonParts, except the base of the container is indices to polygons into the original Polygons, instead of the polygons themselves
 */
class PartsView : public std::vector<std::vector<unsigned int>>
{
public:
    Polygons& polygons;
    PartsView(Polygons& polygons) : polygons(polygons) { }
    /*!
     * Get the index of the PolygonsPart of which the polygon with index \p poly_idx is part.
     * 
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The PolygonsPart containing the polygon with index \p poly_idx
     */
    unsigned int getPartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx = nullptr);
    /*!
     * Assemble the PolygonsPart of which the polygon with index \p poly_idx is part.
     * 
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The PolygonsPart containing the polygon with index \p poly_idx
     */
    PolygonsPart assemblePartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx = nullptr);
    /*!
     * Assemble the PolygonsPart of which the polygon with index \p poly_idx is part.
     * 
     * \param part_idx The index of the part
     * \return The PolygonsPart with index \p poly_idx
     */
    PolygonsPart assemblePart(unsigned int part_idx) const;
};

}//namespace cura

#endif//UTILS_POLYGON_H
