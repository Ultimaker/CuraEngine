// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_POLYGONS_H
#define UTILS_POLYGONS_H

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

class OpenPolyline;
class Polygon;
class Polygons;
class PolygonsPart;
class PartsView;

template<class LineType>
class LinesSet : public std::vector<LineType>
{
public:
    LinesSet() = default;

    LinesSet(const LinesSet& other) = default;

    LinesSet(LinesSet&& other) = default;

    LinesSet(const std::vector<LineType>& lines)
        : std::vector<LineType>(lines)
    {
    }

    LinesSet(std::vector<LineType>&& lines)
        : std::vector<LineType>(std::move(lines))
    {
    }

    LinesSet(const std::vector<path_t>& paths)
        : std::vector<LineType>(*reinterpret_cast<const std::vector<LineType>*>(&paths))
    {
    }

    LinesSet& operator=(const LinesSet& other)
    {
        std::vector<LineType>::operator=(other);
        return *this;
    }

    LinesSet& operator=(LinesSet&& other)
    {
        std::vector<LineType>::operator=(other);
        return *this;
    }

    void splitIntoSegments(std::vector<OpenPolyline>& result) const;
    std::vector<OpenPolyline> splitIntoSegments() const;

    /*!
     * Removes overlapping consecutive line segments which don't delimit a
     * positive area.
     *
     * This function is meant to work on polygons, not polylines. When misused
     * on polylines, it may cause too many vertices to be removed.
     * See \ref removeDegenerateVertsPolyline for a version that works on
     * polylines.
     */
#warning rename this to removeDegenerateVerts
    void removeDegenerateVertsForEveryone();

    Polygons offset(coord_t distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;
    // Polygons offsetPolyLine(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, bool inputPolyIsClosed = false) const;

    const std::vector<std::vector<point_t>>& getCallable() const
    {
        // This does work as long as we don't add any attribute to the Polygon class or any of its
        // parent until std::vector<point_t>
        return *reinterpret_cast<const std::vector<std::vector<point_t>>*>(this);
    }

    std::vector<std::vector<point_t>>& getCallable()
    {
        // This does work as long as we don't add any attribute to the Polygon class or any of its
        // parent until std::vector<point_t>
        return *reinterpret_cast<std::vector<std::vector<point_t>>*>(this);
    }
};

class Polygons : public LinesSet<Polygon>
{
    friend class Polygon;

public:
    Polygons() = default;

    Polygons(const Polygons& other) = default;

    Polygons(Polygons&& other) = default;

    Polygons(const std::initializer_list<Polygon>& initializer)
        : LinesSet<Polygon>(initializer)
    {
    }

    Polygons(const std::vector<path_t>& paths)
        : LinesSet<Polygon>(paths)
    {
    }

    Polygons& operator=(const Polygons& other)
    {
        LinesSet<Polygon>::operator=(other);
        return *this;
    }

    Polygons& operator=(Polygons&& polygons);

    //!< Return the amount of points in all polygons
    size_t pointCount() const;

    /*!
     * Remove a polygon from the list and move the last polygon to its place
     *
     * \warning changes the order of the polygons!
     */
    void remove(size_t index);

    void add(const Polygons& other);

    void addIfNotEmpty(const Polygon& polygon);

    void addIfNotEmpty(Polygon&& polygon);

    /*!
     * Add a 'polygon' consisting of two points
     */
    void addLine(const point_t& from, const point_t& to);

    Polygon& newPoly();

    /*!
     * Convert ClipperLib::PolyTree to a Polygons object,
     * which uses ClipperLib::Paths instead of ClipperLib::PolyTree
     */
    static Polygons toPolygons(ClipperLib::PolyTree& poly_tree);

    Polygons difference(const Polygons& other) const;

    Polygons unionPolygons(const Polygons& other, ClipperLib::PolyFillType fill_type = ClipperLib::pftNonZero) const;

    /*!
     * Union all polygons with each other (When polygons.add(polygon) has been called for overlapping polygons)
     */
    Polygons unionPolygons() const
    {
        return unionPolygons(Polygons());
    }

    Polygons intersection(const Polygons& other) const;


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
    std::vector<OpenPolyline> intersectionPolyLines(const LinesSet<OpenPolyline>& polylines, bool restitch = true, const coord_t max_stitch_distance = 10_mu) const;

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

    Polygons xorPolygons(const Polygons& other, ClipperLib::PolyFillType pft = ClipperLib::pftEvenOdd) const;

    Polygons execute(ClipperLib::PolyFillType pft = ClipperLib::pftEvenOdd) const;

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
    bool inside(Point2LL p, bool border_result = false) const;

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
    bool insideOld(Point2LL p, bool border_result = false) const;

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
    size_t findInside(Point2LL p, bool border_result = false) const;

    /*!
     * Approximates the convex hull of the polygons.
     * \p extra_outset Extra offset outward
     * \return the convex hull (approximately)
     *
     */
    Polygons approxConvexHull(int extra_outset = 0) const;

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
    Polygons smooth_outward(const AngleDegrees angle, int shortcut_length) const;

    Polygons smooth2(int remove_length, int min_area) const; //!< removes points connected to small lines

    void removeColinearEdges(const AngleRadians max_deviation_angle = AngleRadians(0.0005));

    void scale(const Ratio& ratio);

    void translate(const point_t& delta);

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

    /*!
     * Split up the polygons into groups according to the even-odd rule.
     * Each vector in the result has the index to an outline as first index, whereas the rest are indices to holes.
     *
     * \warning Note that this function reorders the polygons!
     */
    PartsView splitIntoPartsView(bool unionAll = false);

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
     * Removes the same polygons from this set (and also empty polygons).
     * Polygons are considered the same if all points lie within [same_distance] of their counterparts.
     */
    Polygons remove(const Polygons& to_be_removed, int same_distance = 0) const;

    Polygons processEvenOdd(ClipperLib::PolyFillType poly_fill_type = ClipperLib::PolyFillType::pftEvenOdd) const;

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

    coord_t length() const;

    point_t min() const;

    point_t max() const;

    void applyMatrix(const PointMatrix& matrix);

    void applyMatrix(const Point3Matrix& matrix);

#warning If this is to be used, rename it
    // Polygons offset(const std::vector<coord_t>& offset_dists) const;

    /*!
     * @brief Export the polygon to a WKT string
     *
     * @param stream The stream to write to
     */
    [[maybe_unused]] void writeWkt(std::ostream& stream) const;

    /*!
     * @brief Import the polygon from a WKT string
     *
     * @param wkt The WKT string to read from
     * @return Polygons The polygons read from the stream
     */
    [[maybe_unused]] static Polygons fromWkt(const std::string& wkt);

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
    void splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Polygons& reordered, ClipperLib::PolyNode* node) const;
};

/*!
 * A single area with holes. The first polygon is the outline, while the rest are holes within this outline.
 *
 * This class has little more functionality than Polygons, but serves to show that a specific instance is ordered such that the first Polygon is the outline and the rest are holes.
 */
class PolygonsPart : public Polygons
{
public:
    Polygon& outerPolygon()
    {
        return front();
    }

    const Polygon& outerPolygon() const
    {
        return front();
    }

    /*!
     * Tests whether the given point is inside this polygon part.
     * \param p The point to test whether it is inside.
     * \param border_result If the point is exactly on the border, this will be
     * returned instead.
     */
    bool inside(Point2LL p, bool border_result = false) const;
};

/*!
 * Extension of vector<vector<unsigned int>> which is similar to a vector of PolygonParts, except the base of the container is indices to polygons into the original Polygons,
 * instead of the polygons themselves
 */
class PartsView : public std::vector<std::vector<size_t>>
{
public:
    Polygons& polygons_;
    PartsView(Polygons& polygons)
        : polygons_(polygons)
    {
    }
    /*!
     * Get the index of the PolygonsPart of which the polygon with index \p poly_idx is part.
     *
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The PolygonsPart containing the polygon with index \p poly_idx
     */
    size_t getPartContaining(size_t poly_idx, size_t* boundary_poly_idx = nullptr) const;
    /*!
     * Assemble the PolygonsPart of which the polygon with index \p poly_idx is part.
     *
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The PolygonsPart containing the polygon with index \p poly_idx
     */
    PolygonsPart assemblePartContaining(size_t poly_idx, size_t* boundary_poly_idx = nullptr) const;
    /*!
     * Assemble the PolygonsPart of which the polygon with index \p poly_idx is part.
     *
     * \param part_idx The index of the part
     * \return The PolygonsPart with index \p poly_idx
     */
    PolygonsPart assemblePart(size_t part_idx) const;
};

template<class LineType>
void LinesSet<LineType>::splitIntoSegments(std::vector<OpenPolyline>& result) const
{
    for (const LineType& line : (*this))
    {
        line.splitIntoSegments(result);
    }
}

template<class LineType>
std::vector<OpenPolyline> LinesSet<LineType>::splitIntoSegments() const
{
    std::vector<OpenPolyline> result;
    for (const LineType& line : (*this))
    {
        line.splitIntoSegments(result);
    }
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

#endif // UTILS_POLYGONS_H
