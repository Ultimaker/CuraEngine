// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POLYGON_UTILS_H
#define UTILS_POLYGON_UTILS_H

#include <functional> // function
#include <limits>
#include <memory> // unique_ptr
#include <numbers>
#include <optional>

#include "PolygonsPointIndex.h"
#include "SparseLineGrid.h"
#include "SparsePointGridInclusive.h"
#include "geometry/ClosedLinesSet.h"
#include "geometry/Polygon.h"

namespace cura
{

/*!
 * Result of finding the closest point to a given within a set of polygons, with extra information on where the point is.
 */
template<class LineType>
struct ClosestPoint
{
    Point2LL location_; //!< Result location
    const LineType* poly_{ nullptr }; //!< Line in which the result was found (or nullptr if no result was found)
    size_t poly_idx_; //!< The index of the polygon in some Polygons where ClosestPoint::poly can be found
    size_t point_idx_; //!< Index to the first point in the polygon of the line segment on which the result was found

    ClosestPoint(Point2LL p, size_t pos, const LineType* poly)
        : location_(p)
        , poly_(poly)
        , poly_idx_(NO_INDEX)
        , point_idx_(pos)
    {
    }

    ClosestPoint(Point2LL p, size_t pos, const LineType* poly, size_t poly_idx)
        : location_(p)
        , poly_(poly)
        , poly_idx_(poly_idx)
        , point_idx_(pos)
    {
    }

    ClosestPoint(const LineType* poly)
        : poly_(poly)
        , poly_idx_(NO_INDEX)
        , point_idx_(NO_INDEX)
    {
    }

    ClosestPoint()
        : poly_idx_(NO_INDEX)
        , point_idx_(NO_INDEX)
    {
    }

    Point2LL p() const
    { // conformity with other classes
        return location_;
    }

    bool isValid() const
    {
        return point_idx_ != NO_INDEX;
    }

    bool operator==(const ClosestPoint& rhs) const
    {
        // no need to compare on poy_idx
        // it's sometimes unused while poly is always initialized
        return poly_ == rhs.poly_ && point_idx_ == rhs.point_idx_ && location_ == rhs.location_;
    }
};

using ClosestPointPolygon = ClosestPoint<Polygon>;

/*!
 * A point within a polygon and the index of which segment in the polygon the point lies on.
 */
struct GivenDistPoint
{
    Point2LL location; //!< Result location
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
};

typedef SparseLineGrid<PolygonsPointIndex, PolygonsPointIndexSegmentLocator> LocToLineGrid;

class PolygonUtils
{
public:
    static const std::function<int(Point2LL)> no_penalty_function; //!< Function always returning zero

    /*!
     * Generate a grid of dots inside of the area of the \p polygons.
     */
    static std::vector<Point2LL> spreadDotsArea(const Shape& polygons, coord_t grid_size);

    static std::vector<Point2LL> spreadDotsArea(const Shape& polygons, Point2LL grid_size);

    /*!
     * Whether a polygon intersects with a line-segment. If true, the closest collision point to 'b' is stored in the result.
     */
    static bool lineSegmentPolygonsIntersection(
        const Point2LL& a,
        const Point2LL& b,
        const Shape& current_outlines,
        const LocToLineGrid& outline_locator,
        Point2LL& result,
        const coord_t within_max_dist);

    /*!
     * Get the normal of a boundary point, pointing outward.
     * Only the direction is set.
     * Nothing is said about the length of the vector returned.
     *
     * \param poly The polygon.
     * \param point_idx The index of the point in the polygon.
     */
    static Point2LL getVertexInwardNormal(const Polyline& poly, unsigned int point_idx);

    /*!
     * Get a point from the \p poly with a given \p offset.
     *
     * \param poly The polygon.
     * \param point_idx The index of the point in the polygon.
     * \param offset The distance the point has to be moved outward from the polygon.
     * \return A point at the given distance inward from the point on the boundary polygon.
     */
    static Point2LL getBoundaryPointWithOffset(const Polyline& poly, unsigned int point_idx, int64_t offset);

    /*!
     * Moves the point \p from onto the nearest polygon or leaves the point as-is, when the comb boundary is not within the root of \p max_dist2 distance.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
     * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
     *
     * \param polygons The polygons onto which to move the point
     * \param from[in,out] The point to move.
     * \param distance The distance by which to move the point.
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \return The index to the polygon onto which we have moved the point.
     */
    static size_t moveInside(const Shape& polygons, Point2LL& from, int distance = 0, int64_t max_dist2 = std::numeric_limits<int64_t>::max());

    /**
     * \brief Moves the point \p from onto the nearest polygon or leaves the
     * point as-is, when the comb boundary is not within the square root of \p
     * max_dist2 distance.
     *
     * Given a \p distance more than zero, the point will end up inside, and
     * conversely outside. When the point is already in/outside by more than \p
     * distance, \p from is unaltered. When the point is in/outside by less than
     * \p distance, \p is moved to the correct place.
     * @param polygon The polygon onto which to move the point.
     * @param from[in, out] The point to move.
     * @param distance The distance by which to move the point.
     * @param max_dist2 The squared maximal allowed distance from the point to
     * the polygon.
     * \return Always returns 0.
     */
    static unsigned int moveInside(const ClosedPolyline& polygon, Point2LL& from, int distance = 0, int64_t max_dist2 = std::numeric_limits<int64_t>::max());

    /*!
     * Moves the point \p from onto the nearest polygon or leaves the point as-is, when the comb boundary is not within the root of \p max_dist2 distance.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
     * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
     *
     * \warning If \p loc_to_line_grid is used, it's best to have all and only \p polygons in there.
     * If \p from is not closest to \p polygons this function may
     * return a ClosestPointPolygon on a polygon in \p loc_to_line_grid which is not in \p polygons.
     *
     * \param polygons The polygons onto which to move the point
     * \param from[in,out] The point to move.
     * \param distance The distance by which to move the point.
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \param loc_to_line_polygons All polygons with which the \p loc_to_line_grid has been created.
     * \param loc_to_line_grid A SparseGrid mapping locations to line segments of \p polygons
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPointPolygon moveInside2(
        const Shape& polygons,
        Point2LL& from,
        const int distance = 0,
        const int64_t max_dist2 = std::numeric_limits<int64_t>::max(),
        const Shape* loc_to_line_polygons = nullptr,
        const LocToLineGrid* loc_to_line_grid = nullptr,
        const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     * Moves the point \p from onto the nearest segment of \p polygon or leaves the point as-is, when the comb boundary is not within the root of \p max_dist2 distance.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
     * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
     *
     * \warning When a \p loc_to_line is given this function only considers nearby elements.
     * Even when the penalty function favours elements farther away.
     * Also using the \p loc_to_line_grid automatically considers \p all_polygons
     *
     * \param loc_to_line_polygons All polygons which are present in the \p loc_to_line_grid of which \p polygon is an element
     * \param polygon The polygon onto which to move the point
     * \param from[in,out] The point to move.
     * \param distance The distance by which to move the point.
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \param loc_to_line_grid A SparseGrid mapping locations to line segments of \p polygon
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPointPolygon moveInside2(
        const Shape& loc_to_line_polygons,
        const Polygon& polygon,
        Point2LL& from,
        const int distance = 0,
        const int64_t max_dist2 = std::numeric_limits<int64_t>::max(),
        const LocToLineGrid* loc_to_line_grid = nullptr,
        const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     * The opposite of moveInside.
     *
     * Moves the point \p from onto the nearest polygon or leaves the point as-is, when the comb boundary is not within \p distance.
     * Given a \p distance more than zero, the point will end up outside, and conversely inside.
     * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
     * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
     *
     * \param polygons The polygons onto which to move the point
     * \param from[in,out] The point to move.
     * \param distance The distance by which to move the point.
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return The index to the polygon onto which we have moved the point.
     */
    static unsigned int moveOutside(const Shape& polygons, Point2LL& from, int distance = 0, int64_t max_dist2 = std::numeric_limits<int64_t>::max());

    /*!
     * Compute a point at a distance from a point on the boundary in orthogonal direction to the boundary.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     *
     * \param cpp The object holding the point on the boundary along with the information of which line segment the point is on.
     * \param distance The distance by which to move the point.
     * \return A point at a \p distance from the point in \p cpp orthogonal to the boundary there.
     */
    static Point2LL moveInside(const ClosestPointPolygon& cpp, const int distance);

    /*!
     * The opposite of moveInside.
     *
     * Compute a point at a distance from a point on the boundary in orthogonal direction to the boundary.
     * Given a \p distance more than zero, the point will end up outside, and conversely inside.
     *
     * \param cpp The object holding the point on the boundary along with the information of which line segment the point is on.
     * \param distance The distance by which to move the point.
     * \return A point at a \p distance from the point in \p cpp orthogonal to the boundary there.
     */
    static Point2LL moveOutside(const ClosestPointPolygon& cpp, const int distance);

    /*!
     * Moves the point \p from onto the nearest polygon or leaves the point as-is, when the comb boundary is not within \p distance.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
     * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
     *
     * \warning May give false positives.
     * Some checking is done to make sure we end up inside the polygon,
     * but it might still be the case that we end up outside:
     * when the closest point on the boundary is very close to another polygon
     *
     * \warning When using a \p loc_to_line_grid which contains more polygons than just \p polygons,
     * the results is only correct if \p from is already closest to \p polygons, rather than other polygons in the \p loc_to_line_grid.
     *
     * \param polygons The polygons onto which to move the point
     * \param from[in,out] The point to move.
     * \param preferred_dist_inside The preferred distance from the boundary to the point
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \param loc_to_line_polygons The original polygons with which the \p loc_to_line_grid has been created
     * \param loc_to_line_grid A SparseGrid mapping locations to line segments of \p polygons
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPointPolygon ensureInsideOrOutside(
        const Shape& polygons,
        Point2LL& from,
        int preferred_dist_inside,
        int64_t max_dist2 = std::numeric_limits<int64_t>::max(),
        const Shape* loc_to_line_polygons = nullptr,
        const LocToLineGrid* loc_to_line_grid = nullptr,
        const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     * Moves the point \p from onto the nearest polygon or leaves the point as-is, when the comb boundary is not within \p distance.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
     * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
     *
     * \warning May give false positives.
     * Some checking is done to make sure we end up inside the polygon,
     * but it might still be the case that we end up outside:
     * when the closest point on the boundary is very close to another polygon
     *
     * \warning When using a \p loc_to_line_grid which contains more polygons than just \p polygons,
     * the results is only correct if \p from is already closest to \p polygons, rather than other polygons in the \p loc_to_line_grid.
     *
     * \param polygons The polygons onto which to move the point
     * \param from[in,out] The point to move.
     * \param closest_polygon_point The point on \p polygons closest to \p from
     * \param preferred_dist_inside The preferred distance from the boundary to the point
     * \param loc_to_line_polygons The original polygons with which the \p loc_to_line_grid has been created
     * \param loc_to_line_grid A SparseGrid mapping locations to line segments of \p polygons
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPointPolygon ensureInsideOrOutside(
        const Shape& polygons,
        Point2LL& from,
        const ClosestPointPolygon& closest_polygon_point,
        int preferred_dist_inside,
        const Shape* loc_to_line_polygons = nullptr,
        const LocToLineGrid* loc_to_line_grid = nullptr,
        const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     *
     * \warning Assumes \p poly1_result and \p poly2_result have their pos and poly fields initialized!
     */
    static void walkToNearestSmallestConnection(ClosestPointPolygon& poly1_result, ClosestPointPolygon& poly2_result);

    /*!
     * Find the nearest closest point on a polygon from a given index.
     *
     * \param from The point from which to get the smallest distance.
     * \param polygon The polygon on which to find the point with the smallest distance.
     * \param start_idx The index of the point in the polygon from which to start looking.
     * \return The nearest point from \p start_idx going along the \p polygon (in both directions) with a locally minimal distance to \p from.
     */
    static ClosestPointPolygon findNearestClosest(const Point2LL& from, const Polygon& polygon, int start_idx);

    /*!
     * Find the nearest closest point on a polygon from a given index walking in one direction along the polygon.
     *
     * \param from The point from which to get the smallest distance.
     * \param polygon The polygon on which to find the point with the smallest distance.
     * \param start_idx The index of the point in the polygon from which to start looking.
     * \param direction The direction to walk: 1 for walking along the \p polygon, -1 for walking in opposite direction
     * \return The nearest point from \p start_idx going along the \p polygon with a locally minimal distance to \p from.
     */
    static ClosestPointPolygon findNearestClosest(const Point2LL& from, const Polygon& polygon, int start_idx, int direction);

    /*!
     * Find the point closest to \p from in all polygons in \p polygons.
     *
     * \note The penalty term is applied to the *squared* distance score
     *
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     */
    static ClosestPointPolygon findClosest(const Point2LL& from, const Shape& polygons, const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     * Find the point closest to \p from in the polygon \p polygon.
     *
     * \note The penalty term is applied to the *squared* distance score
     *
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     */
    static ClosestPointPolygon findClosest(const Point2LL& from, const Polygon& polygon, const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     * Find the nearest vertex to \p from in \p polys
     * \param from the point from where to look
     * \param polys The polygons in which to search
     * \return The nearest vertex on the polygons
     */
    static PolygonsPointIndex findNearestVert(const Point2LL& from, const Shape& polys);

    /*!
     * Find the nearest vertex to \p from in \p poly
     * \param from the point from where to look
     * \param poly The polygon in which to search
     * \return The index to the nearest vertex on the polygon
     */
    static unsigned int findNearestVert(const Point2LL& from, const Polygon& poly);

    /*!
     * Create a SparsePointGridInclusive mapping from locations to line segments occurring in the \p polygons
     *
     * \warning The caller of this function is responsible for deleting the returned object
     *
     * \param polygons The polygons for which to create the mapping
     * \param square_size The cell size used to bundle line segments (also used to chop up lines so that multiple cells contain the same long line)
     * \return A bucket grid mapping spatial locations to poly-point indices into \p polygons
     */
    static std::unique_ptr<LocToLineGrid> createLocToLineGrid(const Shape& polygons, int square_size);

    /*!
     * Find the line segment closest to a given point \p from within a cell-block of a size defined in the SparsePointGridInclusive \p loc_to_line
     *
     * \note The penalty term is applied to the *squared* distance score.
     * Note also that almost only nearby points are considered even when the penalty function would favour points farther away.
     *
     * \param from The location to find a polygon edge close to
     * \param polygons The polygons for which the \p loc_to_line has been built up
     * \param loc_to_line A SparsePointGridInclusive mapping locations to starting vertices of line segmetns of the \p polygons
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return The nearest point on the polygon if the polygon was within a distance equal to the cell_size of the SparsePointGridInclusive
     */
    static std::optional<ClosestPointPolygon>
        findClose(Point2LL from, const Shape& polygons, const LocToLineGrid& loc_to_line, const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     * Find the line segment closest to any point on \p from within cell-blocks of a size defined in the SparsePointGridInclusive \p destination_loc_to_line
     *
     * \note The penalty term is applied to the *squared* distance score.
     * Note also that almost only nearby points are considered even when the penalty function would favour points farther away.
     *
     * \param from The polygon for which to find a polygon edge close to
     * \param destination The polygons for which the \p destination_loc_to_line has been built up
     * \param destination_loc_to_line A SparsePointGridInclusive mapping locations to starting vertices of line segments of the \p destination
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return A collection of near crossing from the \p from polygon to the \p destination polygon. Each element in the sollection is a pair with as first a cpp in the \p from
     * polygon and as second a cpp in the \p destination polygon.
     */
    static std::vector<std::pair<ClosestPointPolygon, ClosestPointPolygon>> findClose(
        const Polygon& from,
        const Shape& destination,
        const LocToLineGrid& destination_loc_to_line,
        const std::function<int(Point2LL)>& penalty_function = no_penalty_function);

    /*!
     * Checks whether a given line segment collides with polygons as given in a loc_to_line grid.
     *
     * If the line segment doesn't intersect with any edge of the polygon, but
     * merely touches it, a collision is also reported. For instance, a
     * collision is reported when the an endpoint of the line is exactly on the
     * polygon, and when the line coincides with an edge.
     *
     * \param[in] from The start point
     * \param[in] to The end point
     * \param[in] loc_to_line A SparsePointGridInclusive mapping locations to starting vertices of line segmetns of the \p polygons
     * \param[out] collision_result (optional) The polygons segment intersecting with the line segment
     * \return whether the line segment collides with the boundary of the polygons
     */
    static bool polygonCollidesWithLineSegment(const Point2LL from, const Point2LL to, const LocToLineGrid& loc_to_line, PolygonsPointIndex* collision_result = nullptr);

    /*!
     * Find the next point (going along the direction of the polygon) with a distance \p dist from the point \p from within the \p poly.
     * Returns whether another point could be found within the \p poly which can be found before encountering the point at index \p start_idx.
     * The point \p from and the polygon \p poly are assumed to lie on the same plane.
     *
     * \param from The point from whitch to find a point on the polygon satisfying the conditions
     * \param start_idx the index of the prev poly point on the poly.
     * \param poly_start_idx The index of the point in the polygon which is to be handled as the start of the polygon. No point further than this point will be the result.
     */
    static bool getNextPointWithDistance(Point2LL from, int64_t dist, const OpenPolyline& poly, int start_idx, int poly_start_idx, GivenDistPoint& result);

    /*!
     * Walk a given \p distance along the polygon from a given point \p from on the polygon
     */
    template<class LineType>
    static ClosestPoint<LineType> walk(const ClosestPoint<LineType>& from, coord_t distance);

    /*!
     * Get the point on a polygon which intersects a line parallel to a line going through the starting point and through another point.
     *
     * Note that the looking direction \p forward doesn't neccesarily determine on which side of the line we cross a parallel line.
     * Depending on the geometry of the polygon the next intersection may be left or right of the input line.
     *
     * \param start The starting point of the search and the starting point of the line
     * \param line_to The end point of the line
     * \param dist The distance from the parallel line to the line defined by the previous two parameters
     * \param forward Whether to look forward from \p start in the direction of the polygon, or go in the other direction.
     * \return The earliest point on the polygon in the given direction which crosses a line parallel to the given one at the distance \p dist - if any
     */
    static std::optional<ClosestPointPolygon> getNextParallelIntersection(const ClosestPointPolygon& start, const Point2LL& line_to, const coord_t dist, const bool forward);

    /*!
     * Checks whether a given line segment collides with a given polygon(s).
     * The transformed_startPoint and transformed_endPoint should have the same
     * Y coordinate.
     *
     * If the line segment doesn't intersect with any edge of the polygon, but
     * merely touches it, a collision is also reported. For instance, a
     * collision is reported when the an endpoint of the line is exactly on the
     * polygon, and when the line coincides with an edge.
     *
     * \param poly The polygon
     * \param transformed_startPoint The start point transformed such that it is
     * on the same horizontal line as the end point
     * \param transformed_endPoint The end point transformed such that it is on
     * the same horizontal line as the start point
     * \param transformation_matrix The transformation applied to the start and
     * end point to be applied to the polygon(s)
     * \return whether the line segment collides with the boundary of the
     * polygon(s)
     */
    static bool
        polygonCollidesWithLineSegment(const Polygon& poly, const Point2LL& transformed_startPoint, const Point2LL& transformed_endPoint, PointMatrix transformation_matrix);

    /*!
     * Checks whether a given line segment collides with a given polygon(s).
     *
     * If the line segment doesn't intersect with any edge of the polygon, but
     * merely touches it, a collision is also reported. For instance, a
     * collision is reported when the an endpoint of the line is exactly on the
     * polygon, and when the line coincides with an edge.
     *
     * \param poly The polygon
     * \param startPoint The start point
     * \param endPoint The end point
     * \return whether the line segment collides with the boundary of the
     * polygon(s)
     */
    static bool polygonCollidesWithLineSegment(const Polygon& poly, const Point2LL& startPoint, const Point2LL& endPoint);

    /*!
     * Checks whether a given line segment collides with a given polygon(s).
     * The transformed_startPoint and transformed_endPoint should have the same
     * Y coordinate.
     *
     * If the line segment doesn't intersect with any edge of the polygon, but
     * merely touches it, a collision is also reported. For instance, a
     * collision is reported when the an endpoint of the line is exactly on the
     * polygon, and when the line coincides with an edge.
     *
     * \param poly The polygon
     * \param transformed_startPoint The start point transformed such that it is
     * on the same horizontal line as the end point
     * \param transformed_endPoint The end point transformed such that it is on
     * the same horizontal line as the start point
     * \param transformation_matrix The transformation applied to the start and
     * end point to be applied to the polygon(s)
     * \return whether the line segment collides with the boundary of the
     * polygon(s)
     */
    static bool polygonCollidesWithLineSegment(const Shape& polys, const Point2LL& transformed_startPoint, const Point2LL& transformed_endPoint, PointMatrix transformation_matrix);

    /*!
     * Checks whether a given line segment collides with a given polygon(s).
     *
     * If the line segment doesn't intersect with any edge of the polygon, but
     * merely touches it, a collision is also reported. For instance, a
     * collision is reported when the an endpoint of the line is exactly on the
     * polygon, and when the line coincides with an edge.
     *
     * \param poly The polygon
     * \param startPoint The start point
     * \param endPoint The end point
     * \return whether the line segment collides with the boundary of the
     * polygon(s)
     */
    static bool polygonCollidesWithLineSegment(const Shape& polys, const Point2LL& startPoint, const Point2LL& endPoint);

    /*!
     * Checks whether two polygon groups intersect - does a BB hit check first and if that succeeds, the full intersection
     *
     * \param poly_a A polygon group
     * \param poly_b Another polygon group
     * \return true if \p poly_a and \p poly_b intersect, false otherwise
     */
    static bool polygonsIntersect(const Polygon& poly_a, const Polygon& poly_b);

    /*!
     * Checks whether two polygons are adjacent (closer than \p max_gap)
     *
     * \param[in] inner_poly A polygon whose vertices will be tested to see if they are closer than \p max_gap to one of the lines in \p outer_poly
     * \param[in] outer_poly A polygon
     * \param[in] max_gap Polygons must be closer together than this distance to be considered adjacent.
     * \return true if a vertex in \p inner_poly is sufficiently close to a line in \p outer_poly, false otherwise
     */
    static bool polygonOutlinesAdjacent(const Polygon& inner_poly, const Polygon& outer_poly, const coord_t max_gap);

    /*!
     * Searches \p possible_adjacent_polys for polygons that are closer to \p poly than \p max_gap. The indices of adjacent polygons are stored in \p adjacent_poly_indices.
     *
     * \param[out] adjacent_poly_indices A vector that will contain the indices of the polygons that are adjacent to \p poly.
     * \param[in] poly The polygon that we are testing adjacency to.
     * \param[in] possible_adjacent_polys The vector of polygons we are testing.
     * \param[in] max_gap Polygons must be closer together than this distance to be considered adjacent.
     */
    static void
        findAdjacentPolygons(std::vector<unsigned>& adjacent_poly_indices, const Polygon& poly, const std::vector<const Polygon*>& possible_adjacent_polys, const coord_t max_gap);

    /*!
     * Calculate the Hamming Distance between two polygons relative to their own
     * surface areas.
     *
     * The Hamming Distance applied to polygons is interpreted as the area of
     * the symmetric difference between the polygons. In this case, we'll
     * divide this area by the total area of the two polygons.
     * \param poly_a One of the polygons to compute the distance between.
     * \param poly_b One of the polygons to compute the distance between.
     * \return The Hamming Distance relative to the total surface area of the
     * two polygons. This will be between 0.0 (the polygons are exactly equal)
     * and 1.0 (the polygons are completely disjunct).
     */
    static double relativeHammingDistance(const Shape& poly_a, const Shape& poly_b);

    /*!
     * Creates a regular polygon that is supposed to approximate a disc.
     *
     * \param mid The center of the disc.
     * \param radius The radius of the disc.
     * \param steps The numbers of segments (definition) of the generated disc.
     * \return A new Polygon containing the disc.
     */
    static Polygon makeDisc(const Point2LL& mid, const coord_t radius, const size_t steps);

    /*!
     * Creates a closed polyline that is supposed to approximate a circle.
     *
     * \param mid The center of the circle.
     * \param radius The radius of the circle.
     * \param segments The numbers of segments (definition) of the generated circle.
     * \tparam explicitly_closed Indicates whether the circle should be explicitely (or implicitely) closed
     * \return A new object containing the circle points.
     */
    template<typename T = ClosedPolyline, bool explicitely_closed = false, typename... VA>
    static T makeCircle(const Point2LL& mid, const coord_t radius, const size_t segments, VA... args)
    {
        T circle;
        const AngleRadians step_angle = (std::numbers::pi * 2) / static_cast<double>(segments);
        for (size_t step = 0; step < segments; ++step)
        {
            const AngleRadians angle = static_cast<double>(step) * step_angle;
            circle.emplace_back(makeCirclePoint(mid, radius, angle), args...);
        }

        if constexpr (explicitely_closed)
        {
            circle.push_back(circle.front());
        }

        return circle;
    }

    /*!
     * Create a point of a circle.
     *
     * \param mid The center of the circle.
     * \param radius The radius of the circle.
     * \param angle The point angular position
     * \return The coordinates of the point on the circle.
     */
    static Point2LL makeCirclePoint(const Point2LL& mid, const coord_t radius, const AngleRadians& angle);

    /*!
     * This creates a polyline which represents the shape of a wheel, which is kind of a "circular zigzag" pattern.
     *
     * \param mid The center of the circle.
     * \param inner_radius The radius of the wheel inner circle.
     * \param outer_radius The radius of the wheel outer circle.
     * \param semi_nb_spokes The semi number of spokes in the wheel. There will actually be N*2 spokes.
     * \param arc_angle_resolution The number of segments on each arc.
     * \return A new Polyline containing the circle.
     */
    static ClosedPolyline makeWheel(const Point2LL& mid, const coord_t inner_radius, const coord_t outer_radius, const size_t semi_nb_spokes, const size_t arc_angle_resolution);

    /*!
     * Connect all polygons to their holes using zero widths hole channels, so that the polygons and their outlines are connected together
     */
    static Shape connect(const Shape& input);

    static void fixSelfIntersections(const coord_t epsilon, Shape& polygon);

    static Shape unionManySmall(const Shape& polygon);

    /*!
     * Intersects a polygon with an AABB.
     * \param src The polygon that has to be intersected with an AABB
     * \param aabb The AABB with which the polygon that has to be intersected with
     * \return A new Polygon that is said intersection
     */
    static Shape clipPolygonWithAABB(const Shape& src, const AABB& aabb);

    /*!
     * Generate a few outset circles around a base, according to the given line width
     *
     * \param center The center of the outset
     * \param inner_radius The inner radius to start generating the outset from
     * \param outer_radius The outer radius to fit the outset into
     * \param line_width The actual line width to distance the polygons from each other (and from the base)
     * \param circle_definition The definition (number of segments) of the generated circles
     * \return The generated outset circles, and the outer radius or the shape
     */
    static std::tuple<ClosedLinesSet, coord_t>
        generateCirculatOutset(const Point2LL& center, const coord_t inner_radius, const coord_t outer_radius, const coord_t line_width, const size_t circle_definition);

    /*!
     * Generate inset circles inside the given base, until there is no space left, according to the given line width
     *
     * \param center The center of the inset
     * \param outer_radius The outer radius to start generating the inset from
     * \param line_width The actual line width to distance the polygons from each other (and from the base)
     * \param circle_definition The definition (number of segments) of the generated circles
     * \return The generated inset circles
     */
    static ClosedLinesSet generateCircularInset(const Point2LL& center, const coord_t outer_radius, const coord_t line_width, const size_t circle_definition);

private:
    /*!
     * Helper function for PolygonUtils::moveInside2: moves a point \p from which was moved onto \p closest_polygon_point towards inside/outside when it's not already
     * inside/outside by enough distance.
     *
     * \param closest_polygon_point The ClosestPointPolygon we have to move inside
     * \param distance The distance by which to move the point.
     * \param from[in,out] The point to move.
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPointPolygon _moveInside2(const ClosestPointPolygon& closest_polygon_point, const int distance, Point2LL& from, const int64_t max_dist2);
};

} // namespace cura

namespace std
{

template<>
struct hash<cura::ClosestPointPolygon>
{
    size_t operator()(const cura::ClosestPointPolygon& cpp) const
    {
        return std::hash<cura::Point2LL>()(cpp.p());
    }
};

} // namespace std

#endif // UTILS_POLYGON_UTILS_H
