/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_POLYGON_UTILS_H
#define UTILS_POLYGON_UTILS_H

#include <functional> // function

#include "polygon.h"
#include "BucketGrid2D.h"

namespace cura 
{

/*!
 * Result of finding the closest point to a given within a set of polygons, with extra information on where the point is.
 */
struct ClosestPolygonPoint
{
    Point location; //!< Result location
    PolygonRef poly; //!< Polygon in which the result was found
    unsigned int poly_idx; //!< The index of the polygon in some Polygons where ClosestPolygonPoint::poly can be found
    unsigned int point_idx; //!< Index to the first point in the polygon of the line segment on which the result was found
    ClosestPolygonPoint(Point p, int pos, PolygonRef poly) :  location(p), poly(poly), poly_idx(NO_INDEX), point_idx(pos) {};
    ClosestPolygonPoint(Point p, int pos, PolygonRef poly, int poly_idx) :  location(p), poly(poly), poly_idx(poly_idx), point_idx(pos) {};
    ClosestPolygonPoint(PolygonRef poly) : poly(poly), poly_idx(NO_INDEX), point_idx(NO_INDEX) {};
};

/*!
 * A point within a polygon and the index of which segment in the polygon the point lies on.
 */
struct GivenDistPoint
{
    Point location; //!< Result location
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
};

struct PolygonsPointIndex
{
    unsigned int poly_idx;
    unsigned int point_idx;
    PolygonsPointIndex()
    : poly_idx(0)
    , point_idx(0)
    {
    }
    PolygonsPointIndex(unsigned int poly_idx, unsigned int point_idx)
    : poly_idx(poly_idx)
    , point_idx(point_idx)
    {
    }
};

class PolygonUtils 
{
public:
    static const std::function<int(Point)> no_penalty_function; //!< Function always returning zero

    /*!
    * Get a point from the \p poly with a given \p offset.
    * 
    * \param poly The polygon.
    * \param point_idx The index of the point in the polygon.
    * \param offset The distance the point has to be moved outward from the polygon.
    * \return A point at the given distance inward from the point on the boundary polygon.
    */
    static Point getBoundaryPointWithOffset(PolygonRef poly, unsigned int point_idx, int64_t offset);

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
    static unsigned int moveInside(const Polygons& polygons, Point& from, int distance = 0, int64_t max_dist2 = std::numeric_limits<int64_t>::max());

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
     * \return The point on the polygon closest to \p from
     */
    static ClosestPolygonPoint moveInside2(const Polygons& polygons, Point& from, int distance = 0, int64_t max_dist2 = std::numeric_limits<int64_t>::max());

    /*!
     * Moves the point \p from onto the nearest segment of \p polygon or leaves the point as-is, when the comb boundary is not within the root of \p max_dist2 distance.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
     * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
     * 
     * \param polygon The polygon onto which to move the point
     * \param from[in,out] The point to move.
     * \param distance The distance by which to move the point.
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPolygonPoint moveInside2(const PolygonRef polygon, Point& from, int distance = 0, int64_t max_dist2 = std::numeric_limits<int64_t>::max());

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
     * \return The index to the polygon onto which we have moved the point.
     */
    static unsigned int moveOutside(const Polygons& polygons, Point& from, int distance = 0, int64_t max_dist2 = std::numeric_limits<int64_t>::max());
    
    /*!
     * Compute a point at a distance from a point on the boundary in orthogonal direction to the boundary.
     * Given a \p distance more than zero, the point will end up inside, and conversely outside.
     * 
     * \param cpp The object holding the point on the boundary along with the information of which line segment the point is on.
     * \param distance The distance by which to move the point.
     * \return A point at a \p distance from the point in \p cpp orthogonal to the boundary there.
     */
    static Point moveInside(const ClosestPolygonPoint& cpp, const int distance);
    
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
    static Point moveOutside(const ClosestPolygonPoint& cpp, const int distance);

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
     * \param polygons The polygons onto which to move the point
     * \param from[in,out] The point to move.
     * \param preferred_dist_inside The preferred distance from the boundary to the point
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPolygonPoint ensureInsideOrOutside(const Polygons& polygons, Point& from, int preferred_dist_inside, int64_t max_dist2 = std::numeric_limits<int64_t>::max());

    /*!
    * Find the two points in two polygons with the smallest distance.
    * 
    * Note: The amount of preliminary distance checks is quadratic in \p sample_size : `O(sample_size ^2)`.
    * Further convergence time depends on polygon size and shape.
    * 
    * \warning The ClosestPolygonPoint::poly fields output parameters should be initialized with the polygons for which to find the smallest connection.
    * 
    * \param poly1_result Output parameter: the point at the one end of the smallest connection between its poly and \p poly2_result.poly.
    * \param poly2_result Output parameter: the point at the other end of the smallest connection between its poly and \p poly1_result.poly.
    * \param sample_size The number of points on each polygon to start the hill climbing search from. 
    */
    static void findSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result, int sample_size);

    /*!
    * 
    * \warning Assumes \p poly1_result and \p poly2_result have their pos and poly fields initialized!
    */
    static void walkToNearestSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result);

    /*!
    * Find the nearest closest point on a polygon from a given index.
    * 
    * \param from The point from which to get the smallest distance.
    * \param polygon The polygon on which to find the point with the smallest distance.
    * \param start_idx The index of the point in the polygon from which to start looking.
    * \return The nearest point from \p start_idx going along the \p polygon (in both directions) with a locally minimal distance to \p from.
    */
    static ClosestPolygonPoint findNearestClosest(Point from, const PolygonRef polygon, int start_idx);

    /*!
    * Find the nearest closest point on a polygon from a given index walking in one direction along the polygon.
    * 
    * \param from The point from which to get the smallest distance.
    * \param polygon The polygon on which to find the point with the smallest distance.
    * \param start_idx The index of the point in the polygon from which to start looking.
    * \param direction The direction to walk: 1 for walking along the \p polygon, -1 for walking in opposite direction
    * \return The nearest point from \p start_idx going along the \p polygon with a locally minimal distance to \p from.
    */
    static ClosestPolygonPoint findNearestClosest(const Point from, const PolygonRef polygon, int start_idx, int direction);

    /*!
     * Find the point closest to \p from in all polygons in \p polygons.
     * 
     * \note The penalty term is applied to the *squared* distance score
     * 
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     */
    static ClosestPolygonPoint findClosest(Point from, const Polygons& polygons, const std::function<int(Point)>& penalty_function = no_penalty_function);
        
    /*!
     * Find the point closest to \p from in the polygon \p polygon.
     * 
     * \note The penalty term is applied to the *squared* distance score
     * 
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     */
    static ClosestPolygonPoint findClosest(Point from, const PolygonRef polygon, const std::function<int(Point)>& penalty_function = no_penalty_function);

    /*!
     * Create a BucketGrid mapping from locations to line segments occurring in the \p polygons
     * 
     * \warning The caller of this function is responsible for deleting the returned object
     * 
     * \param polygons The polygons for which to create the mapping
     * \param square_size The cell size used to bundle line segments (also used to chop up lines so that multiple cells contain the same long line)
     * \return A bucket grid mapping spatial locations to poly-point indices into \p polygons
     */
    static BucketGrid2D<PolygonsPointIndex>* createLocToLineGrid(const Polygons& polygons, int square_size);

    /*!
     * Find the line segment closest to a given point \p from within a cell-block of a size defined in the BucketGrid \p loc_to_line
     * 
     * \note The penalty term is applied to the *squared* distance score.
     * Note also that almost only nearby points are considered even when the penalty function would favour points farther away.
     * 
     * \param from The location to find a polygon edge close to
     * \param polygons The polygons for which the \p loc_to_line has been built up
     * \param loc_to_line A BucketGrid mapping locations to starting vertices of line segmetns of the \p polygons 
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return The nearest point on the polygon if the polygon was within a distance equal to the cell_size of the BucketGrid
     */
    static ClosestPolygonPoint* findClose(Point from, const Polygons& polygons, const BucketGrid2D<PolygonsPointIndex>& loc_to_line, const std::function<int(Point)>& penalty_function = no_penalty_function);
    
    /*!
     * Find the line segment closest to any point on \p from within cell-blocks of a size defined in the BucketGrid \p destination_loc_to_line
     * 
     * \note The penalty term is applied to the *squared* distance score.
     * Note also that almost only nearby points are considered even when the penalty function would favour points farther away.
     * 
     * \param from The polygon for which to find a polygon edge close to
     * \param destination The polygons for which the \p destination_loc_to_line has been built up
     * \param destination_loc_to_line A BucketGrid mapping locations to starting vertices of line segments of the \p destination 
     * \param penalty_function A function returning a penalty term on the squared distance score of a candidate point.
     * \return A collection of near crossing from the \p from polygon to the \p destination polygon. Each element in the sollection is a pair with as first a cpp in the \p from polygon and as second a cpp in the \p destination polygon.
     */
    static std::vector<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> findClose(const PolygonRef from, const Polygons& destination, const BucketGrid2D< PolygonsPointIndex >& destination_loc_to_line, const std::function<int(Point)>& penalty_function = no_penalty_function);

    /*!
    * Find the next point (going along the direction of the polygon) with a distance \p dist from the point \p from within the \p poly.
    * Returns whether another point could be found within the \p poly which can be found before encountering the point at index \p start_idx.
    * The point \p from and the polygon \p poly are assumed to lie on the same plane.
    * 
    * \param from The point from whitch to find a point on the polygon satisfying the conditions
    * \param start_idx the index of the prev poly point on the poly.
    * \param poly_start_idx The index of the point in the polygon which is to be handled as the start of the polygon. No point further than this point will be the result.
    */
    static bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result);



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
    static bool polygonCollidesWithlineSegment(const PolygonRef poly, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix);

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
    static bool polygonCollidesWithlineSegment(const PolygonRef poly, Point& startPoint, Point& endPoint);

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
    static bool polygonCollidesWithlineSegment(const Polygons& polys, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix);

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
    static bool polygonCollidesWithlineSegment(const Polygons& polys, Point& startPoint, Point& endPoint);

private:
    /*!
     * Helper function for PolygonUtils::moveInside2: moves a point \p from which was moved onto \p closest_polygon_point towards inside/outside when it's not already inside/outside by enough distance.
     * 
     * \param closest_polygon_point The ClosestPolygonPoint we have to move inside
     * \param distance The distance by which to move the point.
     * \param from[in,out] The point to move.
     * \param max_dist2 The squared maximal allowed distance from the point to the nearest polygon.
     * \return The point on the polygon closest to \p from
     */
    static ClosestPolygonPoint _moveInside2(const ClosestPolygonPoint& closest_polygon_point, const int distance, Point& from, int64_t max_dist2);
    
};


}//namespace cura

#endif//POLYGON_OPTIMIZER_H
