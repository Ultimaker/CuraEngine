/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_POLYGON_UTILS_H
#define UTILS_POLYGON_UTILS_H

#include "polygon.h"

namespace cura 
{

/*!
 * Result of finding the closest point to a given within a set of polygons, with extra information on where the point is.
 */
struct ClosestPolygonPoint
{
    Point location; //!< Result location
    PolygonRef poly; //!< Polygon in which the result was found
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
    ClosestPolygonPoint(Point p, int pos, PolygonRef poly) :  location(p), poly(poly), pos(pos) {};
    ClosestPolygonPoint(int pos, PolygonRef poly) : poly(poly), pos(pos) {};
    ClosestPolygonPoint(PolygonRef poly) : poly(poly), pos(-1) {};
};

/*!
 * A point within a polygon and the index of which segment in the polygon the point lies on.
 */
struct GivenDistPoint
{
    Point location; //!< Result location
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
};
    
class PolygonUtils 
{
public:
    //! performs an offset compared to an adjacent inset/outset and also computes the area created by gaps between the two consecutive insets/outsets
    static void offsetExtrusionWidth(const Polygons& poly, bool inward, int extrusionWidth, Polygons& result, Polygons* in_between, bool removeOverlappingPerimeters);

    /*!
    * performs an offset compared to an adjacent inset/outset and also computes the area created by gaps between the two consecutive insets/outsets.
    * This function allows for different extrusion widths between the two insets.
    */
    static void offsetSafe(const Polygons& poly, int distance, int offset_first_boundary, int extrusion_width, Polygons& result, Polygons* in_between, bool removeOverlappingPerimeters);

    //! performs an offset and makes sure the lines don't overlap (ignores any area between the original poly and the resulting poly)
    static void offsetSafe(const Polygons& poly, int distance, int extrusionWidth, Polygons& result, bool removeOverlappingPerimeters);

    //! performs offsets to make sure the lines don't overlap (ignores any area between the original poly and the resulting poly)
    static void removeOverlapping(const Polygons& poly, int extrusionWidth, Polygons& result);

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
    * Moves the point \p from onto the nearest polygon or leaves the point as-is, when the comb boundary is not within \p distance.
    * Given a \p distance more than zero, the point will end up inside, and conversely outside.
    * When the point is already in/outside by more than \p distance, \p from is unaltered, but the polygon is returned.
    * When the point is in/outside by less than \p distance, \p from is moved to the correct place.
    * 
    * \param polygons The polygons onto which to move the point
    * \param from The point to move.
    * \param distance The distance by which to move the point.
    * \param maxDist2 The squared maximal allowed distance from the point to the nearest polygon.
    * \return The index to the polygon onto which we have moved the point.
    */
    static unsigned int moveInside(Polygons& polygons, Point& from, int distance = 0, int64_t maxDist2 = std::numeric_limits<int64_t>::max());

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
    static ClosestPolygonPoint findNearestClosest(Point from, PolygonRef polygon, int start_idx);

    /*!
    * Find the nearest closest point on a polygon from a given index walking in one direction along the polygon.
    * 
    * \param from The point from which to get the smallest distance.
    * \param polygon The polygon on which to find the point with the smallest distance.
    * \param start_idx The index of the point in the polygon from which to start looking.
    * \param direction The direction to walk: 1 for walking along the \p polygon, -1 for walking in opposite direction
    * \return The nearest point from \p start_idx going along the \p polygon with a locally minimal distance to \p from.
    */
    static ClosestPolygonPoint findNearestClosest(Point from, PolygonRef polygon, int start_idx, int direction);

    /*!
    * Find the point closest to \p from in all polygons in \p polygons.
    */
    static ClosestPolygonPoint findClosest(Point from, Polygons& polygons);
        
    /*!
    * Find the point closest to \p from in the polygon \p polygon.
    */
    static ClosestPolygonPoint findClosest(Point from, PolygonRef polygon);

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
    static bool polygonCollidesWithlineSegment(PolygonRef poly, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix);

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
    static bool polygonCollidesWithlineSegment(PolygonRef poly, Point& startPoint, Point& endPoint);

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
    static bool polygonCollidesWithlineSegment(Polygons& polys, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix);

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
    static bool polygonCollidesWithlineSegment(Polygons& polys, Point& startPoint, Point& endPoint);
};


}//namespace cura

#endif//POLYGON_OPTIMIZER_H
