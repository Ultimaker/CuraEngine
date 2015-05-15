/** Copyright (C) 2015 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef UTILS_POLYGON_UTILS_H
#define UTILS_POLYGON_UTILS_H

#include "polygon.h"

namespace cura 
{
    
//! performs an offset compared to an adjacent inset/outset and also computes the area created by gaps between the two consecutive insets/outsets
void offsetExtrusionWidth(Polygons& poly, bool inward, int extrusionWidth, Polygons& result, Polygons* in_between, bool avoidOverlappingPerimeters);

//! performs an offset and makes sure the lines don't overlap (ignores any area between the original poly and the resulting poly)
void offsetSafe(Polygons& poly, int distance, int extrusionWidth, Polygons& result, bool avoidOverlappingPerimeters);

//! performs offsets to make sure the lines don't overlap (ignores any area between the original poly and the resulting poly)
void removeOverlapping(Polygons& poly, int extrusionWidth, Polygons& result);

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
void findSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result, int sample_size);

/*!
 * 
 * \warning Assumes \p poly1_result and \p poly2_result have their pos and poly fields initialized!
 */
void walkToNearestSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result);

/*!
 * Find the nearest closest point on a polygon from a given index.
 * 
 * \param from The point from which to get the smallest distance.
 * \param polygon The polygon on which to find the point with the smallest distance.
 * \param start_idx The index of the point in the polygon from which to start looking.
 * \return The nearest point from \p start_idx going along the \p polygon (in both directions) with a locally minimal distance to \p from.
 */
ClosestPolygonPoint findNearestClosest(Point from, PolygonRef polygon, int start_idx);

/*!
 * Find the nearest closest point on a polygon from a given index walking in one direction along the polygon.
 * 
 * \param from The point from which to get the smallest distance.
 * \param polygon The polygon on which to find the point with the smallest distance.
 * \param start_idx The index of the point in the polygon from which to start looking.
 * \param direction The direction to walk: 1 for walking along the \p polygon, -1 for walking in opposite direction
 * \return The nearest point from \p start_idx going along the \p polygon with a locally minimal distance to \p from.
 */
ClosestPolygonPoint findNearestClosest(Point from, PolygonRef polygon, int start_idx, int direction);

/*!
 * Find the point closest to \p from in all polygons in \p polygons.
 */
ClosestPolygonPoint findClosest(Point from, Polygons& polygons);
    
/*!
 * Find the point closest to \p from in the polygon \p polygon.
 */
ClosestPolygonPoint findClosest(Point from, PolygonRef polygon);
    
/*!
 * Find the point closest to \p from on the line from \p p0 to \p p1
 */
Point getClosestOnLine(Point from, Point p0, Point p1);

/*!
 * Find the next point (going along the direction of the polygon) with a distance \p dist from the point \p from within the \p poly.
 * Returns whether another point could be found within the \p poly which can be found before encountering the point at index \p start_idx.
 * The point \p from and the polygon \p poly are assumed to lie on the same plane.
 * 
 * \param start_idx the index of the prev poly point on the poly.
 * \param poly_start_idx The index of the point in the polygon which is to be handled as the start of the polygon. No point further than this point will be the result.
 */
bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result);


}//namespace cura

#endif//POLYGON_OPTIMIZER_H
