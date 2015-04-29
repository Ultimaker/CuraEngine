/** Copyright (C) 2015 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef POLYGON_UTILS_H
#define POLYGON_UTILS_H

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
    Point p; //!< Result location
    PolygonRef poly; //!< Polygon in which the result was found
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
    ClosestPolygonPoint(Point p, int pos, PolygonRef poly) :  p(p), poly(poly), pos(pos) {};
    ClosestPolygonPoint(PolygonRef poly) : poly(poly) {};
};

/*!
 * A point within a polygon and the index of which segment in the polygon the point lies on.
 */
struct GivenDistPoint
{
    Point p; //!< Result location
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
};


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
