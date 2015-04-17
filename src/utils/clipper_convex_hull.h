
#ifndef CLIPPER_CONVEX_HULL_INCLUDED_
#define CLIPPER_CONVEX_HULL_INCLUDED_ 1

#include "clipper.hpp"
//
// Find the convex hull of a set of (x,y) points. Works in-place on a Path
// After the call, 'points' will contain a subset of the original
// set, which form the convex hull.
// Could be:
//     - empty (if input is empty, or if there is an internal error)
//     - single point (if all input points are identical)
//     - 2 points (if >=2 input points, but all collinear)
//     - 3 or more points forming a convex polygon in CCW order.
//
// Returns 0  (or -1 if an internal
// error occurs, in which case the result will be empty).
// No case has been found in testing which causes an internal error. It's
// possible that this could occur if 64-bit multplies and adds overflow.
//
int findConvexHullInplace(ClipperLib::Path  & points);


//
// This returns a new Path with the convex-hull points in it.
//
// The returned points are in CCW order.
// Return value could be:
//     - empty (if input is empty, or if there is an internal error)
//     - single point (if all input points are identical)
//     - 2 points (if >=2 input points, but all collinear)
//     - 3 or more points forming a convex polygon in CCW order.
//
//
inline ClipperLib::Path findConvexHull(ClipperLib::Path const & points)
{
    ClipperLib::Path result = points;
    findConvexHullInplace(result);
    return result;
}

#endif
