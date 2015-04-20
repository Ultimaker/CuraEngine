/** Copyright (C) 2013 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef POLYGON_UTILS_H
#define POLYGON_UTILS_H

#include "polygon.h"

namespace cura {

Polygon convexHull(PolygonRef poly);
    
//! performs an offset compared to an adjacent inset/outset and also computes the area created by gaps between the two consecutive insets/outsets
void offsetExtrusionWidth(Polygons& poly, bool inward, int extrusionWidth, Polygons& result, Polygons* in_between, bool avoidOverlappingPerimeters);

//! performs an offset and makes sure the lines don't overlap (ignores any area between the original poly and the resulting poly)
void offsetSafe(Polygons& poly, int distance, int extrusionWidth, Polygons& result, bool avoidOverlappingPerimeters);

//! performs offsets to make sure the lines don't overlap (ignores any area between the original poly and the resulting poly)
void removeOverlapping(Polygons& poly, int extrusionWidth, Polygons& result);
}//namespace cura

#endif//POLYGON_OPTIMIZER_H
