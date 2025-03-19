// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ListPolyIt.h"

#include <cassert>
#include <cmath> // isfinite
#include <sstream> // ostream

#include "geometry/Polygon.h"
#include "utils/AABB.h" // for debug output svg html
#include "utils/SVG.h"

namespace cura
{


void ListPolyIt::convertPolygonsToLists(const Shape& shape, ListPolygons& result)
{
    for (const Polygon& poly : shape)
    {
        result.emplace_back();
        convertPolygonToList(poly, result.back());
    }
}

void ListPolyIt::convertPolygonToList(const Polygon& poly, ListPolygon& result)
{
#ifdef DEBUG
    Point2LL last = poly.back();
#endif // DEBUG
    for (const Point2LL& p : poly)
    {
        result.push_back(p);
#ifdef DEBUG
        // usually polygons shouldn't have such degenerate verts. It is
        // required to not have degenerate verts, because verts are mapped
        // to links, but if two different verts are at the same place the mapping fails.
        assert(p != last);
        last = p;
#endif // DEBUG
    }
}


void ListPolyIt::convertListPolygonsToPolygons(const ListPolygons& list_polygons, Shape& polygons)
{
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        polygons[poly_idx].clear();
        convertListPolygonToPolygon(list_polygons[poly_idx], polygons[poly_idx]);
    }
}

void ListPolyIt::convertListPolygonToPolygon(const ListPolygon& list_polygon, Polygon& polygon)
{
    for (const Point2LL& p : list_polygon)
    {
        polygon.push_back(p);
    }
}

ListPolyIt ListPolyIt::insertPointNonDuplicate(const ListPolyIt before, const ListPolyIt after, const Point2LL to_insert)
{
    if (to_insert == before.p())
    {
        return before;
    }
    else if (to_insert == after.p())
    {
        return after;
    }
    else
    {
        ListPolygon& poly = *after.poly_;
        return ListPolyIt(poly, poly.insert(after.it_, to_insert));
    }
}


} // namespace cura
