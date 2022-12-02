//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ListPolyIt.h"

#include <cmath> // isfinite
#include <sstream> // ostream

#include "utils/AABB.h" // for debug output svg html
#include "utils/SVG.h"

namespace cura 
{


void ListPolyIt::convertPolygonsToLists(const Polygons& polys, ListPolygons& result)
{
    for (ConstPolygonRef poly : polys)
    {
        result.emplace_back();
        convertPolygonToList(poly, result.back());
    }
}

void ListPolyIt::convertPolygonToList(ConstPolygonRef poly, ListPolygon& result)
{
#ifdef DEBUG
    Point last = poly.back();
#endif // DEBUG
    for (const Point& p : poly)
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


void ListPolyIt::convertListPolygonsToPolygons(const ListPolygons& list_polygons, Polygons& polygons)
{
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        polygons[poly_idx].clear();
        convertListPolygonToPolygon(list_polygons[poly_idx], polygons[poly_idx]);
    }
}

void ListPolyIt::convertListPolygonToPolygon(const ListPolygon& list_polygon, PolygonRef polygon)
{
    for (const Point& p : list_polygon)
    {
        polygon.add(p);
    }
}

ListPolyIt ListPolyIt::insertPointNonDuplicate(const ListPolyIt before, const ListPolyIt after, const Point to_insert)
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
        ListPolygon& poly = *after.poly;
        return ListPolyIt(poly, poly.insert(after.it, to_insert));
    }
}



}//namespace cura 
