#include "ListPolyIt.h"

#include <cmath> // isfinite
#include <sstream> // ostream

#include "AABB.h" // for debug output svg html
#include "SVG.h"

namespace cura 
{


void ListPolyIt::convertPolygonsToLists(const Polygons& polys, ListPolygons& result, bool remove_duplicates)
{
    for (ConstPolygonRef poly : polys)
    {
        result.emplace_back();
        convertPolygonToList(poly, result.back(), remove_duplicates);
    }
}

void ListPolyIt::convertPolygonToList(ConstPolygonRef poly, ListPolygon& result, bool remove_duplicates)
{
    if (remove_duplicates)
    {
        Point last = poly.back();
        for (const Point& p : poly)
        {
            if (p != last)
            {
                result.push_back(p);
                last = p;
            }
        }
    }
    else
    {
        for (const Point& p : poly)
        {
            result.push_back(p);
        }
    }
}


void ListPolyIt::convertListPolygonsToPolygons(const ListPolygons& list_polygons, Polygons& polygons, bool remove_duplicates)
{
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        polygons[poly_idx].clear();
        convertListPolygonToPolygon(list_polygons[poly_idx], polygons[poly_idx], remove_duplicates);
    }
}

void ListPolyIt::convertListPolygonToPolygon(const ListPolygon& list_polygon, PolygonRef polygon, bool remove_duplicates)
{
    if (remove_duplicates)
    {
        Point last = list_polygon.back();
        for (const Point& p : list_polygon)
        {
            if (p != last)
            {
                polygon.add(p);
                last = p;
            }
        }
    }
    else
    {
        for (const Point& p : list_polygon)
        {
            polygon.add(p);
        }
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
