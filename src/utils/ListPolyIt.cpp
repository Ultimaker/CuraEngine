#include "ListPolyIt.h"

#include <cmath> // isfinite
#include <sstream> // ostream

#include "AABB.h" // for debug output svg html
#include "SVG.h"

namespace cura 
{


void ListPolyIt::convertPolygonsToLists(Polygons& polys, ListPolygons& result)
{
    for (PolygonRef poly : polys)
    {
        result.emplace_back();
        convertPolygonToList(poly, result.back());
    }
}

void ListPolyIt::convertPolygonToList(PolygonRef poly, ListPolygon& result)
{
    for (Point& p : poly) 
    {
        result.push_back(p);
    }
}


void ListPolyIt::convertListPolygonsToPolygons(ListPolygons& list_polygons, Polygons& polygons)
{
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        polygons[poly_idx].clear();
        convertListPolygonToPolygon(list_polygons[poly_idx], polygons[poly_idx]);
    }
}

void ListPolyIt::convertListPolygonToPolygon(ListPolygon& list_polygon, PolygonRef polygon)
{
    for (Point& p : list_polygon)
    {
        polygon.add(p);
    }
}


}//namespace cura 
