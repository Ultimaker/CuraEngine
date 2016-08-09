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
#ifdef DEBUG
        Point last = poly.back();
#endif // DEBUG
        for (Point& p : poly) 
        {
            result.back().push_back(p);
#ifdef DEBUG
            // usually polygons shouldn't have such degenerate verts
            // in PolygonProximityLinker (where this function is (also) used) it is
            // required to not have degenerate verts, because verts are mapped
            // to links, but if two different verts are at the same place the mapping fails.
            assert(p != last);
            last = p;
#endif // DEBUG
        }
    }
}

void ListPolyIt::convertListPolygonsToPolygons(ListPolygons& list_polygons, Polygons& polygons)
{
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        polygons[poly_idx].clear();
        for (Point& p : list_polygons[poly_idx])
        {
            polygons[poly_idx].add(p);
        }
    }
}

}//namespace cura 
