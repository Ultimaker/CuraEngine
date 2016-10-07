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

void ListPolyIt::convertPolygonsToLists(Polygons& polys, const std::vector<int>& start_indices, ListPolygons& result)
{
    const int polygon_count = polys.size();
    assert(start_indices.size() == polys.size() && "There should be one start point per polygon, polys.size() == start_indices.size()");
    for (int i = 0; i < polygon_count; ++i)
    {
        PolygonRef poly = polys[i];
        int start_point_index = start_indices[i];
        result.emplace_back();
        convertPolygonToList(poly, start_point_index, result.back());
    }
}

void ListPolyIt::convertPolygonToList(const PolygonRef& poly, ListPolygon& result)
{
#ifdef DEBUG
    Point last = poly.back();
#endif // DEBUG
    for (const Point& p : poly)
    {
        result.push_back(p);
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

void ListPolyIt::convertPolygonToList(PolygonRef poly, const int start_index, ListPolygon& result)
{
    result.push_back(poly[start_index]);
    const int point_count = poly.size();
    for (int i = start_index + 1; i < point_count; ++i)
    {
        Point& p = poly[i];
        if (p != result.back())
        {
            result.push_back(p);
        }
        else
        {
#ifdef DEBUG
            // usually polygons shouldn't have such degenerate verts
            // in PolygonProximityLinker (where this function is (also) used) it is
            // required to not have degenerate verts, because verts are mapped
            // to links, but if two different verts are at the same place the mapping fails.
            assert(p != result.back());
#endif // DEBUG
        }
    }
    for (int i = 0; i < start_index; ++i)
    {
        Point& p = poly[i];
        if (p != result.back())
        {
            result.push_back(p);
        }
        else
        {
#ifdef DEBUG
            // usually polygons shouldn't have such degenerate verts
            // in PolygonProximityLinker (where this function is (also) used) it is
            // required to not have degenerate verts, because verts are mapped
            // to links, but if two different verts are at the same place the mapping fails.
            assert(p != result.back());
#endif // DEBUG
        }
        if (result.front() == result.back())
        {
#ifdef DEBUG
            assert(result.front() != result.back());
#endif // DEBUG
            result.pop_back();
        }
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

void ListPolyIt::convertListPolygonsToPolygons(ListPolygons& list_polygons, std::vector<ListPolygon::const_iterator>& start_point_iterator, Polygons& polygons)
{
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        polygons[poly_idx].clear();
        convertListPolygonToPolygon(list_polygons[poly_idx], start_point_iterator[poly_idx], polygons[poly_idx]);
    }
}

void ListPolyIt::convertListPolygonToPolygon(ListPolygon& list_polygon, PolygonRef polygon)
{
    for (Point& p : list_polygon)
    {
        polygon.add(p);
    }
}

void ListPolyIt::convertListPolygonToPolygon(ListPolygon& list_polygon, ListPolygon::const_iterator start_iterator, PolygonRef polygon)
{
    auto it = start_iterator;
    for (; it != list_polygon.end(); ++it)
    {
        polygon.add(*it);
    }
    for (it = list_polygon.begin(); it != start_iterator; ++it)
    {
        polygon.add(*it);
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
