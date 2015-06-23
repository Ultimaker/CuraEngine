#include "wallOverlap.h"
#include "debug.h"

namespace cura 
{
    
void WallOverlapComputation::findOverlapPoints()
{
    // convert to list polygons for insertion of points
    convertPolygonsToLists(polygons, list_polygons); 
    
    // init loc_to_list_poly_idx
//     for (unsigned int poly_idx = 0; poly_idx < list_polygons.size(); poly_idx++)
//     {
//         for (ListPolygon::iterator it = list_polygons[poly_idx].begin(); it != list_polygons[poly_idx].end(); ++it)
//         {
//             loc_to_list_poly_idx.emplace(*it, it);
//         }
//     }
    
    
    for (unsigned int poly_idx = 0; poly_idx < list_polygons.size(); poly_idx++)
    {
        ListPolygon& poly = list_polygons[poly_idx];
        for (unsigned int poly2_idx = 0; poly2_idx <= poly_idx; poly2_idx++)
        {
            ListPolygon& poly2 = list_polygons[poly2_idx];
            for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
            {
                Point& p = *it;
                if (poly_idx == poly2_idx)
                {
                    findOverlapPoints(p, poly2, it);
                }
                else 
                {
                    findOverlapPoints(p, poly2);
                }
            }
        }
    }
    
    // convert list polygons back
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        convertListPolygonToPolygon(list_polygons[poly_idx], polygons[poly_idx]);
        DEBUG_SHOW(list_polygons[poly_idx].size());
    }
//     for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
//     {
//         PolygonRef poly = polygons[poly_idx];
//         for (unsigned int poly2_idx = 0; poly2_idx <= poly_idx; poly2_idx++)
//         {
//             PolygonRef poly2 = polygons[poly2_idx];
//             for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
//             {
//                 Point& p = poly[point_idx];
//                 findOverlapPoints(p, poly2);
//             }
//         }
//     }
}



void WallOverlapComputation::findOverlapPoints(Point from, PolygonRef to_poly)
{
    ListPolygon converted;
    convertPolygonToList(to_poly, converted);
    findOverlapPoints(from, converted);
    convertListPolygonToPolygon(converted, to_poly); // apply point additions
}

    
void WallOverlapComputation::convertPolygonsToLists(Polygons& polys, ListPolygons& result)
{
    for (PolygonRef poly : polys)
    {
        result.emplace_back();
        convertPolygonToList(poly, result.back());
    }
}    

void WallOverlapComputation::convertPolygonToList(PolygonRef poly, ListPolygon& result)
{
    for (Point& p : poly) 
    {
        result.push_back(p);
    }
}

void WallOverlapComputation::convertListPolygonToPolygon(ListPolygon& poly, PolygonRef result)
{
    result.clear();
    for (Point& p : poly)
    {
        result.add(p);
    }
}
    
void WallOverlapComputation::findOverlapPoints(Point from, ListPolygon& to_list_poly)
{
    findOverlapPoints(from, to_list_poly, to_list_poly.begin());
}

void WallOverlapComputation::findOverlapPoints(Point from, ListPolygon& to_list_poly, ListPolygon::iterator start)
{
    Point last_point = to_list_poly.back();
    for (ListPolygon::iterator it = start; it != to_list_poly.end(); ++it)
    {
        Point& point = *it;
        if (from == last_point || from == point )
        { // we currently consider a linesegment directly connected to [from]
            last_point = point;
            continue;
        }
        Point closest = getClosestOnLine(from, last_point, point);
        
        int64_t dist2 = vSize2(closest - from);
        
        if (dist2 > lineWidth * lineWidth)
        { // line segment too far away to have overlap
            last_point = point;
            continue;
        }
        
        int64_t dist = sqrt(dist2);
        
        if (closest == last_point)
        {
            addOverlapPoint(from, last_point, dist);
        }
        else if (closest == point)
        {
            addOverlapPoint(from, point, dist);
        }
        else 
        {
            ListPolygon::iterator new_it = to_list_poly.insert(it, closest);
             addOverlapPoint(from, closest, dist);
        }
        
        last_point = point;
    }
    
}


WallOverlapPointLink WallOverlapComputation::addOverlapPoint(Point& from, Point& to, int64_t dist)
{
    WallOverlapPointLink link(from, to, dist);
    std::pair<WallOverlapPointLinks::iterator, bool> result =
        overlap_point_links.emplace(link, false);
        
    if (! result.second)
    {
        DEBUG_PRINTLN("couldn't emplace in overlap_point_links! : ");
        DEBUG_PRINTLN(link.a << " - " << link.b <<  " @ " << link.dist << " < old ");
        DEBUG_PRINTLN(result.first->first.a << " - "<< result.first->first.b << " @ " << result.first->first.dist);
    }
    
    return link;
}

void WallOverlapComputation::addOverlapEndings()
{
//     for (WallOverlapPointLink& link : end_points)
//     {
//         // assume positive direction for a, negative for b
//         Loc2ListPolyIndex::iterator a_it_f = loc_to_list_poly_idx.find(link.a);
//         if (a_it_f == loc_to_list_poly_idx.end()) { DEBUG_PRINTLN(" ERROR:  cannot find point a!! "); }
//         ListPolygon::iterator a_it = a_it_f->second;
//         
//         Loc2ListPolyIndex::iterator b_it_f = loc_to_list_poly_idx.find(link.b);
//         if (b_it_f == loc_to_list_poly_idx.end()) { DEBUG_PRINTLN(" ERROR:  cannot find point b!! "); }
//         ListPolygon::iterator b_it = b_it_f->second;
//         
//         ListPolygon::iterator a_it_next = a_it;
//         a_it_next++;
//         if (a_it_next == 
//     }
    
    for (ListPolygon& poly : list_polygons)
    {
        for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
        {
            Point& p = *it;
            
        }
    }
}


    
void WallOverlapComputation::createPoint2LinkMap()
{
    for (WallOverlapPointLinks::iterator it = overlap_point_links.begin(); it != overlap_point_links.end(); ++it)
    {
        addToPoint2LinkMap(it->first.a, it);
        addToPoint2LinkMap(it->first.b, it);
    }
}
    
void WallOverlapComputation::addToPoint2LinkMap(Point p, WallOverlapPointLinks::iterator it)
{
    point_to_link.emplace(p, it);
    // TODO: what to do if the map already contained a link? > three-way overlap
}

float WallOverlapComputation::getFlow(Point& from, Point& to)
{
    Point2Link::iterator from_link_pair = point_to_link.find(from);
    if (from_link_pair == point_to_link.end()) { return 1; }
    Point2Link::iterator to_link_pair = point_to_link.find(to);
    if (to_link_pair == point_to_link.end()) { return 1; }
    
    WallOverlapPointLinks::iterator from_link = from_link_pair->second;
    WallOverlapPointLinks::iterator to_link = to_link_pair->second;
    
    if (!from_link->second || !to_link->second)
    {
//         overlap_point_links.emplace_hint(from_link, from_link->first, true);
//         overlap_point_links.emplace_hint(to_link, to_link->first, true);
//         from_link->second = true;
        to_link->second = true;
        return 1;
    }
//     from_link->second = true;
    to_link->second = true;
    
//     overlap_point_links.emplace_hint(from_link, from_link->first, true);
//     overlap_point_links.emplace_hint(to_link, to_link->first, true);
    
    // both points have already been passed
    
    float avg_link_dist = 0.5 * ( INT2MM(from_link->first.dist) + INT2MM(to_link->first.dist) );
   
    float ratio = avg_link_dist / INT2MM(lineWidth);

    return ratio;
}

    
} // namespace cura 
