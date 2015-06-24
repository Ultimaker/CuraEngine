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
            for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
            {
                ListPolyIt lpi(poly, it);
                if (poly_idx == poly2_idx)
                {
                    findOverlapPoints(lpi, poly2_idx, it);
                }
                else 
                {
                    findOverlapPoints(lpi, poly2_idx);
                }
            }
        }
    }
    
    // convert list polygons back
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        convertListPolygonToPolygon(list_polygons[poly_idx], polygons[poly_idx]);
//         DEBUG_SHOW(list_polygons[poly_idx].size());
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

/*

void WallOverlapComputation::findOverlapPoints(ListPolyIt from, PolygonRef to_poly)
{
    ListPolygon converted;
    convertPolygonToList(to_poly, converted);
    findOverlapPoints(from, converted);
    convertListPolygonToPolygon(converted, to_poly); // apply point additions
}*/

    
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
    
void WallOverlapComputation::findOverlapPoints(ListPolyIt from, unsigned int to_list_poly_idx)
{
    findOverlapPoints(from, to_list_poly_idx, list_polygons[to_list_poly_idx].begin());
}

void WallOverlapComputation::findOverlapPoints(ListPolyIt from_it, unsigned int to_list_poly_idx, ListPolygon::iterator start)
{
    ListPolygon& to_list_poly = list_polygons[to_list_poly_idx];
    Point& from = from_it.p();
    ListPolygon::iterator last_it = to_list_poly.end();
    last_it--;
    for (ListPolygon::iterator it = start; it != to_list_poly.end(); ++it)
    {
        Point& last_point = *last_it;
        Point& point = *it;
        if (from == last_point || from == point )
        { // we currently consider a linesegment directly connected to [from]
            last_it = it;
            continue;
        }
        Point closest = getClosestOnLine(from, last_point, point);
        
        int64_t dist2 = vSize2(closest - from);
        
        if (dist2 > lineWidth * lineWidth)
        { // line segment too far away to have overlap
            last_it = it;
            continue;
        }
        
        int64_t dist = sqrt(dist2);
        
        if (closest == last_point)
        {
//             addOverlapPoint(from_it, ListPolyIt(to_list_poly, last_it), dist);
        }
        else if (closest == point)
        {
//             addOverlapPoint(from_it, ListPolyIt(to_list_poly, it), dist);
        }
        else 
        {
//             ListPolygon::iterator new_it = to_list_poly.insert(it, closest);
//             addOverlapPoint(from_it, ListPolyIt(to_list_poly, new_it), dist);
        }
        
        last_it = it;
    }
    
}


void WallOverlapComputation::addOverlapPoint(ListPolyIt from, ListPolyIt to, int64_t dist)
{
    WallOverlapPointLink link(from, to, dist);
    std::pair<WallOverlapPointLinks::iterator, bool> result =
        overlap_point_links.emplace(link, false);
        
    if (! result.second)
    {
        DEBUG_PRINTLN("couldn't emplace in overlap_point_links! : ");
//         DEBUG_PRINTLN(link.a << " - " << link.b <<  " @ " << link.dist << " < old ");
//         DEBUG_PRINTLN(result.first->first.a << " - "<< result.first->first.b << " @ " << result.first->first.dist);
    }
    else 
    {
        WallOverlapPointLinks::iterator it = result.first;
        addToPoint2LinkMap(*it->first.a.it, it);
        addToPoint2LinkMap(*it->first.b.it, it);
    }
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
    
//     for (ListPolygon& poly : list_polygons)
//     {
//         for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
//         {
//             Point& p = *it;
//             
//         }
//     }
    for (std::pair<WallOverlapPointLink, bool> link_pair : overlap_point_links)
    {
        WallOverlapPointLink& link = link_pair.first;
        ListPolyIt a_next = link.a; ++a_next;
        ListPolyIt b_next = link.b; --b_next;
        Point& a1 = link.a.p();
        Point& a2 = a_next.p();
        Point& b1 = link.b.p();
        Point& b2 = b_next.p();
        Point a = a2-a1;
        Point b = b2-b1;
        if (point_to_link.find(a_next.p()) == point_to_link.end() 
            || point_to_link.find(b_next.p()) == point_to_link.end())
        {
            int64_t dist = overlapEndingDistance(a1, a2, b1, b2, link.dist);
            if (dist > 0)
            {
                Point a_p = a1 + a * dist / vSize(a);
                ListPolygon::iterator new_a = link.a.poly.insert(a_next.it, a_p);
                Point b_p = b1 + b * dist / vSize(b);
                ListPolygon::iterator new_b = link.a.poly.insert(link.b.it, b_p);
                addOverlapPoint(ListPolyIt(link.a.poly, new_a), ListPolyIt(link.b.poly, new_b), lineWidth);
            }
        }
        
        
        
    }
}

int64_t WallOverlapComputation::overlapEndingDistance(Point& a1, Point& a2, Point& b1, Point& b2, int a1b1_dist)
{
    int overlap = lineWidth - a1b1_dist;
    Point a = a2-a1;
    Point b = b2-b1;
    double cos_angle = INT2MM2(dot(a, b)) / vSizeMM(a) / vSizeMM(b);
    // result == .5*overlap / tan(.5*angle) == .5*overlap / tan(.5*acos(cos_angle)) 
    // [wolfram alpha] == 0.5*overlap * sqrt(cos_angle+1)/sqrt(1-cos_angle)
    // [assuming positive x] == 0.5*overlap / sqrt( 2 / (cos_angle + 1) - 1 ) 
    if (cos_angle <= 0)
    {
        return 0;
    }
    else 
    {
        int64_t dist = overlap * double ( 1.0 / (2.0 * sqrt(2.0 / (cos_angle+1.0) - 1.0)) );
        if (dist * dist > vSize2(a) || dist * dist > vSize2(b)) 
        {
            return 0;
            DEBUG_PRINTLN("ERROR! overlap end too long!! ");
        }
        return dist;
    }
    
}

    
void WallOverlapComputation::createPoint2LinkMap()
{
    for (WallOverlapPointLinks::iterator it = overlap_point_links.begin(); it != overlap_point_links.end(); ++it)
    {
        addToPoint2LinkMap(*it->first.a.it, it);
        addToPoint2LinkMap(*it->first.b.it, it);
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

    if (ratio > 1.0) { return 1.0; }
    
    return ratio;
}



void WallOverlapComputation::debugCheck()
{
    for (std::pair<WallOverlapPointLink, bool> pair : overlap_point_links)
    {
        if (std::abs(vSize( pair.first.a.p() - pair.first.b.p()) - pair.first.dist) > 10)
            DEBUG_PRINTLN(vSize( pair.first.a.p() - pair.first.b.p())<<" != " << pair.first.dist);
        
    }
}


void WallOverlapComputation::debugOutputCSV()
{
    for (PolygonRef poly : polygons)
    {
        Point last = poly.back();
        for (Point& p : poly)
        {
            std::cerr << last.X <<", "<<last.Y <<", -1" <<std::endl;
            std::cerr << p.X <<", "<<p.Y <<", -1" <<std::endl;
            std::cerr << std::endl;
            last = p;
        }
            
    }
    for (std::pair<WallOverlapPointLink, bool> pair : overlap_point_links)
    {
        std::cerr << pair.first.a.p().X <<", "<<pair.first.a.p().Y <<", "<<pair.first.dist<<std::endl;
        std::cerr << pair.first.b.p().X <<", "<<pair.first.b.p().Y <<", "<<pair.first.dist<<std::endl;
            std::cerr << std::endl;
        
    }
}

    
} // namespace cura 
