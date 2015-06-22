#include "wallOverlap.h"


namespace cura 
{
    
void WallOverlapComputation::findOverlapPoints()
{
    for (PolygonRef poly : polygons)
    {
        for (Point& p : poly)
        {
            findOverlapPoints(p);
        }
    }
}


    
void WallOverlapComputation::findOverlapPoints(Point from)
{
    for (PolygonRef poly : polygons)
    {
        findOverlapPoints(from, poly);
    }
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
    Point last_point = to_list_poly.back();
    for (ListPolygon::iterator it = to_list_poly.begin(); it != to_list_poly.end(); ++it)
    {
        Point& point = *it;
        if (from == last_point || from == point )
        { // we currently consider a linesegment directly connected to [from]
            last_point = point;
            continue;
        }
        Point closest = getClosestOnLine(from, last_point, point);
        
        int dist = vSize2(closest - from);
        
        if (dist > lineWidth * lineWidth)
        { // line segment too far away to have overlap
            last_point = point;
            continue;
        }
        
        if (closest == last_point)
        {
            overlap_point_links.emplace(from, last_point, dist);
        }
        else if (closest == point)
        {
            overlap_point_links.emplace(from, point, dist);
        }
        else 
        {
            to_list_poly.insert(it, closest);
            overlap_point_links.emplace(from, closest, dist);
        }
        
        last_point = point;
    }
}
    
void WallOverlapComputation::createPoint2LinkMap()
{
    for (WallOverlapPointLinks::iterator it = overlap_point_links.begin(); it != overlap_point_links.end(); ++it)
    {
        addToPoint2LinkMap(it->a, it);
        addToPoint2LinkMap(it->b, it);
    }
}
    
void WallOverlapComputation::addToPoint2LinkMap(Point p, WallOverlapPointLinks::iterator it)
{
    Point2Link::iterator found = point_to_link.find(p);
    if (found == point_to_link.end())
    {
        std::vector<WallOverlapPointLinks::iterator> links;
        links.push_back(it);
        point_to_link.emplace(p, links);
    }
    else 
    {
        found->second.push_back(it);
    }
}

float WallOverlapComputation::getFlow(Point& from, Point& to)
{
    Point2Link::iterator from_link = point_to_link.find(from);
    if (from_link == point_to_link.end()) { return 1; }
    Point2Link::iterator to_link = point_to_link.find(to);
    if (to_link == point_to_link.end()) { return 1; }
    
    //if (from_link->
}

    
} // namespace cura 
