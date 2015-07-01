#include "wallOverlap.h"

#include <stdio.h> // for output to HTML

#include "debug.h"

namespace cura 
{
    
void WallOverlapComputation::findOverlapPoints()
{
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
}


    
void WallOverlapComputation::convertPolygonsToLists(Polygons& polys, ListPolygons& result)
{
    for (PolygonRef poly : polys)
    {
        result.emplace_back();
        for (Point& p : poly) 
        {
            result.back().push_back(p);
        }
    }
}    

void WallOverlapComputation::convertListPolygonsToPolygons(ListPolygons& list_polygons, Polygons& polygons)
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
            addOverlapPoint(from_it, ListPolyIt(to_list_poly, last_it), dist);
        }
        else if (closest == point)
        {
            addOverlapPoint(from_it, ListPolyIt(to_list_poly, it), dist);
        }
        else 
        {
            ListPolygon::iterator new_it = to_list_poly.insert(it, closest);
            addOverlapPoint(from_it, ListPolyIt(to_list_poly, new_it), dist);
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
//         DEBUG_PRINTLN("couldn't emplace in overlap_point_links! : ");
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
    for (std::pair<WallOverlapPointLink, bool> link_pair : overlap_point_links)
    {
        WallOverlapPointLink& link = link_pair.first;
        // an overlap segment can be an ending in two directions
        { 
            ListPolyIt a_next = link.a; ++a_next;
            ListPolyIt b_next = link.b; --b_next;
            addOverlapEnding(link, a_next, b_next, a_next, link.b);
        }
        { 
            ListPolyIt a_next = link.a; --a_next;
            ListPolyIt b_next = link.b; ++b_next;
            addOverlapEnding(link, a_next, b_next, link.a, b_next);
        }
    }
}

void WallOverlapComputation::addOverlapEnding(WallOverlapPointLink& link, ListPolyIt& a_next, ListPolyIt& b_next, const ListPolyIt& a_before_middle, const ListPolyIt& b_before_middle)
{
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
        if (dist > 10)
        {
            Point a_p = a1 + a * dist / vSize(a);
            ListPolygon::iterator new_a = link.a.poly.insert(a_before_middle.it, a_p);
            Point b_p = b1 + b * dist / vSize(b);
            ListPolygon::iterator new_b = link.a.poly.insert(b_before_middle.it, b_p);
            addOverlapPoint(ListPolyIt(link.a.poly, new_a), ListPolyIt(link.b.poly, new_b), lineWidth);
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

void WallOverlapComputation::addSharpCorners()
{
    
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
    WallOverlapPointLinks::iterator from_link = from_link_pair->second;
    if (!from_link->second)// || !to_link->second)
    {
        from_link->second = true;
//         to_link->second = true;
        return 1;
    }
    from_link->second = true;
    
    Point2Link::iterator to_link_pair = point_to_link.find(to);
    if (to_link_pair == point_to_link.end()) { return 1; }
    WallOverlapPointLinks::iterator to_link = to_link_pair->second;
//     to_link->second = true;
    
    
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


void WallOverlapComputation::wallOverlaps2HTML(const char* filename)
{
    FILE* out = fopen(filename, "w");
    fprintf(out, "<!DOCTYPE html><html><body>");
    
    int canvasSize = 1000;
    
    Point min = polygons.min() - Point(100,100);
    Point size = polygons.max() - min + Point(100,100);
    
    auto transform = [&](Point& p) { return (p-min)*canvasSize/size; };
    
    fprintf(out, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style=\"width: %dpx; height:%dpx\">\n", canvasSize, canvasSize);
    
    for(PolygonsPart part : polygons.splitIntoParts())
    {
        for (unsigned int j = 0; j < part.size(); j++)
        {
            fprintf(out, "<polygon points=\"");
            for(Point& p : part[j])
            {
                Point pf = transform(p);
                fprintf(out, "%lli,%lli ", pf.Y, pf.X);
            }
            if (j == 0)
                fprintf(out, "\" style=\"fill:gray; stroke:black;stroke-width:1\" />\n");
            else
                fprintf(out, "\" style=\"fill:white; stroke:black;stroke-width:1\" />\n");
        }
    }
    
    for (PolygonRef poly : polygons)
        for (Point& p : poly)
        {
            Point pf = transform(p);
            fprintf(out, "<circle cx=\"%lli\" cy=\"%lli\" r=\"%d\" stroke=\"black\" stroke-width=\"3\" fill=\"black\" />",pf.Y, pf.X, 3);
        }
    
    for (std::pair<WallOverlapPointLink , bool> pair : overlap_point_links)
    {
        WallOverlapPointLink& link = pair.first;
        Point a = transform(link.a.p());
        Point b = transform(link.b.p());
        fprintf(out, "<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" style=\"stroke:rgb(%d,%d,0);stroke-width:2\" />", a.Y, a.X, b.Y, b.X, pair.second? 0 : 255, pair.second? 255 : 0);
    }
    
    fprintf(out, "</svg>\n");
    
    
    fprintf(out, "</body></html>");
    fclose(out);
}
    
} // namespace cura 
