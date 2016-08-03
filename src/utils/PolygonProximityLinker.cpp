/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include <cmath> // isfinite
#include <sstream> // ostream

#include "PolygonProximityLinker.h"

#include "AABB.h" // for debug output svg html
#include "SVG.h"

namespace cura 
{

PolygonProximityLinker::PolygonProximityLinker(Polygons& polygons, int proximity_distance)
 : polygons(polygons)
 , proximity_distance(proximity_distance) 
{ 
    unsigned int n_points = 0;
    for (PolygonRef poly : polygons)
    {
        n_points += poly.size();
    }

    // reserve enough elements so that iterators don't get invalidated
    proximity_point_links.reserve(n_points * 2); // generally enough, unless there are a lot of 3-way intersections in the model

    // convert to list polygons for insertion of points
    ListPolyIt::convertPolygonsToLists(polygons, list_polygons); 

    findProximatePoints();
    addProximityEndings();
    // TODO: add sharp corners

    // convert list polygons back
    ListPolyIt::convertListPolygonsToPolygons(list_polygons, polygons);
//     wallOverlaps2HTML("output/output.html");
}

bool PolygonProximityLinker::isLinked(Point from)
{
    return point_to_link.count(from) > 0;
}


std::pair<PolygonProximityLinker::Point2Link::iterator, PolygonProximityLinker::Point2Link::iterator> PolygonProximityLinker::getLinks(Point from)
{
    std::pair<Point2Link::iterator, Point2Link::iterator> from_link_pair = point_to_link.equal_range(from);
    return from_link_pair;
}


void PolygonProximityLinker::findProximatePoints()
{
    // link each vertex of each polygon to each proximate line segment of any polygon
    for (unsigned int poly_idx = 0; poly_idx < list_polygons.size(); poly_idx++)
    {
        ListPolygon& poly = list_polygons[poly_idx];
        for (unsigned int poly2_idx = 0; poly2_idx < list_polygons.size(); poly2_idx++)
        {
            for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
            {
                ListPolyIt lpi(poly, it);
                findProximatePoints(lpi, poly2_idx);
            }
        }
    }
}

void PolygonProximityLinker::findProximatePoints(ListPolyIt from_it, unsigned int to_list_poly_idx)
{
    ListPolygon& to_list_poly = list_polygons[to_list_poly_idx];
    ListPolyIt from_lpi(to_list_poly, --to_list_poly.end());
    for (ListPolygon::iterator it = to_list_poly.begin(); it != to_list_poly.end(); ++it)
    {
        ListPolyIt to_lpi(to_list_poly, it);
        findProximatePoints(from_it, to_list_poly, from_lpi, to_lpi);

        from_lpi = to_lpi;
    }
}

void PolygonProximityLinker::findProximatePoints(const ListPolyIt a_from_it, ListPolygon& to_list_poly, const ListPolyIt b_from_it, const ListPolyIt b_to_it)
{
    const Point& a_from = a_from_it.p();

    const Point& b_from = b_from_it.p();
    const Point& b_to = b_to_it.p();

    if (a_from_it == b_from_it || a_from_it == b_to_it // we currently consider a linesegment directly connected to [from]
        || a_from_it.prev() == b_to_it || a_from_it.next() == b_from_it) // line segment from [last_point] to [point] is connected to line segment of which [from] is the other end
    {
        return;
    }
    Point closest = LinearAlg2D::getClosestOnLineSegment(a_from, b_from, b_to);

    int64_t dist2 = vSize2(closest - a_from);

    if (dist2 > proximity_distance * proximity_distance
        || (&a_from_it.poly == &to_list_poly 
            && dot(a_from_it.next().p() - a_from, b_to - b_from) > 0 
            && dot(a_from - a_from_it.prev().p(), b_to - b_from) > 0  ) // line segments are likely connected, because the winding order is in the same general direction
    )
    { // line segment too far away to be proximate
        return;
    }

    int64_t dist = sqrt(dist2);

    if (shorterThen(closest - b_from, 10))
    {
        addProximityLink(a_from_it, b_from_it, dist, ProximityPointLinkType::NORMAL);
    }
    else if (shorterThen(closest - b_to, 10))
    {
        addProximityLink(a_from_it, b_to_it, dist, ProximityPointLinkType::NORMAL);
    }
    else 
    {
        ListPolygon::iterator new_it = to_list_poly.insert(b_to_it.it, closest);
        addProximityLink(a_from_it, ListPolyIt(to_list_poly, new_it), dist, ProximityPointLinkType::NORMAL);
    }
}



bool PolygonProximityLinker::addProximityLink(ListPolyIt from, ListPolyIt to, int64_t dist, const ProximityPointLinkType type)
{
    std::pair<ProximityPointLinks::iterator, bool> result =
        proximity_point_links.emplace(from, to, dist, type);

    if (!result.second)
    { // links was already made!
        return false;
    }
    ProximityPointLinks::iterator it = result.first;
    addToPoint2LinkMap(*it->a.it, it);
    addToPoint2LinkMap(*it->b.it, it);

    return result.second;
}

void PolygonProximityLinker::addProximityEndings()
{
    for (const ProximityPointLink& link : proximity_point_links)
    {

        if (link.dist == proximity_distance)
        { // its ending itself
            continue;
        }
        const ListPolyIt& a_1 = link.a;
        const ListPolyIt& b_1 = link.b;
        // an overlap segment can be an ending in two directions
        { 
            ListPolyIt a_2 = a_1.next();
            ListPolyIt b_2 = b_1.prev();
            addProximityEnding(link, a_2, b_2, a_2, b_1);
        }
        { 
            ListPolyIt a_2 = a_1.prev();
            ListPolyIt b_2 = b_1.next();
            addProximityEnding(link, a_2, b_2, a_1, b_2);
        }
    }
}

void PolygonProximityLinker::addProximityEnding(const ProximityPointLink& link, const ListPolyIt& a2_it, const ListPolyIt& b2_it, const ListPolyIt& a_after_middle, const ListPolyIt& b_after_middle)
{
    Point& a1 = link.a.p();
    Point& a2 = a2_it.p();
    Point& b1 = link.b.p();
    Point& b2 = b2_it.p();
    Point a = a2-a1;
    Point b = b2-b1;

    if (isLinked(a2_it.p()) && isLinked(b2_it.p())) // overlap area stops at one side
    {
        return;
    }
    if (isLinked(a2_it, link.b) || isLinked(b2_it, link.a))
    { // other side of ending continues to overlap with the same ending
        //     link considered
        //     *
        // o<--o<--o<--
        // :   : .`  one more link from the upper side
        // o-->o  last overlap point on this side
        //     |
        //     v
        //     0
        return;
    }
    if (a2_it == b2_it)
    { // overlap ends in pointy end
        //  o-->o-->o
        //  :   :   : \,
        //  :   :   :  o  wasn't linked yet because it's connected to the upper and lower part
        //  :   :   :,/
        //  o<--o<--o
        int64_t dist = 0;
        addProximityLink(a2_it, a2_it, dist, ProximityPointLinkType::ENDING_CORNER);
        return;
    }

    int64_t dist = proximityEndingDistance(a1, a2, b1, b2, link.dist);
    if (dist < 0) { return; }
    int64_t a_length2 = vSize2(a);
    int64_t b_length2 = vSize2(b);
    if (dist*dist > std::min(a_length2, b_length2) )
    { // TODO remove this /\ case if error below is never shown
//         DEBUG_PRINTLN("Next point should have been linked already!!");
        dist = std::sqrt(std::min(a_length2, b_length2));
        if (a_length2 < b_length2)
        {
            Point b_p = b1 + normal(b, dist);
            ListPolygon::iterator new_b = link.b.poly.insert(b_after_middle.it, b_p);
            addProximityLink(a2_it, ListPolyIt(link.b.poly, new_b), proximity_distance, ProximityPointLinkType::ENDING);
        }
        else if (b_length2 < a_length2)
        {
            Point a_p = a1 + normal(a, dist);
            ListPolygon::iterator new_a = link.a.poly.insert(a_after_middle.it, a_p);
            addProximityLink(ListPolyIt(link.a.poly, new_a), b2_it, proximity_distance, ProximityPointLinkType::ENDING);
        }
        else // equal
        {
            addProximityLink(a2_it, b2_it, proximity_distance, ProximityPointLinkType::ENDING);
        }
    }
    if (dist > 0)
    {
        Point a_p = a1 + normal(a, dist);
        ListPolygon::iterator new_a = link.a.poly.insert(a_after_middle.it, a_p);
        Point b_p = b1 + normal(b, dist);
        ListPolygon::iterator new_b = link.b.poly.insert(b_after_middle.it, b_p);
        addProximityLink(ListPolyIt(link.a.poly, new_a), ListPolyIt(link.b.poly, new_b), proximity_distance, ProximityPointLinkType::ENDING);
    }
    else if (dist == 0)
    {
//         addProximityLink(link.a, link.b, proximity_distance);
        // won't be inserted any way, because there already is such a link!
    }
}

int64_t PolygonProximityLinker::proximityEndingDistance(Point& a1, Point& a2, Point& b1, Point& b2, int a1b1_dist)
{
    int overlap = proximity_distance - a1b1_dist;
    Point a = a2-a1;
    Point b = b2-b1;
    double cos_angle = INT2MM2(dot(a, b)) / vSizeMM(a) / vSizeMM(b);
    // result == .5*overlap / tan(.5*angle) == .5*overlap / tan(.5*acos(cos_angle)) 
    // [wolfram alpha] == 0.5*overlap * sqrt(cos_angle+1)/sqrt(1-cos_angle)
    // [assuming positive x] == 0.5*overlap / sqrt( 2 / (cos_angle + 1) - 1 ) 
    if (cos_angle <= 0
        || ! std::isfinite(cos_angle) )
    {
        return -1; // line_width / 2;
    }
    else if (cos_angle > .9999) // values near 1 can lead too large numbers  for 1/x
    {
        return std::min(vSize(b), vSize(a));
    }
    else
    {
        int64_t dist = overlap * double ( 1.0 / (2.0 * sqrt(2.0 / (cos_angle+1.0) - 1.0)) );
        return dist;
    }
}

void PolygonProximityLinker::addSharpCorners()
{
    
}

void PolygonProximityLinker::addToPoint2LinkMap(Point p, ProximityPointLinks::iterator it)
{
    point_to_link.emplace(p, *it); // copy element from proximity_point_links set to Point2Link map
    // TODO: what to do if the map already contained a link? > three-way proximity
}


void PolygonProximityLinker::proximity2HTML(const char* filename) const
{
    PolygonProximityLinker copy = *this; // copy, cause getFlow might change the state of the overlap computation!

    AABB aabb(copy.polygons);

    aabb.expand(200);

    SVG svg(filename, aabb, Point(1024 * 2, 1024 * 2));


    svg.writeAreas(copy.polygons);

    { // output points and coords
        for (ListPolygon poly : copy.list_polygons)
        {
            for (Point& p : poly)
            {
                svg.writePoint(p, true);
            }
        }
    }

    { // output links
        // output normal links
        for (const ProximityPointLink& link : copy.proximity_point_links)
        {
            svg.writePoint(link.a.p(), false, 3, SVG::Color::GRAY);
            svg.writePoint(link.b.p(), false, 3, SVG::Color::GRAY);
            Point a = svg.transform(link.a.p());
            Point b = svg.transform(link.b.p());
            svg.printf("<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" style=\"stroke:rgb(%d,%d,0);stroke-width:1\" />", a.X, a.Y, b.X, b.Y, link.dist == proximity_distance? 0 : 255, link.dist==proximity_distance? 255 : 0);
        }
    }
}

bool PolygonProximityLinker::isLinked(ListPolyIt a, ListPolyIt b)
{
    ProximityPointLink test_link(a, b, 0, ProximityPointLinkType::NORMAL);
    return proximity_point_links.count(test_link) > 0;
}

const ProximityPointLink* PolygonProximityLinker::getLink(ListPolyIt a, ListPolyIt b)
{
    ProximityPointLink test_link(a, b, 0, ProximityPointLinkType::NORMAL);
    ProximityPointLinks::const_iterator found = proximity_point_links.find(test_link);

    if (found != proximity_point_links.end())
    {
        return &*found;
    }
    return nullptr;
}

}//namespace cura 
