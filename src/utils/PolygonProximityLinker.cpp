/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include <cmath> // isfinite
#include <sstream> // ostream

#include "PolygonProximityLinker.h"
#include "linearAlg2D.h"

#include "AABB.h" // for debug output svg html
#include "SVG.h"

namespace cura 
{

PolygonProximityLinker::PolygonProximityLinker(Polygons& polygons, int proximity_distance)
 : polygons(polygons)
 , proximity_distance(proximity_distance)
 , proximity_distance_2(proximity_distance * proximity_distance)
 , line_grid(proximity_distance, polygons.pointCount(), 3.0f)
{
    // heuristic reserve a good amount of elements
    proximity_point_links.reserve(polygons.pointCount()); // When the whole model consists of thin walls, there will generally be a link for every point, plus some endings minus some points which map to eachother

    // convert to list polygons for insertion of points
    ListPolyIt::convertPolygonsToLists(polygons, list_polygons); 

    // link each corner to itself
    addSharpCorners();

    // map each vertex onto nearby line segments
    findProximatePoints();

    // add links where line segments diverge from below the proximity distance to over the proximity distance
    addProximityEndings();

    // convert list polygons back
    ListPolyIt::convertListPolygonsToPolygons(list_polygons, polygons);
//     proximity2HTML("linker.html");
}

bool PolygonProximityLinker::isLinked(Point from)
{
    return point_to_link.count(from) > 0;
}


std::pair<PolygonProximityLinker::Point2Link::iterator, PolygonProximityLinker::Point2Link::iterator> PolygonProximityLinker::getLinks(Point from)
{
    std::pair<Point2Link::iterator, Point2Link::iterator> from_link_pair = point_to_link.equal_range(from);
#ifdef DEBUG
    for (Point2Link::iterator it = from_link_pair.first; it != from_link_pair.second; ++it)
    {
        if (!(it->second.a.p() == from || it->second.b.p() == from))
        {
            std::cerr << " ERROR!\n" << it->first << " == " << from << "\n";
            std::cerr << "should be either " << it->second.a.p() << " or " << it->second.b.p() << "\n";
            std::cerr << (from_link_pair.first == from_link_pair.second) << " ; " << (from_link_pair.first == point_to_link.end()) << " ; " << (from_link_pair.second == point_to_link.end()) << "\n";
            std::cerr << std::hash<Point>()(from) << " hashes " << std::hash<Point>()(it->second.a.p()) << " or " << std::hash<Point>()(it->second.b.p()) << "\n";
//             std::cerr << "ERROR! some point got mapped to a link which doesn't have the point as one of the end points!\n";
            
            std::cerr << "\n all links:\n";
            for (std::pair<const Point, const ProximityPointLink> pair : point_to_link)
            {
                std::cerr << pair.first << " : " << pair.second.a.p() << "-" <<pair.second.b.p() << "\n";
            }
            
            std::cerr << "\n link set \n";
            for (const ProximityPointLink link : proximity_point_links)
            {
                std::cerr << link.a.p() << "-" << link.b.p() << " hashes as " << std::hash<ProximityPointLink>()(link) << "\n";
            }
            assert(false && "some point got mapped to a link which doesn't have the point as one of the end points!");
        }
    }
#endif
    return from_link_pair;
}

void PolygonProximityLinker::createLineGrid()
{
    for (unsigned int poly_idx = 0; poly_idx < list_polygons.size(); poly_idx++)
    {
        ListPolygon& poly = list_polygons[poly_idx];
        for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
        {
            ListPolyIt point_it(poly, it);
            line_grid.insert(point_it);
        }
    }
}

void PolygonProximityLinker::findProximatePoints()
{
    createLineGrid();

    for (unsigned int poly_idx = 0; poly_idx < list_polygons.size(); poly_idx++)
    {
        ListPolygon& poly = list_polygons[poly_idx];
        for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
        {
            ListPolyIt point_it(poly, it);
            if (new_points.count(point_it) == 0)
            {
                // handle new_points separately
                // to prevent this:
                //  1    3   5   7
                //  o<-.
                //  :   'o<-.
                //  :   /:   o<.
                //  :  / :  /:  'o<.
                //  : /  : / : / :
                //  :/   :/  :/  :   etc.
                //  o--->o-->o-->o->
                //  2    4   6   8
                std::unordered_set<ListPolyIt> nearby_lines;
                auto process_func = [&nearby_lines](const ListPolyIt& elem)
                {
                    nearby_lines.emplace(elem);
                };
                line_grid.processNearby(point_it.p(), proximity_distance, process_func);
                for (const ListPolyIt& nearby_line : nearby_lines)
                {
                    findProximatePoints(point_it, *nearby_line.poly, nearby_line, nearby_line.next());
                }
            }
        }
    }

    for (const ListPolyIt& new_point_it : new_points)
    {
        // link with existing points, but don't introduce new points for line segments
        // to prevent this:
        //  1    3   5   7
        //  o<-.
        //  :   'o<-.
        //  :   /:   o<.
        //  :  / :  /:  'o<.
        //  : /  : / : / :
        //  :/   :/  :/  :   etc.
        //  o--->o-->o-->o->
        //  2    4   6   8
        std::unordered_set<ListPolyIt> nearby_vert_its;
        auto process_func = [&nearby_vert_its](const ListPolyIt& elem)
        {
            nearby_vert_its.emplace(elem);
        };
        line_grid.processNearby(new_point_it.p(), proximity_distance, process_func);
        // because we use the same line_grid as before the resulting nearby_points
        // will also have points which are not nearby, (But when the line segment *is* nearby.)
        // but at least we don't have to create a whole new SparsePointGrid
        for (const ListPolyIt& nearby_vert_it : nearby_vert_its)
        {
            Point new_point = new_point_it.p();
            Point nearby_vert = nearby_vert_it.p();
            int64_t dist2 = vSize2(new_point - nearby_vert);
            if (dist2 < proximity_distance_2
                && new_point != nearby_vert // not the same point
                && new_point_it.next() != nearby_vert_it && new_point_it.prev() != nearby_vert_it // not directly connected
            )
            {
                addProximityLink(new_point_it, nearby_vert_it, sqrt(dist2), ProximityPointLinkType::NORMAL);
            }
        }
    }
}

void PolygonProximityLinker::findProximatePoints(const ListPolyIt a_from_it, ListPolygon& to_list_poly, const ListPolyIt b_from_it, const ListPolyIt b_to_it)
{
    const Point& a_from = a_from_it.p();

    const Point& b_from = b_from_it.p();
    const Point& b_to = b_to_it.p();

    if (a_from_it == b_from_it || a_from_it == b_to_it) // we currently consider a linesegment directly connected to [from]
    {
        return;
    }
    if (a_from_it.prev() == b_to_it) // [a] is connected to a line segment directly connected to the line segment [b]
    {
        // only check whether we need to link points; don't project
        int64_t dist2 = vSize2(b_from - a_from);
        if (dist2 < proximity_distance_2)
        {
            int64_t dist = sqrt(dist2);
            addProximityLink(a_from_it, b_from_it, dist, ProximityPointLinkType::NORMAL);
        }
        return;
    }
    if (a_from_it.next() == b_from_it) // [a] is connected to a line segment directly connected to the line segment [b]
    {
        // only check whether we need to link points; don't project
        int64_t dist2 = vSize2(b_to - a_from);
        if (dist2 < proximity_distance_2)
        {
            int64_t dist = sqrt(dist2);
            addProximityLink(a_from_it, b_to_it, dist, ProximityPointLinkType::NORMAL);
        }
        return;
    }



    Point closest = LinearAlg2D::getClosestOnLineSegment(a_from, b_from, b_to);

    int64_t dist2 = vSize2(closest - a_from);

    if (dist2 > proximity_distance_2
        || (a_from_it.poly == &to_list_poly
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
        if (new_points.count(a_from_it) == 0)
        {
            // don't introduce new points for newly introduced points
            // to prevent this:
            //  1    3   5   7
            //  o<-.
            //  :   'o<-.
            //  :   /:   o<.
            //  :  / :  /:  'o<.
            //  : /  : / : / :
            //  :/   :/  :/  :   etc.
            //  o--->o-->o-->o->
            //  2    4   6   8
            ListPolyIt new_it = addNewPolyPoint(closest, b_from_it, b_to_it, b_to_it);
            addProximityLink(a_from_it, new_it, dist, ProximityPointLinkType::NORMAL);
        }
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
    assert(it->a == from);
    assert(it->b == to);
    assert(it->a.p() == from.p());
    assert(it->b.p() == to.p());
    addToPoint2LinkMap(from.p(), it);
    addToPoint2LinkMap(to.p(), it);

    return result.second;
}

bool PolygonProximityLinker::addCornerLink(ListPolyIt corner_point, const ProximityPointLinkType type)
{
    constexpr int dist = 0;
    std::pair<ProximityPointLinks::iterator, bool> result =
        proximity_point_links.emplace(corner_point, corner_point, dist, type);

    if (!result.second)
    { // links was already made!
        return false;
    }
    ProximityPointLinks::iterator it = result.first;
    assert(it->a == corner_point);
    assert(it->b == corner_point);
    addToPoint2LinkMap(corner_point.p(), it);

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
        addCornerLink(a2_it, ProximityPointLinkType::ENDING_CORNER);
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
            ListPolyIt new_b = addNewPolyPoint(b_p, link.b, b2_it, b_after_middle);
            addProximityLink(a2_it, new_b, proximity_distance, ProximityPointLinkType::ENDING);
        }
        else if (b_length2 < a_length2)
        {
            Point a_p = a1 + normal(a, dist);
            ListPolyIt new_a = addNewPolyPoint(a_p, link.a, a2_it, a_after_middle);
            addProximityLink(new_a, b2_it, proximity_distance, ProximityPointLinkType::ENDING);
        }
        else // equal
        {
            addProximityLink(a2_it, b2_it, proximity_distance, ProximityPointLinkType::ENDING);
        }
    }
    else if (dist > 0)
    {
        Point a_p = a1 + normal(a, dist);
        ListPolyIt new_a = addNewPolyPoint(a_p, link.a, a2_it, a_after_middle);
        Point b_p = b1 + normal(b, dist);
        ListPolyIt new_b = addNewPolyPoint(b_p, link.b, b2_it, b_after_middle);
        addProximityLink(new_a, new_b, proximity_distance, ProximityPointLinkType::ENDING);
    }
    else if (dist == 0)
    {
//         addProximityLink(link.a, link.b, proximity_distance);
        // won't be inserted any way, because there already is such a link!
    }
}

ListPolyIt PolygonProximityLinker::addNewPolyPoint(const Point point, const ListPolyIt line_start, const ListPolyIt line_end, const ListPolyIt before)
{
    if (point == line_start.p())
    {
        return line_start;
    }
    if (point == line_end.p())
    {
        return line_end;
    }
    ListPolygon::iterator new_p = before.poly->insert(before.it, point);
    ListPolyIt lpi(*before.poly, new_p);
    line_grid.insert(lpi);
    // TODO: remove new part of old segment from the line_grid (?)
    new_points.emplace(lpi);
    return lpi;
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
    for (ListPolygon& poly : list_polygons)
    {
        ListPolyIt here(poly, --poly.end());
        ListPolyIt prev = here.prev();
        for (ListPolygon::iterator it = poly.begin(); it != poly.end(); ++it)
        {
            ListPolyIt next(poly, it);

            if (LinearAlg2D::isAcuteCorner(prev.p(), here.p(), next.p()) > 0)
            {
                addCornerLink(here, ProximityPointLinkType::SHARP_CORNER);
            }

            prev = here;
            here = next;
        }
    }
}

void PolygonProximityLinker::addToPoint2LinkMap(Point p, ProximityPointLinks::iterator it)
{
    const ProximityPointLink& link = *it;
    point_to_link.emplace(p, link); // copy element from proximity_point_links set to Point2Link map
    assert(p == link.a.p() || p == link.b.p());
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
