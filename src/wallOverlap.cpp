#include "wallOverlap.h"

#include <cmath> // isfinite
#include <sstream>

#include "utils/AABB.h" // for debug output svg html
#include "utils/SVG.h"

namespace cura 
{

WallOverlapComputation::WallOverlapComputation(Polygons& polygons, int line_width)
: overlap_linker(polygons, line_width)
, line_width(line_width)
{ 

}


float WallOverlapComputation::getFlow(Point& from, Point& to)
{
    using Point2LinkIt = PolygonProximityLinker::Point2Link::iterator;

    const std::pair<Point2LinkIt, Point2LinkIt> from_links = overlap_linker.getLinks(from);
    if (from_links.first == from_links.second)
    {
        return 1;
    }
    const std::pair<Point2LinkIt, Point2LinkIt> to_links = overlap_linker.getLinks(to);
    if (to_links.first == to_links.second)
    {
        return 1;
    }

    int64_t overlap_area = 0;
    for (Point2LinkIt it = to_links.first; it != to_links.second; ++it)
    {
        const PolygonProximityLinker::ProximityPointLink& to_link = it->second;
        ListPolyIt to_it = to_link.a;
        ListPolyIt to_other_it = to_link.b;
        if (to_link.a.p() != to)
        {
            assert(to_link.b.p() == to && "Either part of the link should be the point in the link!");
            std::swap(to_it, to_other_it);
        }
        ListPolyIt from_it = to_it.prev();
        assert(from_it.p() == from && "From location doesn't seem to be connected to destination location!");

        // handle multiple points  linked to [to]
        ListPolyIt to_other_next_it = to_other_it.next(); // move towards [from]
        std::optional<PolygonProximityLinker::ProximityPointLink> other_next_2_same_to_link = overlap_linker.getLink(to_other_next_it, to_it);
        if (other_next_2_same_to_link)
        { // segment overlaps with to vertex
            if (!to_link.passed || !other_next_2_same_to_link->passed)
            { // check whether the segment is already passed
                to_link.passed = true;
                other_next_2_same_to_link->passed = true;
                continue;
            }
            // mark the segment as passed
            to_link.passed = true;
            other_next_2_same_to_link->passed = true;
            overlap_area += getApproxOverlapArea(to_link, *other_next_2_same_to_link);
        }

        // handle multiple points  linked to [to_other]
        std::optional<PolygonProximityLinker::ProximityPointLink> other_2_same_from_link = overlap_linker.getLink(to_other_it, from_it);
        if (other_2_same_from_link)
        {
            if (!to_link.passed || !other_2_same_from_link->passed)
            { // check whether the segment is already passed
                to_link.passed = true;
                other_2_same_from_link->passed = true;
                continue;
            }
            // mark the segment as passed
            to_link.passed = true;
            other_2_same_from_link->passed = true;
            overlap_area += getApproxOverlapArea(to_link, *other_2_same_from_link);
        }

        // handle normal case where the segment from-to overlaps with another segment
        std::optional<PolygonProximityLinker::ProximityPointLink> prev_proper_link = overlap_linker.getLink(to_other_next_it, from_it);
        if (prev_proper_link)
        {
            if (!to_link.passed || !prev_proper_link->passed)
            { // check whether the segment is already passed
                to_link.passed = true;
                prev_proper_link->passed = true;
                continue;
            }
            // mark the segment as passed
            to_link.passed = true;
            prev_proper_link->passed = true;
            overlap_area += getApproxOverlapArea(to_link, *other_next_2_same_to_link);
        }
    }

    int64_t normal_area = vSize(from - to) * line_width;
    float ratio = float(normal_area - overlap_area) / normal_area;
    if (ratio > 1.0)
    {
        return 1.0;
    }
    return ratio;
}

int64_t WallOverlapComputation::getApproxOverlapArea(const PolygonProximityLinker::ProximityPointLink& from, const PolygonProximityLinker::ProximityPointLink& to)
{
    return getApproxOverlapArea(from.a.p(), from.b.p(), from.dist, to.a.p(), to.b.p(), to.dist);
}


int64_t WallOverlapComputation::getApproxOverlapArea(const Point from_a, const Point from_b, const int64_t from_dist, const Point to_a, const Point to_b, const int64_t to_dist)
{
    const Point from_middle = from_a + from_b; // dont divide by two just yet
    const Point to_middle = to_a + to_b; // dont divide by two just yet

    const int64_t dist_2 = vSize(from_middle - to_middle);

    const int64_t average_dists_2 = line_width * 2 - from_dist - to_dist; // dont divide by two just yet

    const int64_t area = dist_2 * average_dists_2 / 4; // divide by 2 two times: once for the middles and once for the average_dists
    return area;
}


}//namespace cura 
