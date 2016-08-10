/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
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

    if (!overlap_linker.isLinked(from))
    { // [from] is not linked
        return 1;
    }
    const std::pair<Point2LinkIt, Point2LinkIt> to_links = overlap_linker.getLinks(to);
    if (to_links.first == to_links.second)
    { // [to] is not linked
        return 1;
    }

    int64_t overlap_area = 0;
    // note that we don't need to loop over all from_links, because they are handled in the previous getFlow(.) call (or in the very last)
    for (Point2LinkIt to_link_it = to_links.first; to_link_it != to_links.second; ++to_link_it)
    {
        const ProximityPointLink& to_link = to_link_it->second;
        ListPolyIt to_it = to_link.a;
        ListPolyIt to_other_it = to_link.b;
        if (to_link.a.p() != to)
        {
            assert(to_link.b.p() == to && "Either part of the link should be the point in the link!");
            std::swap(to_it, to_other_it);
        }
        ListPolyIt from_it = to_it.prev();
        assert(from_it.p() == from && "From location doesn't seem to be connected to destination location!");

        ListPolyIt to_other_next_it = to_other_it.next(); // move towards [from]; the lines on the other side move in the other direction
        //           to  from
        //   o<--o<--T<--F
        //   |       :   :
        //   v       :   :
        //   o-->o-->o-->o
        //           ,   ,
        //           ;   to_other_next
        //           to other

        bool are_in_same_general_direction = dot(from - to, to_other_it.p() - to_other_next_it.p()) > 0;
        // handle multiple points  linked to [to]
        //   o<<<T<<<F
        //     / |
        //    /  |
        //   o>>>o>>>o
        //   ,   ,
        //   ;   to other next
        //   to other
        if (!are_in_same_general_direction)
        {
            overlap_area += handlePotentialOverlap(to_it, to_it, to_link, to_other_it, to_other_next_it);
        }

        // handle multiple points  linked to [to_other]
        //   o<<<T<<<F
        //       |  /
        //       | /
        //   o>>>o>>>o
        bool all_are_in_same_general_direction = are_in_same_general_direction && dot(from - to, to_other_it.prev().p() - to_other_it.p()) > 0;
        if (!all_are_in_same_general_direction)
        {
            overlap_area += handlePotentialOverlap(from_it, to_it, to_link, to_other_it, to_other_it);
        }

        // handle normal case where the segment from-to overlaps with another segment
        //   o<<<T<<<F
        //       |   |
        //       |   |
        //   o>>>o>>>o
        //       ,   ,
        //       ;   to other next
        //       to other
        if (!are_in_same_general_direction)
        {
            overlap_area += handlePotentialOverlap(from_it, to_it, to_link, to_other_it, to_other_next_it);
        }
    }

    int64_t normal_area = vSize(from - to) * line_width;
    float ratio = float(normal_area - overlap_area) / normal_area;
    // clamp the ratio because overlap compensation might be faulty because
    // WallOverlapComputation::getApproxOverlapArea only gives roughly accurate results
    return std::min(1.0f, std::max(0.0f, ratio));
}

int64_t WallOverlapComputation::handlePotentialOverlap(const ListPolyIt from_it, const ListPolyIt to_it, const ProximityPointLink& to_link, const ListPolyIt to_other_it, const ListPolyIt from_other_it)
{
    const ProximityPointLink* from_link = overlap_linker.getLink(from_it, from_other_it);
    if (!from_link)
    {
        return 0;
    }
    if (!getIsPassed(to_link, *from_link))
    { // check whether the segment is already passed
        setIsPassed(to_link, *from_link);
        return 0;
    }
    // mark the segment as passed
    setIsPassed(to_link, *from_link);
    return getApproxOverlapArea(from_it.p(), to_it.p(), to_link.dist, from_other_it.p(), to_other_it.p(), from_link->dist);
}

int64_t WallOverlapComputation::getApproxOverlapArea(const Point from, const Point to, const int64_t to_dist, const Point to_other, const Point from_other, const int64_t from_dist)
{
    const Point from_middle = from_other + from; // dont divide by two just yet
    const Point to_middle = to_other + to; // dont divide by two just yet

    const int64_t link_dist_2 = vSize(from_middle - to_middle);

    const int64_t average_overlap_dist_2 = line_width * 2 - from_dist - to_dist; // dont divide by two just yet

    const int64_t area = link_dist_2 * average_overlap_dist_2 / 4; // divide by 2 two times: once for the middles and once for the average_dists
    return area;
}

bool WallOverlapComputation::getIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b)
{
    return passed_links.find(SymmetricPair<ProximityPointLink>(link_a, link_b)) != passed_links.end();
}

void WallOverlapComputation::setIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b)
{
    passed_links.emplace(link_a, link_b);
}


}//namespace cura 
