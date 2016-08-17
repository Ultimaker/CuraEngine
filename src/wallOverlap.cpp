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

        if (from_it.p() != from)
        {
            logWarning("Polygon has multiple verts at the same place: (%lli, %lli); PolygonProximityLinker fails in such a case!\n", from.X, from.Y);
        }

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
            overlap_area += handlePotentialOverlap(to_it, to_it, to_link, to_other_next_it, to_other_it);
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
            overlap_area += handlePotentialOverlap(from_it, to_it, to_link, to_other_next_it, to_other_it);
        }
    }

    int64_t normal_area = vSize(from - to) * line_width;
    float ratio = float(normal_area - overlap_area) / normal_area;
    // clamp the ratio because overlap compensation might be faulty because
    // WallOverlapComputation::getApproxOverlapArea only gives roughly accurate results
    return std::min(1.0f, std::max(0.0f, ratio));
}

int64_t WallOverlapComputation::handlePotentialOverlap(const ListPolyIt from_it, const ListPolyIt to_it, const ProximityPointLink& to_link, const ListPolyIt from_other_it, const ListPolyIt to_other_it)
{
    if (from_it == to_other_it && from_it == from_other_it)
    { // don't compute overlap with a line and itself
        return 0;
    }
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
    return getApproxOverlapArea(from_it.p(), to_it.p(), to_link.dist, from_other_it.p(), to_other_it.p(), from_link->dist);
}

int64_t WallOverlapComputation::getApproxOverlapArea(const Point from, const Point to, const int64_t to_dist, const Point from_other, const Point to_other, const int64_t from_dist)
{
    const Point& other_from = to_other; // the one opposite to [to] is the starting point of the other line segment
    const Point& other_to = from_other; // these two lines are only renames to interpret the code below easier

    std::optional<int64_t> link_dist_2_override; // (an approximation of) twice the length of the overlap area

    // check whether the line segment overlaps with the point if one of the line segments is just a point
    if (from == to)
    {
        if (LinearAlg2D::pointIsProjectedBeyondLine(from, other_from, other_to) != 0)
        {
            return 0;
        }
    }
    else if (other_from == other_to)
    {
        if (LinearAlg2D::pointIsProjectedBeyondLine(other_from, from, to) != 0)
        {
            return 0;
        }
    }
    else
    {
        short from_rel = LinearAlg2D::pointIsProjectedBeyondLine(from, other_from, other_to);
        short to_rel = LinearAlg2D::pointIsProjectedBeyondLine(to, other_from, other_to);
        short other_from_rel = LinearAlg2D::pointIsProjectedBeyondLine(other_from, from, to);
        short other_to_rel = LinearAlg2D::pointIsProjectedBeyondLine(other_to, from, to);
        if (from_rel != 0 && from_rel == to_rel && to_rel == other_from_rel && other_from_rel == other_to_rel)
        {
            // both segments project fully beyond or before each other
            // for example:
            // O<------O   .
            //         :   :
            //         '   O------->O
            return 0;
        }
        if ( to_rel != 0 && to_rel == other_to_rel && from_rel == 0 && other_from_rel == 0 )
        {
            // only beginnings of line segments overlap
            //
            //           from_proj
            //           ^^^^^
            //      O<---+---O
            //           :   :
            //           O---+---->O
            //           ,,,,,
            // other_from_proj
            const Point other_vec = other_to - other_from;
            int64_t from_proj = dot(from - other_from, other_vec) / vSize(other_vec);

            const Point vec = to - from;
            int64_t other_from_proj = dot(other_from - from, vec) / vSize(vec);

            link_dist_2_override = from_proj + other_from_proj;
        }
        if ( from_rel != 0 && from_rel == other_from_rel && to_rel == 0 && other_to_rel == 0 )
        {
            // only ends of line segments overlap
            //
            //       to_proj
            //         ^^^^^
            //         O<--+----O
            //         :   :
            //   O-----+-->O
            //         ,,,,,
            //         other_to_proj
            const Point other_vec = other_from - other_to;
            int64_t to_proj = dot(to - other_to, other_vec) / vSize(other_vec);

            const Point vec = from - to;
            int64_t other_to_proj = dot(other_to - to, vec) / vSize(vec);

            link_dist_2_override = to_proj + other_to_proj;
        }
    }

    const Point from_middle = from_other + from; // dont divide by two just yet
    const Point to_middle = to_other + to; // dont divide by two just yet

    const int64_t link_dist_2 = (link_dist_2_override)? *link_dist_2_override : vSize(from_middle - to_middle); // (an approximation of) twice the length of the overlap area

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
