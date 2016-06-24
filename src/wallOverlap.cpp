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
    const PolygonProximityLinker::ProximityPointLink* from_link = overlap_linker.getLink(from);
    if (!from_link)
    {
        return 1;
    }
    const PolygonProximityLinker::ProximityPointLink* to_link = overlap_linker.getLink(to);
    if (!to_link)
    {
        return 1;
    }

    if (!from_link->passed || !to_link->passed)
    {
        from_link->passed = true;
        to_link->passed = true;
        return 1;
    }
    from_link->passed = true;
    to_link->passed = true;

    // both points have already been passed

    float avg_link_dist = 0.5 * ( INT2MM(from_link->dist) + INT2MM(to_link->dist) );

    float ratio = avg_link_dist / INT2MM(line_width);

    if (ratio > 1.0) { return 1.0; }
    
    return ratio;
}


}//namespace cura 
