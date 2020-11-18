//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolylineStitcher.h"
#include "SparsePointGrid.h"
#include "PolygonsPointIndex.h"

#include "SymmetricPair.h"
#include <unordered_set>
#include <cassert>

namespace cura
{


void PolylineStitcher::stitch(const Polygons& lines, Polygons& result_lines, Polygons& result_polygons, coord_t max_stitch_distance, coord_t snap_distance)
{
    if (lines.empty())
    {
        return;
    }

    SparsePointGrid<PolygonsPointIndex, PolygonsPointIndexLocator> grid(max_stitch_distance, lines.size() * 2);
    
    // populate grid
    for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
    {
        ConstPolygonRef line = lines[line_idx];
        grid.insert(PolygonsPointIndex(&lines, line_idx, 0));
        grid.insert(PolygonsPointIndex(&lines, line_idx, line.size() - 1));
    }
    
    std::vector<bool> processed(lines.size(), false);
    
    for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
    {
        if (processed[line_idx])
        {
            continue;
        }
        processed[line_idx] = true;
        ConstPolygonRef line = lines[line_idx];
        
        Polygon chain = line;
        bool closest_is_closing_polygon = false;
        for (bool go_in_reverse_direction : { false, true })
        {
            if (go_in_reverse_direction)
            { // try extending chain in the other direction
                chain.reverse();
            }
            
            while (true)
            {
                Point from = chain.back();
                
                PolygonsPointIndex closest;
                coord_t closest_distance = std::numeric_limits<coord_t>::max();
                grid.processNearby(from, max_stitch_distance, 
                    std::function<bool (const PolygonsPointIndex&)> (
                        [from, &chain, &closest, &closest_is_closing_polygon, &closest_distance, &processed, max_stitch_distance, snap_distance](const PolygonsPointIndex& nearby)->bool
                    {
                        bool is_closing_segment = false;
                        coord_t dist = vSize(nearby.p() - from);
                        if (dist > max_stitch_distance)
                        {
                            return true; // keep looking
                        }
                        if (nearby.p() == chain.front())
                        {
                            if (chain.polylineLength() + dist < 3 * max_stitch_distance // prevent closing of small poly, cause it might be able to continue making a larger polyline
                                || chain.size() <= 2) // don't make 2 vert polygons
                            {
                                return true; // look for a better next line
                            }
                            is_closing_segment = true;
                            dist += 10; // prefer continuing polyline over closing a polygon; avoids closed zigzags from being printed separately
                            // continue to see if closing segment is also the closest 
                            // there might be a segment smaller than [max_stitch_distance] which closes the polygon better
                        }
                        else if (processed[nearby.poly_idx])
                        { // it was already moved to output
                            return true; // keep looking for a connection
                        }
                        if (dist < closest_distance)
                        {
                            closest_distance = dist;
                            closest = nearby;
                            closest_is_closing_polygon = is_closing_segment;
                        }
                        if (dist < snap_distance)
                        { // we have found a good enough next line
                            return false; // stop looking for alternatives
                        }
                        return true; // keep processing elements 
                    })
                );
                
                if (!closest.initialized()          // we couldn't find any next line 
                    || closest_is_closing_polygon   // we closed the polygon
                )
                {
                    break;
                }
                

                if (closest.point_idx == 0)
                {
                    assert(vSize(chain.back() - *closest.getPolygon().begin()) <= max_stitch_distance + 10);
                    chain.insert(chain.end(), closest.getPolygon().begin(), closest.getPolygon().end());
                }
                else
                {
                    assert(vSize(chain.back() - *closest.getPolygon().rbegin()) <= max_stitch_distance + 10);
                    chain.insert(chain.end(), closest.getPolygon().rbegin(), closest.getPolygon().rend());
                }
                assert( ! processed[closest.poly_idx]);
                processed[closest.poly_idx] = true;
            }
            if (closest_is_closing_polygon)
            {
                break; // don't consider reverse direction
            }
        }
        if (closest_is_closing_polygon)
        {
            result_polygons.add(chain);
        }
        else
        {
            result_lines.add(chain);
        }
    }
}

}//namespace cura

