// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolylineStitcher.h"

#include "geometry/ClosedLinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "utils/ExtrusionLineStitcher.h"
#include "utils/OpenPolylineStitcher.h"
#include "utils/PolygonsPointIndex.h"

namespace cura
{

template<>
bool ExtrusionLineStitcher::canReverse(const PathsPointIndex<VariableWidthLines>& ppi)
{
    if ((*ppi.polygons_)[ppi.poly_idx_].is_odd_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<>
bool OpenPolylineStitcher::canReverse(const PathsPointIndex<OpenLinesSet>&)
{
    return true;
}

template<>
bool PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>::canReverse(const PathsPointIndex<OpenLinesSet>&)
{
    return true;
}

template<>
bool ExtrusionLineStitcher::canConnect(const ExtrusionLine& a, const ExtrusionLine& b)
{
    return a.is_odd_ == b.is_odd_;
}

template<>
bool OpenPolylineStitcher::canConnect(const OpenPolyline&, const OpenPolyline&)
{
    return true;
}

template<>
bool PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>::canConnect(const OpenPolyline&, const OpenPolyline&)
{
    return true;
}

template<>
bool ExtrusionLineStitcher::isOdd(const ExtrusionLine& line)
{
    return line.is_odd_;
}

template<>
bool OpenPolylineStitcher::isOdd(const OpenPolyline&)
{
    return false;
}

template<>
bool PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>::isOdd(const OpenPolyline&)
{
    return false;
}

template<>
void ExtrusionLineStitcher::pushToClosedResult(VariableWidthLines& result_polygons, const ExtrusionLine& polyline)
{
    result_polygons.push_back(polyline);
}

template<>
void OpenPolylineStitcher::pushToClosedResult(Shape& result_polygons, const OpenPolyline& polyline)
{
    result_polygons.emplace_back(polyline.getPoints(), true);
}

template<>
void PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>::pushToClosedResult(ClosedLinesSet& result_polygons, const OpenPolyline& polyline)
{
    result_polygons.emplace_back(polyline.getPoints(), true);
}

template<typename InputPaths, typename OutputPaths, typename Path, typename Junction>
void PolylineStitcher<InputPaths, OutputPaths, Path, Junction>::stitch(
    const InputPaths& lines,
    InputPaths& result_lines,
    OutputPaths& result_polygons,
    coord_t max_stitch_distance,
    coord_t snap_distance)
{
    if (lines.empty())
    {
        return;
    }

    SparsePointGrid<PathsPointIndex<InputPaths>, PathsPointIndexLocator<InputPaths>> grid(max_stitch_distance, lines.size() * 2);

    // populate grid
    for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
    {
        const auto line = lines[line_idx];
        grid.insert(PathsPointIndex<InputPaths>(&lines, line_idx, 0));
        grid.insert(PathsPointIndex<InputPaths>(&lines, line_idx, line.size() - 1));
    }

    std::vector<bool> processed(lines.size(), false);

    for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
    {
        if (processed[line_idx])
        {
            continue;
        }
        processed[line_idx] = true;
        const auto line = lines[line_idx];
        bool should_close = isOdd(line);

        Path chain = line;
        bool closest_is_closing_polygon = false;
        for (bool go_in_reverse_direction : { false, true }) // first go in the unreversed direction, to try to prevent the chain.reverse() operation.
        { // NOTE: Implementation only works for this order; we currently only re-reverse the chain when it's closed.
            if (go_in_reverse_direction)
            { // try extending chain in the other direction
                chain.reverse();
            }
            coord_t chain_length = chain.length();

            while (true)
            {
                Point2LL from = make_point(chain.back());

                PathsPointIndex<InputPaths> closest;
                coord_t closest_distance = std::numeric_limits<coord_t>::max();
                grid.processNearby(
                    from,
                    max_stitch_distance,
                    std::function<bool(const PathsPointIndex<InputPaths>&)>(
                        [from,
                         &chain,
                         &closest,
                         &closest_is_closing_polygon,
                         &closest_distance,
                         &processed,
                         &chain_length,
                         go_in_reverse_direction,
                         max_stitch_distance,
                         snap_distance,
                         should_close](const PathsPointIndex<InputPaths>& nearby) -> bool
                        {
                            bool is_closing_segment = false;
                            coord_t dist = vSize(nearby.p() - from);
                            if (dist > max_stitch_distance)
                            {
                                return true; // keep looking
                            }
                            if (vSize2(nearby.p() - make_point(chain.front())) < snap_distance * snap_distance)
                            {
                                if (chain_length + dist < 3 * max_stitch_distance // prevent closing of small poly, cause it might be able to continue making a larger polyline
                                    || chain.size() <= 2) // don't make 2 vert polygons
                                {
                                    return true; // look for a better next line
                                }
                                is_closing_segment = true;
                                if (! should_close)
                                {
                                    dist += 10; // prefer continuing polyline over closing a polygon; avoids closed zigzags from being printed separately
                                    // continue to see if closing segment is also the closest
                                    // there might be a segment smaller than [max_stitch_distance] which closes the polygon better
                                }
                                else
                                {
                                    dist -= 10; // Prefer closing the polygon if it's 100% even lines. Used to create closed contours.
                                    // Continue to see if closing segment is also the closest.
                                }
                            }
                            else if (processed[nearby.poly_idx_])
                            { // it was already moved to output
                                return true; // keep looking for a connection
                            }
                            bool nearby_would_be_reversed = nearby.point_idx_ != 0;
                            nearby_would_be_reversed = nearby_would_be_reversed != go_in_reverse_direction; // flip nearby_would_be_reversed when searching in the reverse direction
                            if (! canReverse(nearby) && nearby_would_be_reversed)
                            { // connecting the segment would reverse the polygon direction
                                return true; // keep looking for a connection
                            }
                            if (! canConnect(chain, (*nearby.polygons_)[nearby.poly_idx_]))
                            {
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
                        }));

                if (! closest.initialized() // we couldn't find any next line
                    || closest_is_closing_polygon // we closed the polygon
                )
                {
                    break;
                }


                coord_t segment_dist = vSize(make_point(chain.back()) - closest.p());
                assert(segment_dist <= max_stitch_distance + 10);
                const size_t old_size = chain.size();
                if (closest.point_idx_ == 0)
                {
                    auto start_pos = (*closest.polygons_)[closest.poly_idx_].begin();
                    if (segment_dist < snap_distance)
                    {
                        ++start_pos;
                    }
                    chain.insert(chain.end(), start_pos, (*closest.polygons_)[closest.poly_idx_].end());
                }
                else
                {
                    auto start_pos = (*closest.polygons_)[closest.poly_idx_].rbegin();
                    if (segment_dist < snap_distance)
                    {
                        ++start_pos;
                    }
                    chain.insert(chain.end(), start_pos, (*closest.polygons_)[closest.poly_idx_].rend());
                }
                for (size_t i = old_size; i < chain.size(); ++i) // Update chain length.
                {
                    chain_length += vSize(chain[i] - chain[i - 1]);
                }
                should_close = should_close & ! isOdd((*closest.polygons_)[closest.poly_idx_]); // If we connect an even to an odd line, we should no longer try to close it.
                assert(! processed[closest.poly_idx_]);
                processed[closest.poly_idx_] = true;
            }

            if (closest_is_closing_polygon)
            {
                if (go_in_reverse_direction)
                { // re-reverse chain to retain original direction
                    // NOTE: not sure if this code could ever be reached, since if a polygon can be closed that should be already possible in the forward direction
                    chain.reverse();
                }

                break; // don't consider reverse direction
            }
        }
        if (closest_is_closing_polygon)
        {
            pushToClosedResult(result_polygons, chain);
        }
        else
        {
            PathsPointIndex<InputPaths> ppi_here(&lines, line_idx, 0);
            if (! canReverse(ppi_here))
            { // Since closest_is_closing_polygon is false we went through the second iterations of the for-loop, where go_in_reverse_direction is true
                // the polyline isn't allowed to be reversed, so we re-reverse it.
                chain.reverse();
            }
            result_lines.emplace_back(chain);
        }
    }
}

template void OpenPolylineStitcher::stitch(const OpenLinesSet& lines, OpenLinesSet& result_lines, Shape& result_polygons, coord_t max_stitch_distance, coord_t snap_distance);

template void ExtrusionLineStitcher::stitch(
    const VariableWidthLines& lines,
    VariableWidthLines& result_lines,
    VariableWidthLines& result_polygons,
    coord_t max_stitch_distance,
    coord_t snap_distance);

template void PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>::stitch(
    const OpenLinesSet& lines,
    OpenLinesSet& result_lines,
    ClosedLinesSet& result_polygons,
    coord_t max_stitch_distance,
    coord_t snap_distance);

} // namespace cura
