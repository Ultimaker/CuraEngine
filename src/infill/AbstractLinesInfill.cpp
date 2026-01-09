// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/AbstractLinesInfill.h"

#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"
#include "utils/OpenPolylineStitcher.h"
#include "utils/linearAlg2D.h"

namespace cura
{

void AbstractLinesInfill::generateInfill(
    OpenLinesSet& result_polylines,
    Shape& result_polygons,
    const bool zig_zaggify,
    const coord_t line_distance,
    const Shape& in_outline,
    const coord_t z,
    const coord_t line_width,
    const AngleDegrees& rotation) const
{
    Shape rotated_outline = in_outline;
    PointMatrix rotation_matrix(rotation);
    if (rotation != 0)
    {
        rotated_outline.applyMatrix(rotation_matrix);
    }

    const OpenLinesSet raw_lines = generateParallelLines(line_distance, rotated_outline, z, line_width);
    const OpenLinesSet fit_lines = fitLines(raw_lines, zig_zaggify, rotated_outline);
    OpenPolylineStitcher::stitch(fit_lines, result_polylines, result_polygons, line_width);

    if (rotation != 0)
    {
        rotation_matrix = rotation_matrix.inverse();
        result_polylines.applyMatrix(rotation_matrix);
        result_polygons.applyMatrix(rotation_matrix);
    }
}

OpenLinesSet AbstractLinesInfill::zigZaggify(
    const std::array<std::vector<Point2LL>, 2>& chains,
    std::array<std::vector<unsigned>, 2>& connected_to,
    const std::vector<int>& line_numbers,
    const Shape& in_outline)
{
    // zig-zaggification consists of joining alternate chain ends to make a chain of chains
    // the basic algorithm is that we follow the infill area boundary and as we progress we are either drawing a connector or not
    // whenever we come across the end of a chain we toggle the connector drawing state
    // things are made more complicated by the fact that we want to avoid generating loops and so we need to keep track
    // of the indentity of the first chain in a connected sequence

    OpenLinesSet result;
    int chain_ends_remaining = chains[0].size() * 2;

    for (const Polygon& outline_poly : in_outline)
    {
        std::vector<Point2LL> connector_points; // the points that make up a connector line

        // we need to remember the first chain processed and the path to it from the first outline point
        // so that later we can possibly connect to it from the last chain processed
        unsigned first_chain_chain_index = std::numeric_limits<unsigned>::max();
        std::vector<Point2LL> path_to_first_chain;

        bool drawing = false; // true when a connector line is being (potentially) created

        // keep track of the chain+point that a connector line started at
        unsigned connector_start_chain_index = std::numeric_limits<unsigned>::max();
        unsigned connector_start_point_index = std::numeric_limits<unsigned>::max();

        Point2LL cur_point; // current point of interest - either an outline point or a chain end

        // go round all of the region's outline and find the chain ends that meet it
        // quit the loop early if we have seen all the chain ends and are not currently drawing a connector
        for (unsigned outline_point_index = 0; (chain_ends_remaining > 0 || drawing) && outline_point_index < outline_poly.size(); ++outline_point_index)
        {
            Point2LL op0 = outline_poly[outline_point_index];
            Point2LL op1 = outline_poly[(outline_point_index + 1) % outline_poly.size()];
            std::vector<unsigned> points_on_outline_chain_index;
            std::vector<unsigned> points_on_outline_point_index;

            // collect the chain ends that meet this segment of the outline
            for (unsigned chain_index = 0; chain_index < chains[0].size(); ++chain_index)
            {
                for (unsigned point_index = 0; point_index < 2; ++point_index)
                {
                    // don't include chain ends that are close to the segment but are beyond the segment ends
                    short beyond = 0;
                    if (LinearAlg2D::getDist2FromLineSegment(op0, chains[point_index][chain_index], op1, &beyond) < 10 && ! beyond)
                    {
                        points_on_outline_point_index.push_back(point_index);
                        points_on_outline_chain_index.push_back(chain_index);
                    }
                }
            }

            if (outline_point_index == 0 || vSize2(op0 - cur_point) > MM2INT(0.1))
            {
                // this is either the first outline point or it is another outline point that is not too close to cur_point

                if (first_chain_chain_index == std::numeric_limits<unsigned>::max())
                {
                    // include the outline point in the path to the first chain
                    path_to_first_chain.push_back(op0);
                }

                cur_point = op0;
                if (drawing)
                {
                    // include the start point of this outline segment in the connector
                    connector_points.push_back(op0);
                }
            }

            // iterate through each of the chain ends that meet the current outline segment
            while (points_on_outline_chain_index.size() > 0)
            {
                // find the nearest chain end to the current point
                unsigned nearest_point_index = 0;
                double nearest_point_dist2 = std::numeric_limits<float>::infinity();
                for (unsigned pi = 0; pi < points_on_outline_chain_index.size(); ++pi)
                {
                    double dist2 = vSize2f(chains[points_on_outline_point_index[pi]][points_on_outline_chain_index[pi]] - cur_point);
                    if (dist2 < nearest_point_dist2)
                    {
                        nearest_point_dist2 = dist2;
                        nearest_point_index = pi;
                    }
                }
                const unsigned point_index = points_on_outline_point_index[nearest_point_index];
                const unsigned chain_index = points_on_outline_chain_index[nearest_point_index];

                // make the chain end the current point and add it to the connector line
                cur_point = chains[point_index][chain_index];

                if (drawing && connector_points.size() > 0 && vSize2(cur_point - connector_points.back()) < MM2INT(0.1))
                {
                    // this chain end will be too close to the last connector point so throw away the last connector point
                    connector_points.pop_back();
                }
                connector_points.push_back(cur_point);

                if (first_chain_chain_index == std::numeric_limits<unsigned>::max())
                {
                    // this is the first chain to be processed, remember it
                    first_chain_chain_index = chain_index;
                    path_to_first_chain.push_back(cur_point);
                }

                if (drawing)
                {
                    // add the connector line segments but only if
                    //  1 - the start/end points are not the opposite ends of the same chain
                    //  2 - the other end of the current chain is not connected to the chain the connector line is coming from

                    if (chain_index != connector_start_chain_index && connected_to[(point_index + 1) % 2][chain_index] != connector_start_chain_index)
                    {
                        result.push_back(OpenPolyline{ connector_points });
                        drawing = false;
                        connector_points.clear();
                        // remember the connection
                        connected_to[point_index][chain_index] = connector_start_chain_index;
                        connected_to[connector_start_point_index][connector_start_chain_index] = chain_index;
                    }
                    else
                    {
                        // start a new connector from the current location
                        connector_points.clear();
                        connector_points.push_back(cur_point);

                        // remember the chain+point that the connector started from
                        connector_start_chain_index = chain_index;
                        connector_start_point_index = point_index;
                    }
                }
                else
                {
                    // we have just jumped a gap so now we want to start drawing again
                    drawing = true;

                    // if this connector is the first to be created or we are not connecting chains from the same row/column,
                    // remember the chain+point that this connector is starting from
                    if (connector_start_chain_index == std::numeric_limits<unsigned>::max() || line_numbers[chain_index] != line_numbers[connector_start_chain_index])
                    {
                        connector_start_chain_index = chain_index;
                        connector_start_point_index = point_index;
                    }
                }

                // done with this chain end
                points_on_outline_chain_index.erase(points_on_outline_chain_index.begin() + nearest_point_index);
                points_on_outline_point_index.erase(points_on_outline_point_index.begin() + nearest_point_index);

                // decrement total amount of work to do
                --chain_ends_remaining;
            }
        }

        // we have now visited all the points in the outline, if a connector was (potentially) being drawn
        // check whether the first chain is already connected to the last chain and, if not, draw the
        // connector between
        if (drawing && first_chain_chain_index != std::numeric_limits<unsigned>::max() && first_chain_chain_index != connector_start_chain_index
            && connected_to[0][first_chain_chain_index] != connector_start_chain_index && connected_to[1][first_chain_chain_index] != connector_start_chain_index)
        {
            // output the connector line segments from the last chain to the first point in the outline
            connector_points.push_back(outline_poly[0]);
            result.push_back(OpenPolyline{ connector_points });
            // output the connector line segments from the first point in the outline to the first chain
            result.push_back(OpenPolyline{ path_to_first_chain });
        }

        if (chain_ends_remaining < 1)
        {
            break;
        }
    }

    return result;
}

OpenLinesSet AbstractLinesInfill::fitLines(const OpenLinesSet& raw_lines, const bool zig_zaggify, const Shape& in_outline)
{
    OpenLinesSet result;
    unsigned num_columns = 0;
    std::array<std::vector<Point2LL>, 2> chains; // [start_points[], end_points[]]
    std::array<std::vector<unsigned>, 2> connected_to; // [chain_indices[], chain_indices[]]
    std::vector<int> line_numbers; // which row/column line a chain is part o

    for (const OpenPolyline& raw_line : raw_lines)
    {
        if (! raw_line.isValid())
        {
            continue;
        }

        Point2LL last = raw_line.front();
        bool last_inside = in_outline.inside(last, true);
        unsigned chain_end_index = 0;
        Point2LL chain_end[2];
        for (auto iterator = raw_line.beginSegments(); iterator != raw_line.endSegments(); ++iterator)
        {
            const Point2LL& point = (*iterator).end;
            const bool current_inside = in_outline.inside(point, true);
            if (last_inside && current_inside)
            {
                // line doesn't hit the boundary, add the whole line
                result.addSegment(last, point);
            }
            else if (last_inside != current_inside)
            {
                // line hits the boundary, add the part that's inside the boundary
                OpenLinesSet line;
                line.addSegment(last, point);
                constexpr bool restitch = false; // only a single line doesn't need stitching
                line = in_outline.intersection(line, restitch);
                if (line.size() > 0)
                {
                    // some of the line is inside the boundary
                    result.addSegment(line[0][0], line[0][1]);
                    if (zig_zaggify)
                    {
                        chain_end[chain_end_index] = line[0][(line[0][0] != last && line[0][0] != point) ? 0 : 1];
                        if (++chain_end_index == 2)
                        {
                            chains[0].push_back(chain_end[0]);
                            chains[1].push_back(chain_end[1]);
                            chain_end_index = 0;
                            connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                            connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                            line_numbers.push_back(num_columns);
                        }
                    }
                }
                else
                {
                    // none of the line is inside the boundary so the point that's actually on the boundary
                    // is the chain end
                    if (zig_zaggify)
                    {
                        chain_end[chain_end_index] = (last_inside) ? last : point;
                        if (++chain_end_index == 2)
                        {
                            chains[0].push_back(chain_end[0]);
                            chains[1].push_back(chain_end[1]);
                            chain_end_index = 0;
                            connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                            connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                            line_numbers.push_back(num_columns);
                        }
                    }
                }
            }

            last = point;
            last_inside = current_inside;
        }
        ++num_columns;
    }

    if (zig_zaggify && chains[0].size() > 0)
    {
        result.push_back(zigZaggify(chains, connected_to, line_numbers, in_outline));
    }

    return result;
}

} // namespace cura
