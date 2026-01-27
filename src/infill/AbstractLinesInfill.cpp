// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/AbstractLinesInfill.h"

#include <fmt/format.h>
#include <range/v3/view/enumerate.hpp>

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

    const OpenLinesSet raw_lines = generateParallelLines(line_distance, AABB(rotated_outline), z, line_width);
    const OpenLinesSet fit_lines = fitLines(raw_lines, zig_zaggify, rotated_outline);
    OpenPolylineStitcher::stitch(fit_lines, result_polylines, result_polygons, line_width);

    if (rotation != 0)
    {
        rotation_matrix = rotation_matrix.inverse();
        result_polylines.applyMatrix(rotation_matrix);
        result_polygons.applyMatrix(rotation_matrix);
    }
}

OpenLinesSet AbstractLinesInfill::zigZaggify(std::vector<SplitLines>& split_lines, const Shape& in_outline)
{
    // zig-zaggification consists of joining alternate chain ends to make a chain of chains
    // the basic algorithm is that we follow the infill area boundary and as we progress we are either drawing a connector or not
    // whenever we come across the end of a chain we toggle the connector drawing state
    // things are made more complicated by the fact that we want to avoid generating loops and so we need to keep track
    // of the indentity of the first chain in a connected sequence

    OpenLinesSet result;
    int chain_ends_remaining = split_lines.size() * 2;
    std::vector<std::array<std::optional<size_t>, 2>> connected_to(split_lines.size());

    for (const Polygon& outline_poly : in_outline)
    {
        std::vector<Point2LL> connector_points; // the points that make up a connector line

        // we need to remember the first chain processed and the path to it from the first outline point
        // so that later we can possibly connect to it from the last chain processed
        std::optional<size_t> first_chain_chain_index;
        std::vector<Point2LL> path_to_first_chain;

        bool drawing = false; // true when a connector line is being (potentially) created

        // keep track of the chain+point that a connector line started at
        std::optional<size_t> connector_start_chain_index;
        std::optional<size_t> connector_start_point_index;

        Point2LL cur_point; // current point of interest - either an outline point or a chain end

        // go round all of the region's outline and find the chain ends that meet it
        // quit the loop early if we have seen all the chain ends and are not currently drawing a connector
        for (size_t outline_point_index = 0; (chain_ends_remaining > 0 || drawing) && outline_point_index < outline_poly.size(); ++outline_point_index)
        {
            Point2LL op0 = outline_poly[outline_point_index];
            Point2LL op1 = outline_poly[(outline_point_index + 1) % outline_poly.size()];
            std::vector<size_t> points_on_outline_chain_index;
            std::vector<size_t> points_on_outline_point_index;

            // collect the chain ends that meet this segment of the outline
            for (const auto& [chain_index, split_line] : split_lines | ranges::views::enumerate)
            {
                for (const auto& [point_index, point] : split_line.chain | ranges::views::enumerate)
                {
                    // don't include chain ends that are close to the segment but are beyond the segment ends
                    short beyond = 0;
                    if (LinearAlg2D::getDist2FromLineSegment(op0, point, op1, &beyond) < 10 && ! beyond)
                    {
                        points_on_outline_point_index.push_back(point_index);
                        points_on_outline_chain_index.push_back(chain_index);
                    }
                }
            }

            if (outline_point_index == 0 || vSize2(op0 - cur_point) > MM2INT(0.1))
            {
                // this is either the first outline point or it is another outline point that is not too close to cur_point

                if (! first_chain_chain_index.has_value())
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
                size_t nearest_point_index = 0;
                double nearest_point_dist2 = std::numeric_limits<float>::infinity();
                for (size_t pi = 0; pi < points_on_outline_chain_index.size(); ++pi)
                {
                    double dist2 = vSize2f(split_lines[points_on_outline_chain_index[pi]].chain[points_on_outline_point_index[pi]] - cur_point);
                    if (dist2 < nearest_point_dist2)
                    {
                        nearest_point_dist2 = dist2;
                        nearest_point_index = pi;
                    }
                }
                const size_t point_index = points_on_outline_point_index[nearest_point_index];
                const size_t chain_index = points_on_outline_chain_index[nearest_point_index];

                // make the chain end the current point and add it to the connector line
                cur_point = split_lines[chain_index].chain[point_index];

                if (drawing && connector_points.size() > 0 && vSize2(cur_point - connector_points.back()) < MM2INT(0.1))
                {
                    // this chain end will be too close to the last connector point so throw away the last connector point
                    connector_points.pop_back();
                }
                connector_points.push_back(cur_point);

                if (! first_chain_chain_index.has_value())
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

                    if (chain_index != connector_start_chain_index && connected_to[chain_index][(point_index + 1) % 2] != connector_start_chain_index)
                    {
                        result.push_back(OpenPolyline{ connector_points });
                        drawing = false;
                        connector_points.clear();
                        // remember the connection
                        connected_to[chain_index][point_index] = connector_start_chain_index;
                        connected_to[*connector_start_chain_index][*connector_start_point_index] = chain_index;
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
                    if (! connector_start_chain_index.has_value() || split_lines[chain_index].column_id != split_lines[*connector_start_chain_index].column_id)
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
        if (drawing && first_chain_chain_index.has_value() && first_chain_chain_index != connector_start_chain_index
            && connected_to[*first_chain_chain_index][0] != connector_start_chain_index && connected_to[*first_chain_chain_index][1] != connector_start_chain_index)
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
    std::vector<SplitLines> split_lines;

    for (const auto& [column_id, raw_line] : raw_lines | ranges::views::enumerate)
    {
        if (! raw_line.isValid())
        {
            continue;
        }

        constexpr bool restitch = false;
        constexpr coord_t max_stitching_distance = 0;
        constexpr bool split_into_segments = false;
        const OpenLinesSet intersected_lines = in_outline.intersection(OpenLinesSet(raw_line), restitch, max_stitching_distance, split_into_segments);
        for (const OpenPolyline& intersected_line : intersected_lines)
        {
            result.push_back(intersected_line);
            if (zig_zaggify)
            {
                split_lines.emplace_back(SplitLines{ std::array{ intersected_line.front(), intersected_line.back() }, column_id });
            }
        }
    }

    if (! split_lines.empty())
    {
        result.push_back(zigZaggify(split_lines, in_outline));
    }

    return result;
}

} // namespace cura
