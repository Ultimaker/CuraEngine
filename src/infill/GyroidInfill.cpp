//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GyroidInfill.h"
#include "../utils/AABB.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

namespace cura {

GyroidInfill::GyroidInfill() {
}

GyroidInfill::~GyroidInfill() {
}

void GyroidInfill::generateTotalGyroidInfill(Polygons& result_lines, bool zig_zaggify, coord_t outline_offset, coord_t infill_line_width, coord_t line_distance, const Polygons& in_outline, coord_t z)
{
    // generate infill based on the gyroid equation: sin_x * cos_y + sin_y * cos_z + sin_z * cos_x = 0
    // kudos to the author of the Slic3r implementation equation code, the equation code here is based on that

    if (zig_zaggify)
    {
        outline_offset -= infill_line_width / 2; // the infill line zig zag connections must lie next to the border, not on it
    }

    const Polygons outline = in_outline.offset(outline_offset);
    const AABB aabb(outline);

    int pitch = line_distance * 2.41; // this produces similar density to the "line" infill pattern
    int num_steps = 4;
    int step = pitch / num_steps;
    while (step > 500 && num_steps < 16)
    {
        num_steps *= 2;
        step = pitch / num_steps;
    }
    pitch = step * num_steps; // recalculate to avoid precision errors
    const double z_rads = 2 * M_PI * z / pitch;
    const double cos_z = std::cos(z_rads);
    const double sin_z = std::sin(z_rads);
    std::vector<coord_t> odd_line_coords;
    std::vector<coord_t> even_line_coords;
    Polygons result;
    std::vector<Point> chains[2]; // [start_points[], end_points[]]
    std::vector<unsigned> connected_to[2]; // [chain_indices[], chain_indices[]]
    std::vector<int> line_numbers; // which row/column line a chain is part of
    if (std::abs(sin_z) <= std::abs(cos_z))
    {
        // "vertical" lines
        const double phase_offset = ((cos_z < 0) ? M_PI : 0) + M_PI;
        for (coord_t y = 0; y < pitch; y += step)
        {
            const double y_rads = 2 * M_PI * y / pitch;
            const double a = cos_z;
            const double b = std::sin(y_rads + phase_offset);
            const double odd_c = sin_z * std::cos(y_rads + phase_offset);
            const double even_c = sin_z * std::cos(y_rads + phase_offset + M_PI);
            const double h = std::sqrt(a * a + b * b);
            const double odd_x_rads = ((h != 0) ? std::asin(odd_c / h) + std::asin(b / h) : 0) - M_PI/2;
            const double even_x_rads = ((h != 0) ? std::asin(even_c / h) + std::asin(b / h) : 0) - M_PI/2;
            odd_line_coords.push_back(odd_x_rads / M_PI * pitch);
            even_line_coords.push_back(even_x_rads / M_PI * pitch);
        }
        const unsigned num_coords = odd_line_coords.size();
        unsigned num_columns = 0;
        for (coord_t x = (std::floor(aabb.min.X / pitch) - 2.25) * pitch; x <= aabb.max.X + pitch/2; x += pitch/2)
        {
            bool is_first_point = true;
            Point last;
            bool last_inside = false;
            unsigned chain_end_index = 0;
            Point chain_end[2];
            for (coord_t y = (std::floor(aabb.min.Y / pitch) - 1) * pitch; y <= aabb.max.Y + pitch; y += pitch)
            {
                for (unsigned i = 0; i < num_coords; ++i)
                {
                    Point current(x + ((num_columns & 1) ? odd_line_coords[i] : even_line_coords[i])/2 + pitch, y + (coord_t)(i * step));
                    bool current_inside = outline.inside(current, true);
                    if (!is_first_point)
                    {
                        if (last_inside && current_inside)
                        {
                            // line doesn't hit the boundary, add the whole line
                            result.addLine(last, current);
                        }
                        else if (last_inside != current_inside)
                        {
                            // line hits the boundary, add the part that's inside the boundary
                            Polygons line;
                            line.addLine(last, current);
                            line = outline.intersectionPolyLines(line);
                            if (line.size() > 0)
                            {
                                // some of the line is inside the boundary
                                result.addLine(line[0][0], line[0][1]);
                                if (zig_zaggify)
                                {
                                    chain_end[chain_end_index] = line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1];
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
                                    chain_end[chain_end_index] = (last_inside) ? last : current;
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
                    }
                    last = current;
                    last_inside = current_inside;
                    is_first_point = false;
                }
            }
            ++num_columns;
        }
    }
    else
    {
        // "horizontal" lines
        const double phase_offset = (sin_z < 0) ? M_PI : 0;
        for (coord_t x = 0; x < pitch; x += step)
        {
            const double x_rads = 2 * M_PI * x / pitch;
            const double a = sin_z;
            const double b = std::cos(x_rads + phase_offset);
            const double odd_c = cos_z * std::sin(x_rads + phase_offset + M_PI);
            const double even_c = cos_z * std::sin(x_rads + phase_offset);
            const double h = std::sqrt(a * a + b * b);
            const double odd_y_rads = ((h != 0) ? std::asin(odd_c / h) + std::asin(b / h) : 0) + M_PI/2;
            const double even_y_rads = ((h != 0) ? std::asin(even_c / h) + std::asin(b / h) : 0) + M_PI/2;
            odd_line_coords.push_back(odd_y_rads / M_PI * pitch);
            even_line_coords.push_back(even_y_rads / M_PI * pitch);
        }
        const unsigned num_coords = odd_line_coords.size();
        unsigned num_rows = 0;
        for (coord_t y = (std::floor(aabb.min.Y / pitch) - 1) * pitch; y <= aabb.max.Y + pitch/2; y += pitch/2)
        {
            bool is_first_point = true;
            Point last;
            bool last_inside = false;
            unsigned chain_end_index = 0;
            Point chain_end[2];
            for (coord_t x = (std::floor(aabb.min.X / pitch) - 1) * pitch; x <= aabb.max.X + pitch; x += pitch)
            {
                for (unsigned i = 0; i < num_coords; ++i)
                {
                    Point current(x + (coord_t)(i * step), y + ((num_rows & 1) ? odd_line_coords[i] : even_line_coords[i])/2);
                    bool current_inside = outline.inside(current, true);
                    if (!is_first_point)
                    {
                        if (last_inside && current_inside)
                        {
                            // line doesn't hit the boundary, add the whole line
                            result.addLine(last, current);
                        }
                        else if (last_inside != current_inside)
                        {
                            // line hits the boundary, add the part that's inside the boundary
                            Polygons line;
                            line.addLine(last, current);
                            line = outline.intersectionPolyLines(line);
                            if (line.size() > 0)
                            {
                                // some of the line is inside the boundary
                                result.addLine(line[0][0], line[0][1]);
                                if (zig_zaggify)
                                {
                                    chain_end[chain_end_index] = line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1];
                                    if (++chain_end_index == 2)
                                    {
                                        chains[0].push_back(chain_end[0]);
                                        chains[1].push_back(chain_end[1]);
                                        chain_end_index = 0;
                                        connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                                        connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                                        line_numbers.push_back(num_rows);
                                    }
                                }
                            }
                            else
                            {
                                // none of the line is inside the boundary so the point that's actually on the boundary
                                // is the chain end
                                if (zig_zaggify)
                                {
                                    chain_end[chain_end_index] = (last_inside) ? last : current;
                                    if (++chain_end_index == 2)
                                    {
                                        chains[0].push_back(chain_end[0]);
                                        chains[1].push_back(chain_end[1]);
                                        chain_end_index = 0;
                                        connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                                        connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                                        line_numbers.push_back(num_rows);
                                    }
                                }
                            }
                        }
                    }
                    last = current;
                    last_inside = current_inside;
                    is_first_point = false;
                }
            }
            ++num_rows;
        }
    }

    if (zig_zaggify && chains[0].size() > 0)
    {
        // zig-zaggification consists of joining alternate chain ends to make a chain of chains
        // the basic algorithm is that we follow the infill area boundary and as we progress we are either drawing a connector or not
        // whenever we come across the end of a chain we toggle the connector drawing state
        // things are made more complicated by the fact that we want to avoid generating loops and so we need to keep track
        // of the indentity of the first chain in a connected sequence

        int chain_ends_remaining = chains[0].size() * 2;

        for (ConstPolygonRef outline_poly : outline)
        {
            std::vector<Point> connector_points; // the points that make up a connector line

            // we need to remember the first chain processed and the path to it from the first outline point
            // so that later we can possibly connect to it from the last chain processed
            unsigned first_chain_chain_index = std::numeric_limits<unsigned>::max();
            std::vector<Point> path_to_first_chain;

            bool drawing = false; // true when a connector line is being (potentially) created

            // keep track of the chain+point that a connector line started at
            unsigned connector_start_chain_index = std::numeric_limits<unsigned>::max();
            unsigned connector_start_point_index = std::numeric_limits<unsigned>::max();

            Point cur_point; // current point of interest - either an outline point or a chain end

            // go round all of the region's outline and find the chain ends that meet it
            // quit the loop early if we have seen all the chain ends and are not currently drawing a connector
            for (unsigned outline_point_index = 0; (chain_ends_remaining > 0 || drawing) && outline_point_index < outline_poly.size(); ++outline_point_index)
            {
                Point op0 = outline_poly[outline_point_index];
                Point op1 = outline_poly[(outline_point_index + 1) % outline_poly.size()];
                std::vector<unsigned> points_on_outline_chain_index;
                std::vector<unsigned> points_on_outline_point_index;

                // collect the chain ends that meet this segment of the outline
                for (unsigned chain_index = 0; chain_index < chains[0].size(); ++chain_index)
                {
                    for (unsigned point_index = 0; point_index < 2; ++point_index)
                    {
                        // don't include chain ends that are close to the segment but are beyond the segment ends
                        short beyond = 0;
                        if (LinearAlg2D::getDist2FromLineSegment(op0, chains[point_index][chain_index], op1, &beyond) < 10 && !beyond)
                        {
                            points_on_outline_point_index.push_back(point_index);
                            points_on_outline_chain_index.push_back(chain_index);
                        }
                    }
                }

                if (outline_point_index == 0 || vSize2(op0 - cur_point) > 100)
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
                    float nearest_point_dist2 = std::numeric_limits<float>::infinity();
                    for (unsigned pi = 0; pi < points_on_outline_chain_index.size(); ++pi)
                    {
                        float dist2 = vSize2f(chains[points_on_outline_point_index[pi]][points_on_outline_chain_index[pi]] - cur_point);
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

                    if (drawing && connector_points.size() > 0 && vSize2(cur_point - connector_points.back()) < 100)
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
                            for (unsigned pi = 1; pi < connector_points.size(); ++pi)
                            {
                                result.addLine(connector_points[pi - 1], connector_points[pi]);
                            }
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
            if (drawing && first_chain_chain_index != std::numeric_limits<unsigned>::max()
                && first_chain_chain_index != connector_start_chain_index
                && connected_to[0][first_chain_chain_index] != connector_start_chain_index
                && connected_to[1][first_chain_chain_index] != connector_start_chain_index)
            {
                // output the connector line segments from the last chain to the first point in the outline
                connector_points.push_back(outline_poly[0]);
                for (unsigned pi = 1; pi < connector_points.size(); ++pi)
                {
                    result.addLine(connector_points[pi - 1], connector_points[pi]);
                }
                // output the connector line segments from the first point in the outline to the first chain
                for (unsigned pi = 1; pi < path_to_first_chain.size(); ++pi)
                {
                    result.addLine(path_to_first_chain[pi - 1], path_to_first_chain[pi]);
                }
            }

            if (chain_ends_remaining < 1)
            {
                break;
            }
        }
    }

    result_lines = result;
}

} // namespace cura