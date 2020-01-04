//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TPMSInfill.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

namespace cura {

void TPMSInfillSchwarzP::generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step)
{
    // generate Schwarz P "Primitive" surface defined by equation: cos(x) + cos(y) + cos(z) = 0
    // see https://en.wikipedia.org/wiki/Schwarz_minimal_surface

    const double cos_z = std::cos(2 * M_PI * z / pitch);

    std::vector<bool> gaps;
    std::vector<coord_t> x_coords;
    std::vector<coord_t> y_coords;

    bool at_start = true;
    double large_y_inc = step / 4.0;
    double small_y_inc = step / 16.0;
    double y_inc = small_y_inc;

    // generate coordinates for first half of the steps in the y direction
    for (double y = 0; y < pitch/2; y += y_inc)
    {
        const double y_rads = 2 * M_PI * y / pitch;
        double x_val = std::cos(y_rads) + cos_z;
        // if x_val is not in range, we have a gap in the line
        bool gap = (x_val < -1.0 || x_val > 1.0);
        if (at_start)
        {
            if (gap)
            {
                if (gaps.empty())
                {
                    // mark the gap
                    gaps.push_back(true);
                    x_coords.push_back(0);
                    y_coords.push_back(0);
                }
                continue;
            }
            if (!gaps.empty())
            {
                // insert extra point that lies exactly on the surface
                gaps.push_back(false);
                x_coords.push_back(std::acos((x_val < 0) ? -1.0 : 1.0) / M_PI / 2 * pitch + pitch/4);
                y_coords.push_back(y);
            }
            at_start = false;
            y_inc = large_y_inc;
        }
        else if (gap)
        {
            if (y_inc != small_y_inc)
            {
                // we have gone too far, go back and use smaller increments
                y -= y_inc;
                y_inc = small_y_inc;
                continue;
            }
            else
            {
                // insert extra point that lies exactly on the surface
                gaps.push_back(false);
                x_coords.push_back(std::acos((x_val < 0) ? -1.0 : 1.0) / M_PI / 2 * pitch + pitch/4);
                y_coords.push_back(y);
            }
        }

        if (gap)
        {
            if(gaps.empty() || !gaps.back())
            {
                // mark the gap
                gaps.push_back(true);
                x_coords.push_back(0);
                y_coords.push_back(0);
            }
        }
        else
        {
            // a point on the surface
            gaps.push_back(false);
            x_coords.push_back(std::acos(x_val) / M_PI / 2 * pitch + pitch/4);
            y_coords.push_back(y);
        }
    }

    // value for y == pitch/2
    double x_val = cos_z - 1;
    if (x_val < -1.0)
    {
        gaps.push_back(true);
        x_coords.push_back(0);
        y_coords.push_back(0);
    }
    else
    {
        gaps.push_back(false);
        x_coords.push_back(std::acos(x_val) / M_PI / 2 * pitch + pitch/4);
        y_coords.push_back(pitch/2);
    }

    // mirror the values below pitch/2

    for (int i = x_coords.size() - 2; i > 0; --i)
    {
        gaps.push_back(gaps[i]);
        x_coords.push_back(x_coords[i]);
        y_coords.push_back(pitch - y_coords[i]);
    }

    // repeat the lines over the infill area
    const unsigned num_coords = x_coords.size();
    unsigned num_columns = 0;
    for (coord_t x = x_min - pitch; x < x_max; x += pitch/2)
    {
        bool is_first_point = true;
        Point last;
        bool last_inside = false;
        unsigned num_rows = 0;
        for (coord_t y = y_min; y < y_max; y += pitch)
        {
            for (unsigned i = 0; i < num_coords; ++i)
            {
                if (gaps[i])
                {
                    is_first_point = true;
                    continue;
                }

                Point current(x + ((num_columns & 1) ? -x_coords[i] : x_coords[i]) + pitch, y + y_coords[i]);
                current = rotate_around_origin(current, fill_angle_rads);
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
                        unsigned connection_id = 0;
                        if (cos_z >= 0)
                        {
                            connection_id = (num_columns/2 << 16) | num_rows;
                        }
                        else
                        {
                            connection_id = (((num_columns+1)/2) << 16) | (num_rows + (i > num_coords/2));
                        }
                        if (line.size() > 0)
                        {
                            // some of the line is inside the boundary
                            result.addLine(line[0][0], line[0][1]);
                            if (zig_zaggify)
                            {
                                connection_points.push_back(line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1]);
                                connection_ids.push_back(connection_id);
                            }
                        }
                        else
                        {
                            // none of the line is inside the boundary so the point that's actually on the boundary
                            // is the chain end
                            if (zig_zaggify)
                            {
                                connection_points.push_back((last_inside) ? last : current);
                                connection_ids.push_back(connection_id);
                            }
                        }
                    }
                }
                last = current;
                last_inside = current_inside;
                is_first_point = false;
            }
            ++num_rows;
        }
        ++num_columns;
    }
}

void TPMSInfillSchwarzP::generateConnections(Polygons& result, const Polygons& outline)
{
    // zig-zaggification consists of simply joining alternate connection ends with lines that follow the outline

    int connections_remaining = connection_points.size();

    if (connections_remaining == 0)
    {
        return;
    }

    unsigned last_connection_point_id = std::numeric_limits<unsigned>::max();

    for (ConstPolygonRef outline_poly : outline)
    {
        std::vector<Point> connector_points; // the points that make up a connector line

        // we need to remember the first connection processed and the path to it from the first outline point
        // so that later we can possibly connect to it from the last connection processed
        unsigned first_connection_index = std::numeric_limits<unsigned>::max();
        std::vector<Point> path_to_first_connection;

        bool drawing = false; // true when a connector line is being (potentially) created

        Point cur_point; // current point of interest - either an outline point or a connection

        // go round all of the region's outline and find the connections that meet it
        // quit the loop early if we have seen all the connections and are not currently drawing a connector
        for (unsigned outline_point_index = 0; (connections_remaining > 0 || drawing) && outline_point_index < outline_poly.size(); ++outline_point_index)
        {
            Point op0 = outline_poly[outline_point_index];
            Point op1 = outline_poly[(outline_point_index + 1) % outline_poly.size()];
            std::vector<unsigned> points_on_outline_connection_point_index;

            // collect the connections that meet this segment of the outline
            for (unsigned connection_points_index = 0; connection_points_index < connection_points.size(); ++connection_points_index)
            {
                // don't include connections that are close to the segment but are beyond the segment ends
                short beyond = 0;
                if (LinearAlg2D::getDist2FromLineSegment(op0, connection_points[connection_points_index], op1, &beyond) < 10 && !beyond)
                {
                    points_on_outline_connection_point_index.push_back(connection_points_index);
                }
            }

            if (outline_point_index == 0 || vSize2(op0 - cur_point) > 100)
            {
                // this is either the first outline point or it is another outline point that is not too close to cur_point

                if (first_connection_index == std::numeric_limits<unsigned>::max())
                {
                    // include the outline point in the path to the first connection
                    path_to_first_connection.push_back(op0);
                }

                cur_point = op0;
                if (drawing)
                {
                    // include the start point of this outline segment in the connector
                    connector_points.push_back(op0);
                }
            }

            // iterate through each of the connection points that meet the current outline segment
            while (points_on_outline_connection_point_index.size() > 0)
            {
                // find the nearest connection point to the current point
                unsigned nearest_connection_point_index = 0;
                float nearest_point_dist2 = std::numeric_limits<float>::infinity();
                for (unsigned pi = 0; pi < points_on_outline_connection_point_index.size(); ++pi)
                {
                    float dist2 = vSize2f(connection_points[points_on_outline_connection_point_index[pi]] - cur_point);
                    if (dist2 < nearest_point_dist2)
                    {
                        nearest_point_dist2 = dist2;
                        nearest_connection_point_index = pi;
                    }
                }

                // make the connection the current point and add it to the connector line
                cur_point = connection_points[points_on_outline_connection_point_index[nearest_connection_point_index]];

                if (drawing && connector_points.size() > 0 && vSize2(cur_point - connector_points.back()) < 100)
                {
                    // this connection will be too close to the last connector point so throw away the last connector point
                    connector_points.pop_back();
                }
                connector_points.push_back(cur_point);

                if (first_connection_index == std::numeric_limits<unsigned>::max())
                {
                    // this is the first connection to be processed, remember it
                    first_connection_index = nearest_connection_point_index;
                    path_to_first_connection.push_back(cur_point);
                }

                const unsigned connection_point_id = connection_ids[points_on_outline_connection_point_index[nearest_connection_point_index]];

                if (drawing)
                {
                    if (last_connection_point_id != connection_point_id)
                    {
                        for (unsigned pi = 1; pi < connector_points.size(); ++pi)
                        {
                            result.addLine(connector_points[pi - 1], connector_points[pi]);
                        }
                        drawing = false;
                        connector_points.clear();
                    }
                    else
                    {
                        // start a new connector from the current location
                        connector_points.clear();
                        connector_points.push_back(cur_point);
                    }
                }
                else
                {
                    // we have just jumped a gap so now we want to start drawing again
                    drawing = true;
                }

                // remember the connection point that the connector started from
                last_connection_point_id = connection_point_id;

                // done with this connection
                points_on_outline_connection_point_index.erase(points_on_outline_connection_point_index.begin() + nearest_connection_point_index);

                // decrement total amount of work to do
                --connections_remaining;
            }
        }

        // we have now visited all the points in the outline, if a connector was (potentially) being drawn
        // draw the connector between the last and first connections

        if (drawing && first_connection_index != std::numeric_limits<unsigned>::max())
        {
            // output the connector line segments from the last connection to the first point in the outline
            connector_points.push_back(outline_poly[0]);
            for (unsigned pi = 1; pi < connector_points.size(); ++pi)
            {
                result.addLine(connector_points[pi - 1], connector_points[pi]);
            }
            // output the connector line segments from the first point in the outline to the first connection
            for (unsigned pi = 1; pi < path_to_first_connection.size(); ++pi)
            {
                result.addLine(path_to_first_connection[pi - 1], path_to_first_connection[pi]);
            }
        }

        if (connections_remaining < 1)
        {
            break;
        }
    }
}

} // namespace cura