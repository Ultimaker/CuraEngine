//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TPMSInfill.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

#include <unordered_map>

namespace cura {

void TPMSInfillSchwarzD::generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step)
{
    // generate Schwarz D "Primitive" surface defined by equation: sin(x) sin(y) sin(z) + sin(x) cos(y) cos(z) + cos(x) sin(y) cos(z) + cos(x) cos(y) sin(z) = 0
    // see https://en.wikipedia.org/wiki/Schwarz_minimal_surface

    const double cos_z = std::cos(M_PI * z / pitch);
    const double sin_z = std::sin(M_PI * z / pitch);

    std::vector<coord_t> x_coords;
    std::vector<coord_t> y_coords;

    double x_inc = step / 4.0;

    for (double x = 0; x <= pitch; x += x_inc)
    {
        const double x_rads = M_PI * x / pitch;
        const double sin_x = std::sin(x_rads);
        const double cos_x = std::cos(x_rads);

        const double A = sin_x * sin_z;
        const double B = cos_x * cos_z;
        const double C = sin_x * cos_z;
        const double D = cos_x * sin_z;
        const double y_rads = std::atan(-(C + D) / (A + B));

        x_coords.push_back(x);
        y_coords.push_back(y_rads / M_PI * pitch);
    }

    // repeat the lines over the infill area
    const unsigned num_coords = x_coords.size();
    for (coord_t x = x_min; x < x_max; x += pitch)
    {
        for (coord_t y = y_min; y < y_max + pitch; y += pitch)
        {
            bool is_first_point = true;
            Point last;
            bool last_inside = false;
            for (unsigned i = 0; i < num_coords; ++i)
            {
                coord_t y_coord = y_coords[i];
                unsigned last_i = (i + num_coords - 1) % num_coords;
                bool discontinuity = false;
                if (std::abs(y_coords[last_i] - y_coords[i]) > pitch/2)
                {
                    y_coord += (y_coords[last_i] < y_coords[i]) ? -pitch : pitch;
                    discontinuity = true;
                }
                Point current(x + x_coords[i], y + y_coord);
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
                        if (line.size() > 0)
                        {
                            // some of the line is inside the boundary
                            result.addLine(line[0][0], line[0][1]);
                            if (zig_zaggify)
                            {
                                Point connection_point = line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1];
                                connection_points.push_back(connection_point);
                                ConnectionId connection_id = 0;
                                const int col = (connection_point.X - x_min) / pitch;
                                const int row = (connection_point.Y - y_min) / pitch;
                                if (std::abs(cos_z) >= 0.707)
                                {
                                    // sloping backwards (\\)
                                    connection_id = -col - row;
                                }
                                else
                                {
                                    // sloping forwards (//)
                                    connection_id = col - row;
                                }
                                connection_ids.push_back(connection_id);
                            }
                        }
                        else
                        {
                            // none of the line is inside the boundary so the point that's actually on the boundary
                            // is the chain end
                            if (zig_zaggify)
                            {
                                Point connection_point = (last_inside) ? last : current;
                                connection_points.push_back(connection_point);
                                ConnectionId connection_id = 0;
                                const int col = (connection_point.X - x_min) / pitch;
                                const int row = (connection_point.Y - y_min) / pitch;
                                if (std::abs(cos_z) >= 0.707)
                                {
                                    // sloping backwards (\\)
                                    connection_id = -col - row;
                                }
                                else
                                {
                                    // sloping forwards (//)
                                    connection_id = col - row;
                                }
                                connection_ids.push_back(connection_id);
                            }
                        }
                    }
                }
                if (discontinuity)
                {
                    last = rotate_around_origin(Point(x + x_coords[i], y + y_coords[i]), fill_angle_rads);
                    last_inside = outline.inside(last, true);
                }
                else
                {
                    last = current;
                    last_inside = current_inside;
                }
                is_first_point = false;
            }
        }
    }
}

void TPMSInfillSchwarzD::generateConnections(Polygons& result, const Polygons& outline)
{
    // zig-zaggification consists of simply joining alternate connection ends with lines that follow the outline

    int connections_remaining = connection_points.size();

    if (connections_remaining == 0)
    {
        return;
    }

    for (ConstPolygonRef outline_poly : outline)
    {
        std::vector<Point> connector_points; // the points that make up a connector line

        // we need to remember the first connection processed and the path to it from the first outline point
        // so that later we can possibly connect to it from the last connection processed
        ConnectionId first_connection_id = std::numeric_limits<ConnectionId>::max();
        std::vector<Point> path_to_first_connection;

        ConnectionId last_point_id = std::numeric_limits<ConnectionId>::max();

        std::unordered_map<ConnectionId, std::vector<ConnectionId>> connections_to;
        std::unordered_map<ConnectionId, std::vector<ConnectionId>> connections_from;

        bool drawing = false; // true when a connector line is being (potentially) created

        Point cur_point; // current point of interest - either an outline point or a connection point

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

                if (first_connection_id == std::numeric_limits<ConnectionId>::max())
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

                // make the connection point the current point and add it to the connector line
                cur_point = connection_points[points_on_outline_connection_point_index[nearest_connection_point_index]];

                if (drawing && connector_points.size() > 0 && vSize2(cur_point - connector_points.back()) < 100)
                {
                    // this connection point will be too close to the last connector point so throw away the last connector point
                    connector_points.pop_back();
                }
                connector_points.push_back(cur_point);

                const ConnectionId this_point_id = connection_ids[points_on_outline_connection_point_index[nearest_connection_point_index]];

                if (first_connection_id == std::numeric_limits<ConnectionId>::max())
                {
                    // this is the first connection to be processed, remember it
                    first_connection_id = this_point_id;
                    path_to_first_connection.push_back(cur_point);
                }

                if (drawing)
                {
                    std::vector<ConnectionId>& from = connections_from[this_point_id];
                    std::vector<ConnectionId>& to = connections_to[this_point_id];
                    if (this_point_id != last_point_id && std::find(from.begin(), from.end(), last_point_id) == from.end() && std::find(to.begin(), to.end(), last_point_id) == to.end())
                    {
                        for (unsigned pi = 1; pi < connector_points.size(); ++pi)
                        {
                            result.addLine(connector_points[pi - 1], connector_points[pi]);
                        }
                        drawing = false;
                        connector_points.clear();

                        from.push_back(last_point_id);
                        connections_to[last_point_id].push_back(this_point_id);
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

                last_point_id = this_point_id;

                // done with this connection point
                points_on_outline_connection_point_index.erase(points_on_outline_connection_point_index.begin() + nearest_connection_point_index);

                // decrement total amount of work to do
                --connections_remaining;
            }
        }

        // we have now visited all the points in the outline, if a connector was (potentially) being drawn
        // draw the connector between the last and first connections

        if (drawing && first_connection_id != std::numeric_limits<ConnectionId>::max() && first_connection_id != last_point_id)
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