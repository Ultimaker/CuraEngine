//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDERMONOTONIC_H
#define PATHORDERMONOTONIC_H

#include <cmath> //For std::sin() and std::cos().
#include <unordered_set> //To track starting points of monotonic sequences.
#include <unordered_map> //To track monotonic sequences.

#include "PathOrder.h"

namespace cura
{

/*!
 * Class that orders paths monotonically.
 *
 * This is a utility class that changes the order in which things are printed,
 * to ensure that they are printed in the same major direction. Printing
 * adjacent lines in the same direction ensures that they layer on top of each
 * other in the same way. That helps to make the entire surface look consistent.
 *
 * To use this class, first create an instance and provide some parameters as
 * metadata. Then add polygons and polylines to the class. Then call the
 * \ref optimize function to compute the order. Finally, print the polygons and
 * polylines in the \ref paths field in the order in which they are given.
 *
 * In the output of this class, polylines and polygons are combined into a
 * single list: \ref paths . Each path contains a pointer to the original
 * polygon data, as well as whether that data represented a polygon or a
 * polyline, which direction to print the path in, and where to start along the
 * path.
 *
 * The monotonic order does not use the Z seam settings. It is meant to apply
 * only to polylines. If given polygons, it will place the seam in the location
 * closest to the source direction of the monotonicity vector.
 */
template<typename PathType>
class PathOrderMonotonic : public PathOrder<PathType>
{
public:
    using typename PathOrder<PathType>::Path;
    using PathOrder<PathType>::paths;
    using PathOrder<PathType>::detectLoops;
    using PathOrder<PathType>::getVertexData;

    PathOrderMonotonic(const AngleRadians monotonic_direction)
    : monotonic_vector(std::cos(monotonic_direction) * 1000, std::sin(monotonic_direction) * 1000)
    {}

    void optimize()
    {
        if(paths.empty())
        {
            return;
        }

        //Get the vertex data and store it in the paths.
        for(Path& path : paths)
        {
            path.converted = getVertexData(path.vertices);
        }

        std::vector<Path> reordered; //To store the result in. At the end, we'll std::swap with the real paths.
        reordered.reserve(paths.size());

        //First print all the looping polygons, if there are any.
        std::vector<Path*> polylines; //Also find all polylines and store them in a vector that we can sort in-place without making copies all the time.
        detectLoops(); //Always filter out loops. We don't specifically want to print those in monotonic order.
        for(Path& path : paths)
        {
            if(path.is_closed || path.vertices.size() <= 1)
            {
                reordered.push_back(path);
            }
            else
            {
                polylines.push_back(&path);
            }
        }

        //Sort the polylines by their projection on the monotonic vector.
        std::sort(polylines.begin(), polylines.end(), [this](Path* a, Path* b) {
            const coord_t a_start_projection = dot(a->converted->front(), monotonic_vector);
            const coord_t a_end_projection = dot(a->converted->back(), monotonic_vector);
            const coord_t a_projection = std::min(a_start_projection, a_end_projection); //The projection of a path is the endpoint furthest back of the two endpoints.

            const coord_t b_start_projection = dot(b->converted->front(), monotonic_vector);
            const coord_t b_end_projection = dot(b->converted->back(), monotonic_vector);
            const coord_t b_projection = std::min(b_start_projection, b_end_projection);

            return a_projection < b_projection;
        });

        //Find out which lines overlap with which adjacent lines, when projected perpendicularly to the monotonic vector.
        //Create a DAG structure of overlapping lines.
        const Point perpendicular = turn90CCW(monotonic_vector);

        std::unordered_set<Path*> unconnected_polylines; //Polylines that haven't been overlapped yet by a previous line.
        unconnected_polylines.insert(polylines.begin(), polylines.end());
        std::unordered_map<Path*, Path*> connections; //For each polyline, which polyline it overlaps with, closest in the projected order.

        for(auto polyline_it = polylines.begin(); polyline_it != polylines.end(); polyline_it++)
        {
            coord_t my_start = dot((*polyline_it)->converted->front(), perpendicular);
            coord_t my_end = dot((*polyline_it)->converted->back(), perpendicular);
            if(my_start > my_end)
            {
                std::swap(my_start, my_end);
            }
            //Find the next polyline this line overlaps with.
            //Lines are already sorted, so the first overlapping line we encounter is the next closest line to overlap with.
            for(auto overlapping_line = polyline_it + 1; overlapping_line != polylines.end(); overlapping_line++)
            {
                //Does this one overlap?
                coord_t their_start = dot((*overlapping_line)->converted->front(), perpendicular);
                coord_t their_end = dot((*overlapping_line)->converted->back(), perpendicular);
                if(their_start > their_end)
                {
                    std::swap(their_start, their_end);
                }
                if(    (my_start > their_start && my_start < their_end) //It overlaps if any endpoint is between the endpoints of the other line.
                    || (my_end > their_start   && my_end < their_end)
                    || (their_start > my_start && their_start < my_end)
                    || (their_end > my_start   && their_end < my_end))
                {
                    connections.emplace(*polyline_it, *overlapping_line);
                    const auto is_unconnected = unconnected_polylines.find(*overlapping_line);
                    if(is_unconnected != unconnected_polylines.end())
                    {
                        unconnected_polylines.erase(is_unconnected); //The overlapping line is now connected.
                    }
                    break;
                }
            }
        }

        //Now that we know which lines overlap with which other lines, iterate over them again to print connected lines in order.
        for(Path* polyline : polylines)
        {
            if(unconnected_polylines.find(polyline) == unconnected_polylines.end()) //Polyline is reached through another line.
            {
                continue;
            }
            reordered.push_back(*polyline); //Add the start of the connected sequence.
            auto connection = connections.find(polyline);
            while(connection != connections.end())
            {
                polyline = connection->second;
                reordered.push_back(*polyline);
                connection = connections.find(polyline);
            }
        }

        std::swap(reordered, paths); //Store the resulting list in the main paths.
    }

protected:
    /*!
     * The direction in which to print montonically, encoded as vector of length
     * 1000.
     *
     * The resulting ordering will cause clusters of paths to be sorted
     * according to their projection on this vector.
     */
    Point monotonic_vector;
};

}

#endif //PATHORDERMONOTONIC_H