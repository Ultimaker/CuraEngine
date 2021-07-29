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

    PathOrderMonotonic(const AngleRadians monotonic_direction, const Point start_point)
    : monotonic_vector(std::cos(monotonic_direction) * 1000, std::sin(monotonic_direction) * 1000)
    {
        this->start_point = start_point;
    }

    void optimize()
    {
        if(this->paths.empty())
        {
            return;
        }

        //Get the vertex data and store it in the paths.
        for(Path& path : this->paths)
        {
            path.converted = this->getVertexData(path.vertices);
        }

        std::vector<Path> reordered; //To store the result in. At the end, we'll std::swap with the real paths.
        reordered.reserve(this->paths.size());

        //First print all the looping polygons, if there are any.
        std::vector<Path*> polylines; //Also find all polylines and store them in a vector that we can sort in-place without making copies all the time.
        this->detectLoops(); //Always filter out loops. We don't specifically want to print those in monotonic order.
        for(Path& path : this->paths)
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
        std::unordered_set<Path*> starting_lines; //Starting points of a linearly connected segment.
        starting_lines.insert(polylines.begin(), polylines.end());
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
                /*There are 5 possible cases of overlapping:
                - We are behind them, partially overlapping. my_start is between their_start and their_end.
                - We are in front of them, partially overlapping. my_end is between their_start and their_end.
                - We are a smaller line, they completely overlap us. Both my_start and my_end are between their_start and their_end. (Caught with the first 2 conditions already.)
                - We are a bigger line, and completely overlap them. Both their_start and their_end are between my_start and my_end.
                - Lines are exactly equal. Start and end are the same. (Caught with the previous condition too.)*/
                if(    (my_start > their_start && my_start < their_end)
                    || (my_end > their_start   && my_end < their_end)
                    || (their_start >= my_start && their_end <= my_end))
                {
                    const auto is_unconnected = unconnected_polylines.find(*overlapping_line);
                    if(is_unconnected == unconnected_polylines.end()) //It was already connected to another line.
                    {
                        starting_lines.insert(*overlapping_line); //The overlapping line is a junction where two segments come together. Make it possible to start from there.
                    }
                    else
                    {
                        connections.emplace(*polyline_it, *overlapping_line);
                        unconnected_polylines.erase(is_unconnected); //The overlapping line is now connected.
                        starting_lines.erase(*overlapping_line); //If it was a starting line, it is no longer. It is now a later line in a sequence.
                    }
                    break;
                }
            }
        }

        //Order the starting points of each segments monotonically. This is the order in which to print each segment.
        std::vector<Path*> starting_lines_monotonic;
        starting_lines_monotonic.resize(starting_lines.size());
        std::partial_sort_copy(starting_lines.begin(), starting_lines.end(), starting_lines_monotonic.begin(), starting_lines_monotonic.end(), [this](Path* a, Path* b) {
            const coord_t a_start_projection = dot(a->converted->front(), monotonic_vector);
            const coord_t a_end_projection = dot(a->converted->back(), monotonic_vector);
            const coord_t a_projection = std::min(a_start_projection, a_end_projection); //The projection of a path is the endpoint furthest back of the two endpoints.

            const coord_t b_start_projection = dot(b->converted->front(), monotonic_vector);
            const coord_t b_end_projection = dot(b->converted->back(), monotonic_vector);
            const coord_t b_projection = std::min(b_start_projection, b_end_projection);

            return a_projection < b_projection;
        });

        //Now that we have the segments of overlapping lines, and know in which order to print the segments, print segments in monotonic order.
        Point current_pos = this->start_point;
        for(Path* line : starting_lines_monotonic)
        {
            optimizeClosestStartPoint(*line, current_pos);
            reordered.push_back(*line); //Plan the start of the sequence to be printed next!
            auto connection = connections.find(line);
            while(connection != connections.end() && starting_lines.find(connection->second) == starting_lines.end()) //Stop if the sequence ends or if we hit another starting point.
            {
                line = connection->second;
                optimizeClosestStartPoint(*line, current_pos);
                reordered.push_back(*line); //Plan this line in, to be printed next!
                connection = connections.find(line);
            }
        }

        std::swap(reordered, this->paths); //Store the resulting list in the main paths.
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

    /*!
     * For a given path, make sure that it is configured correctly to start
     * printing from the best endpoint.
     *
     * This changes the path's ``start_vertex`` and ``backwards`` fields, and
     * also adjusts the \ref current_pos in-place.
     *
     * Will cause a crash if given a path with 0 vertices!
     * \param path The path to adjust the start and direction parameters for.
     * \param current_pos The last position of the nozzle before printing this
     * path.
     */
    void optimizeClosestStartPoint(Path& path, Point& current_pos)
    {
        const coord_t dist_start = vSize2(current_pos - path.converted->front());
        const coord_t dist_end = vSize2(current_pos - path.converted->back());
        if(dist_start < dist_end)
        {
            path.start_vertex = 0;
            path.backwards = false;
            current_pos = path.converted->back();
        }
        else
        {
            path.start_vertex = path.converted->size() - 1;
            path.backwards = true;
            current_pos = path.converted->front();
        }
    }
};

}

#endif //PATHORDERMONOTONIC_H