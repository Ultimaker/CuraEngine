//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDERMONOTONIC_H
#define PATHORDERMONOTONIC_H

#include <cmath> //For std::sin() and std::cos().

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
    PathOrderMonotonic(const AngleRadians monotonic_direction)
    : monotonic_vector(std::cos(monotonic_direction) * 1000, std::sin(monotonic_direction) * 1000)
    {}

    void optimize()
    {
        //First print all the looping polygons, if there are any.
        detectLoops(); //Always filter out loops. We don't specifically want to print those in monotonic order.
        const auto polylines_start = std::partition(paths.begin(), paths.end(), [](const Path& path) {
            return path.is_closed;
        });
        //Now we only need to reorder paths from polylines_start to the end.
        //TODO.
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