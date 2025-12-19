// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDERMONOTONIC_H
#define PATHORDERMONOTONIC_H

#include <cmath> //For std::sin() and std::cos().
#include <deque>

#include "PathOrder.h"
#include "geometry/Point2D.h"
#include "path_ordering.h"

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
    using Path = PathOrdering<PathType>;
    using PathOrder<PathType>::coincident_point_distance_;

    PathOrderMonotonic(const AngleRadians& monotonic_direction, const coord_t max_adjacent_distance, const Point2LL& start_point, const bool interlaced = false)
        : PathOrder<PathType>(start_point)
        // The monotonic vector needs to rotate clockwise instead of counter-clockwise, the same as how the infill patterns are generated.
        , monotonic_vector_(-std::cos(monotonic_direction), std::sin(monotonic_direction))
        , max_adjacent_distance_(max_adjacent_distance)
        , interlaced_(interlaced)
    {
    }

    void optimize();

protected:
    /*!
     * The direction in which to print monotonically
     *
     * The resulting ordering will cause clusters of paths to be sorted
     * according to their projection on this vector.
     */
    Point2D monotonic_vector_;

    /*!
     * Maximum distance at which lines are considered to be adjacent.
     *
     * The monotonicity constraint is only held for lines that are closer than
     * this distance together.
     */
    coord_t max_adjacent_distance_;

    /*!
     * Interlaced monotonic will order lines in two passes, so that adjacent lines will never be printed just after each other
     */
    bool interlaced_{ false };

    // Divide by a precision factor before doing the rounding, so that we are sure that aligned lines will end up in the same bucket
    static constexpr double precision_factor{ 10.0 };

    /*!
     * Set the proper start vertex for the given path, taking care that it should be processed either forwards or backwards
     * @param path The path to be setup
     * @param backwards Indicates whether the path should be processed backwards (or forwards)
     */
    static void setStartVertex(Path* path, const bool backwards)
    {
        if (backwards)
        {
            path->start_vertex_ = path->converted_->size() - 1;
            path->backwards_ = true;
        }
        else
        {
            path->start_vertex_ = 0;
            path->backwards_ = false;
        }
    }

    /*!
     * For a given path, make sure that it is configured correctly to start
     * printing from the best endpoint.
     *
     * This changes the path's ``start_vertex`` and ``backwards`` fields, and
     * also adjusts the \ref current_pos in-place.
     *
     * If the path already had a ``start_vertex`` set, this will not be
     * adjusted. Only the ``current_pos`` will be set then.
     *
     * Will cause a crash if given a path with 0 vertices!
     * \param path The path to adjust the start and direction parameters for.
     * \param current_pos The last position of the nozzle before printing this
     * path.
     */
    static void optimizeClosestStartPoint(Path* path, Point2LL& current_pos)
    {
        if (path->start_vertex_ == path->converted_->size())
        {
            const coord_t dist_start = vSize2(current_pos - path->converted_->front());
            const coord_t dist_end = vSize2(current_pos - path->converted_->back());
            setStartVertex(path, dist_start >= dist_end);
        }
        current_pos = (*path->converted_)[path->converted_->size() - 1 - path->start_vertex_]; // Opposite of the start vertex.
    }

    /*!
     * Projects the given point along the given direction
     */
    static double projectToVector(const Point2LL& point, const Point2D& direction)
    {
        return Point2D::dot(Point2D(point.X, point.Y), direction);
    }

    /*!
     * Projects the given point along the monotonic direction
     */
    double projectToMonotonicVector(const Point2LL& point) const
    {
        return projectToVector(point, monotonic_vector_);
    }

    /*!
     * Some input contains line segments or polylines that are separate paths,
     * but are still intended to be printed as a long sequence. This function
     * finds such strings of polylines.
     * \param polyline Any polyline which may be part of a string of polylines.
     * \param line_bucket_grid A pre-computed bucket grid to allow quick look-up
     * of which vertices are nearby.
     * \return A list of polylines, in the order in which they should be
     * printed. All paths in this string already have their start_vertex set
     * correctly.
     */
    std::deque<Path*> findPolylineString(Path* polyline, const SparsePointGridInclusive<Path*>& line_bucket_grid);

    /*!
     * Get the endpoint of the polyline that is farthest away from the given
     * point.
     * \param polyline The polyline to get an endpoint of.
     * \param point The point to get far away from.
     * \return The vertex index of the endpoint that is farthest away.
     */
    static size_t getFarthestEndpoint(Path* polyline, const Point2LL& point)
    {
        const coord_t front_dist = vSize2(polyline->converted_->front() - point);
        const coord_t back_dist = vSize2(polyline->converted_->back() - point);
        if (front_dist < back_dist)
        {
            return polyline->converted_->size() - 1;
        }
        else
        {
            return 0;
        }
    }

    /*!
     * Find which lines are overlapping with a certain line.
     * \param polyline_it The line with which to find overlaps. Given as an
     * iterator into the sorted polylines list, to cut down on the search space.
     * If the lines don't have too much overlap, this should result in only a
     * handful of lines being searched at all.
     * \param perpendicular A vector perpendicular to the monotonic vector, pre-
     * calculated.
     * \param polylines The sorted list of polylines.
     * \param max_adjacent_distance The maximum distance for which to consider segments as adjacent
     */
    std::vector<Path*> getOverlappingLines(
        const typename std::vector<Path*>::const_iterator& polyline_it,
        const Point2D& perpendicular,
        const std::vector<Path*>& polylines,
        const coord_t max_adjacent_distance);

private:
    /*!
     * Predicate to check if a nearby path is okay for polylines to connect
     * with.
     *
     * It is okay if the endpoints are sufficiently close together, and the
     * polyline is not yet connected to a different string of polylines.
     * \param nearby_endpoint The endpoint of the current string of polylines.
     * We'll check if the candidate polyline is nearby enough.
     * \param found_path A candidate polyline, as found in the bucket grid. This
     * struct of the bucket grid contains not only the actual path (via pointer)
     * but also the endpoint of it that it found to be nearby.
     */
    static bool canConnectToPolyline(const Point2LL& nearby_endpoint, SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Path*> found_path)
    {
        return found_path.val->start_vertex_ == found_path.val->converted_->size() // Don't find any line already in the string.
            && vSize2(found_path.point - nearby_endpoint) < coincident_point_distance_ * coincident_point_distance_; // And only find close lines.
    }

    /*!
     * Order the given lines, according to the given adjacence distance
     * @param lines The lines to be ordered
     * @param max_adjacent_distance The maximum distance for which to consider segments as adjacent
     * @return The lines in monotonic order
     */
    std::vector<Path> makeOrderedPath(const std::vector<Path*>& lines, const coord_t max_adjacent_distance);
};

} // namespace cura

#endif // PATHORDERMONOTONIC_H
