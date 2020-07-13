//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <stdint.h>
#include "settings/EnumSettings.h"
#include "utils/polygon.h"
#include "utils/polygonUtils.h"

namespace cura {

/*!
 * Helper class that encapsulates the various criteria that define the location
 * of the z-seam.
 * Instances of this are passed to the PathOrderOptimizer to specify where the
 * seam is to be located.
 */
struct ZSeamConfig
{
    /*!
     * Strategy to place the seam (user-specified, shortest distance, sharpest
     * corner, etc.).
     */
    EZSeamType type;

    /*!
     * When using a user-specified position for the seam, this is the position
     * that the user specified.
     */
    Point pos;

    /*!
     * Corner preference type, if using the sharpest corner strategy.
     */
    EZSeamCornerPrefType corner_pref;

    /*!
     * Default constructor for use when memory must be allocated before it gets
     * filled (like with some data structures).
     *
     * This will select the "shortest" seam strategy.
     */
    ZSeamConfig();

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, when using the sharpest corner
     * strategy.
     */
    ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref);
};

/*!
 * Path order optimization class.
 * 
 * Utility class for optimizing the order in which things are printed, by
 * minimizing the distance traveled between different items to be printed. For
 * each item to be printed, it also chooses a starting point as to where on the
 * polygon or polyline to start printing, and determines which direction to
 * print in.
 *
 * To use this class, first create an instance and provide some parameters as
 * metadata. Then add polygons and polylines to the class. Then call the
 * \ref optimize function to compute the optimization. Finally, print the
 * polygons and polylines in the \ref paths field in the order in which they are
 * given.
 *
 * In the output of this class, polylines and polygons are combined into a
 * single vector: \ref paths . Each path contains a pointer to the original
 * polygon data, as well as whether that data represented a polygon or a
 * polyline, which direction to print that path in, and where to start along the
 * path.
 *
 * The optimizer will always start a polyline from either end, never halfway.
 * The Z seam is not used for these.
 */
class PathOrderOptimizer
{
public:
    /*!
     * Represents a path which has been optimized, the output of the
     * optimization.
     *
     * This small data structure contains the vertex data of a path,  where to
     * start along the path and in which direction to print it, as well as
     * whether the path should be closed (in case of a polygon) or open (in case
     * of a polyline.
     *
     * After optimization is completed, the \ref paths vector will be filled
     * with optimized paths.
     */
    struct Path
    {
        /*!
         * Construct a new path.
         */
        Path(const ConstPolygonPointer vertices, const bool is_closed = false, const size_t start_vertex = 0, const bool backwards = false);

        /*!
         * The vertex data of the path.
         */
        ConstPolygonPointer vertices;

        /*!
         * Which vertex along the path to start printing with.
         *
         * If this path represents a polyline, this will always be one of the
         * endpoints of the path; either 0 or ``vertices->size() - 1``.
         */
        size_t start_vertex;

        /*!
         * Whether the path should be closed at the ends or not.
         *
         * If this path should be closed, it represents a polygon. If it should
         * not be closed, it represents a polyline.
         */
        bool is_closed;

        /*!
         * Whether the path should be traversed in backwards direction.
         *
         * For a polyline it may be more efficient to print the path in
         * backwards direction, if the last vertex is closer than the first.
         */
        bool backwards;
    };

    /*!
     * The location where the nozzle is assumed to start from before printing
     * these parts.
     */
    Point start_point;

    /*!
     * Seam settings.
     */
    const ZSeamConfig config;

    /*!
     * After optimizing, this contains the paths that need to be printed in the
     * correct order.
     *
     * Each path contains the information necessary to print the parts: A
     * pointer to the vertex data, whether or not to close the loop, the
     * direction in which to print the path and where to start the path.
     */
    std::vector<Path> paths;

    /*!
     * Construct a new optimizer.
     *
     * This doesn't actually optimize the order yet, so the ``poly_order`` and
     * ``poly_start`` fields will not be filled yet.
     * \param start_point The location where the nozzle is assumed to start from
     * before printing these parts.
     * \param config Seam settings.
     * \param combing_boundary Boundary to avoid when making travel moves.
     */
    PathOrderOptimizer(const Point start_point, const ZSeamConfig config = ZSeamConfig(), const Polygons* combing_boundary = nullptr);

    /*!
     * Add a new polygon to be optimized.
     * \param polygon The polygon to optimize.
     */
    void addPolygon(const PolygonRef& polygon);

    /*!
     * Add a new polygon to be optimized.
     * \param polygon The polygon to optimize.
     */
    void addPolygon(const ConstPolygonRef& polygon);

    /*!
     * Add a complex polygon to be optimized.
     *
     * Each contour of this complex polygon will be optimized separately, so it
     * could be that the order of these polygons will not cause the contours of
     * a complex polygon to be printed together.
     * \param polygons The complex polygon to optimize.
     */
    void addPolygons(const Polygons& polygons);

    /*!
     * Add a new polyline to be optimized.
     * \param polyline The polyline to optimize.
     */
    void addPolyline(const PolygonRef& polyline);

    /*!
     * Add a new polyline to be optimized.
     * \param polyline The polyline to optimize.
     */
    void addPolyline(const ConstPolygonRef& polyline);

    /*!
     * Add a set of polylines to be optimized.
     *
     * A shorthand, if you've already got this packed in a ``Polygons``
     * instance.
     * \param polylines The polylines to optimize.
     */
    void addPolylines(const Polygons& polylines);

    /*!
     * Perform the calculations to optimize the order of the parts.
     *
     * This sets the \ref poly_start and \ref poly_order fields. They will then
     * refer by index to the polygons in the \ref polygons field.
     */
    void optimize();

protected:
    /*!
     * Hash map storing where each line is.
     *
     * This allows us to quickly find any nearby other lines.
     */
    LocToLineGrid* loc_to_line;

    /*!
     * Boundary to avoid when making travel moves.
     */
    const Polygons* combing_boundary;

    /*!
     * Find the vertex of a polygon that is closest to another point.
     * \param prev The point that the vertex must be close to.
     * \param i_polygon The index of the polygon in the \ref polygons field of
     * which to find a vertex.
     * \return An index to a vertex in that polygon.
     */
    size_t getClosestPointInPolygon(const Point prev, const size_t i_polygon) const;

    /*!
     * Get a random vertex of a polygon.
     * \param poly_idx The index of the polygon in the \ref polygons field of
     * which to find a vertex.
     * \return A random index in that polygon.
     */
    size_t getRandomPointInPolygon(const size_t poly_idx) const;
};

} //namespace cura

#endif //PATHOPTIMIZER_H
