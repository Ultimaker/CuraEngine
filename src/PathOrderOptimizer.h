//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDEROPTIMIZER_H
#define PATHORDEROPTIMIZER_H

#include <stdint.h>
#include <utility>
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
    ZSeamConfig()
    : type(EZSeamType::SHORTEST)
    , pos(Point(0, 0))
    , corner_pref(EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
    {
    }

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, when using the sharpest corner
     * strategy.
     */
    ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref)
    : type(type)
    , pos(pos)
    , corner_pref(corner_pref)
    {
    }
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
 * \tparam PathType The type of paths that will be optimized by this optimizer.
 * Make sure that a specialization is available for \ref get_vertex and
 * \ref get_size in order for the optimizer to know how to read information from
 * your path.
 */
template<typename PathType>
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
        Path(const PathType& vertices, const bool is_closed = false, const size_t start_vertex = 0, const bool backwards = false)
        : vertices(vertices)
        , start_vertex(start_vertex)
        , is_closed(is_closed)
        , backwards(backwards)
        {
            std::cout << "++++++++++++++++++++++ constructing path. Vertices = " << &(this->vertices) << std::endl;
        }

        /*!
         * The vertex data of the path.
         */
        const PathType vertices;

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

        /*!
         * Swap position with another path.
         *
         * This is required in order to use std::swap on the paths, or to use
         * algorithms like std::reverse.
         */
        void swap(Path& other)
        {
            std::swap(vertices, other.vertices);
            std::swap(start_vertex, other.start_vertex);
            std::swap(is_closed, other.is_closed);
            std::swap(backwards, other.backwards);
        }
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
     * This doesn't actually optimize the order yet, so the ``paths`` field will
     * not be filled yet.
     * \param start_point The location where the nozzle is assumed to start from
     * before printing these parts.
     * \param config Seam settings.
     * \param combing_boundary Boundary to avoid when making travel moves.
     */
    PathOrderOptimizer(const Point start_point, const ZSeamConfig config = ZSeamConfig(), const Polygons* combing_boundary = nullptr)
    : start_point(start_point)
    , config(config)
    , combing_boundary((combing_boundary != nullptr && combing_boundary->size() > 0) ? combing_boundary : nullptr)
    {
    }

    /*!
     * Add a new polygon to be optimized.
     * \param polygon The polygon to optimize.
     */
    void addPolygon(const PathType& polygon)
    {
        constexpr bool is_closed = true;
        paths.emplace_back(polygon, is_closed);
    }

    /*!
     * Add a new polyline to be optimized.
     * \param polyline The polyline to optimize.
     */
    void addPolyline(const PathType& polyline)
    {
        std::cout << "++++++ got a polyline to add: " << &polyline << std::endl;
        paths.emplace_back(polyline);
    }

    /*!
     * Perform the calculations to optimize the order of the parts.
     *
     * This reorders the \ref paths field and fills their starting vertices and
     * directions.
     */
    void optimize()
    {
        ConstPolygonRef vertices = getVertexData(paths[0].vertices); //Placeholder to catch compilation errors.
        //TODO: Implement optimization!
    }

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
     * Some input data structures need to be converted to polygons before use.
     * For those, we need to store the vertex data somewhere during the lifetime
     * of the object. Store them here.
     *
     * For example, if the ``PathType`` is a list of ``ExtrusionJunction``s,
     * this will store the coordinates of those junctions.
     */
    std::vector<Polygon> cached_vertices;

    /*!
     * Find the vertex of a polygon that is closest to another point.
     * \param prev The point that the vertex must be close to.
     * \param i_polygon The index of the polygon in the \ref polygons field of
     * which to find a vertex.
     * \return An index to a vertex in that polygon.
     */
    size_t getClosestPointInPolygon(const Point prev, const size_t i_polygon) const
    {
        return 0; //TODO: Reimplement with template code.
    }

    /*!
     * Get a random vertex of a polygon.
     * \param poly_idx The index of the polygon in the \ref polygons field of
     * which to find a vertex.
     * \return A random index in that polygon.
     */
    size_t getRandomPointInPolygon(const size_t poly_idx) const
    {
        return 0; //TODO: Reimplement with template code.
    }

    /*!
     * Get vertex data from the custom path type.
     *
     * This is a function that allows the optimization algorithm to work with
     * any type of input data structure. It provides a translation from the
     * input data structure that the user would like to have ordered to a data
     * structure that the optimization algorithm can work with. It's unknown how
     * the ``PathType`` object is structured or how to get the vertex data from
     * it. This function tells the optimizer how, but it needs to be specialized
     * for each different type that this optimizer is used. See the .cpp file
     * for examples and where to add a new specialization.
     */
    ConstPolygonRef getVertexData(const PathType path);
};

} //namespace cura

#endif //PATHORDEROPTIMIZER_H
