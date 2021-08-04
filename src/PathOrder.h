//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDER_H
#define PATHORDER_H

#include "settings/ZSeamConfig.h" //To get the seam settings.
#include "utils/polygonUtils.h"

namespace cura
{

/*!
 * Parent class of all path ordering techniques.
 *
 * This path ordering provides a virtual interface for the methods that path
 * ordering techniques should implement in order to provide a consistent API to
 * allow interchanging the techniques used to optimise the printing order.
 *
 * It also provides some base members that can be used by all path ordering
 * techniques, to reduce code duplication.
 * \param PathType The type of paths to optimise. This class can reorder any
 * type of paths as long as an overload of ``getVertexData`` exists to convert
 * that type into a list of vertices.
 */
template<typename PathType>
class PathOrder
{
public:
    /*!
     * Represents a path which has been optimised, the output of the ordering.
     *
     * This small data structure contains the vertex data of a path, where to
     * start along the path and in which direction to print it, as well as
     * whether the path should be closed (in the case of a polygon) or open (in
     * case of a polyline).
     *
     * After the ordering has completed, the \ref paths vector will be filled
     * with optimized paths.
     */
    struct Path
    {
        /*!
         * Construct a new planned path.
         *
         * The \ref converted field is not initialized yet. This can only be
         * done after all of the input paths have been added, to prevent
         * invalidating the pointers.
         */
        Path(const PathType& vertices, const bool is_closed = false, const size_t start_vertex = 0, const bool backwards = false)
            : vertices(vertices)
            , start_vertex(start_vertex)
            , is_closed(is_closed)
            , backwards(backwards)
        {}

        /*!
         * The vertex data of the path.
         */
        PathType vertices;

        /*!
         * Vertex data, converted into a Polygon so that the orderer knows how
         * to deal with this data.
         */
        ConstPolygonPointer converted;

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
     * After reordering, this contains the paths that need to be printed in the
     * correct order.
     *
     * Each path contains the information necessary to print the paths: A
     * pointer to the vertex data, whether to close the loop or not, the
     * direction in which to print the path and where to start the path.
     */
    std::vector<Path> paths;

    /*!
     * The location where the nozzle is assumed to start from before printing
     * these parts.
     */
    Point start_point;

    /*!
     * Seam settings.
     */
    ZSeamConfig seam_config;

    /*!
     * Add a new polygon to be planned.
     *
     * This will be interpreted as a closed polygon.
     * \param polygon The polygon to plan.
     */
    void addPolygon(const PathType& polygon)
    {
        constexpr bool is_closed = true;
        paths.emplace_back(polygon, is_closed);
    }

    /*!
     * Add a new polyline to be planned.
     *
     * This polyline will be interpreted as an open polyline, not a polygon.
     * \param polyline The polyline to plan.
     */
    void addPolyline(const PathType& polyline)
    {
        constexpr bool is_closed = false;
        paths.emplace_back(polyline, is_closed);
    }

    /*!
     * Call on this method to reorder all paths to an appropriate printing
     * order.
     *
     * Every ordering technique will need to implement this to produce the
     * correct order.
     *
     * The \ref paths vector will be edited by this function call to reorder it,
     * to adjust the starting positions of those paths and perhaps to mark those
     * paths as being printed backwards.
     */
    virtual void optimize() = 0;

protected:
    /*!
     * Two line endpoints are considered to be the same if they are within this
     * margin of error apart from each other.
     *
     * If endpoints are very close together, this will cause the ordering to
     * pretend they are the same point.
     * This is used for detecting loops and chaining lines together.
     */
    constexpr static coord_t coincident_point_distance = 10;

    /*!
     * Get vertex data from the custom path type.
     *
     * This is a function that allows the reordering algorithm to work with any
     * type of input data structure. It provides a translation from the input
     * data structure that the user would like to have reordered to a data
     * structure that the reordering algorithm can work with. It's unknown how
     * the ``PathType`` object is structured or how to get the vertex data from
     * it. This function tells the optimizer how, but it needs to be specialized
     * for each different type that this class is used with. See the .cpp file
     * for examples and where to add a new specialization.
     */
    ConstPolygonRef getVertexData(const PathType path);

    /*!
     * In the current set of paths, detect all loops and mark them as such.
     *
     * This will go through all polylines in the paths. If the endpoints of
     * these polylines are close enough together, it considers them a polygon.
     * It will then mark these polylines as being polygons for the future.
     */
    void detectLoops()
    {
        for(Path& path : paths)
        {
            if(path.is_closed) //Already a polygon. No need to detect loops.
            {
                continue;
            }
            if(path.converted->size() < 3) //Not enough vertices to really be a closed loop.
            {
                continue;
            }
            if(vSize2(path.converted->back() - path.converted->front()) < coincident_point_distance * coincident_point_distance)
            {
                //Endpoints are really close to one another. Consider it a closed loop.
                path.is_closed = true;
            }
        }
    }
};

}

#endif //PATHORDER_H