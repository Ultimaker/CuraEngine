//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDER_H
#define PATHORDER_H

#include "utils/polygonUtils.h"

/*!
 * Parent class of all path ordering techniques.
 *
 * This path ordering provides a virtual interface for the methods that path
 * ordering techniques should implement in order to provide a consistent API to
 * allow interchanging the techniques used to optimise the printing order.
 *
 * It also provides some base members that can be used by all path ordering
 * techniques, to reduce code duplication.
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
};

#endif //PATHORDER_H