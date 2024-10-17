// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATH_ORDER_PATH_H
#define PATH_ORDER_PATH_H

#include <optional>

#include "settings/ZSeamConfig.h" //To get the seam settings.
#include "utils/polygonUtils.h"

namespace cura
{

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
template<typename PathType>
struct PathOrdering
{
    /*!
     * Construct a new planned path.
     *
     * The \ref converted field is not initialized yet. This can only be
     * done after all of the input paths have been added, to prevent
     * invalidating the pointers.
     */
    PathOrdering(const PathType& vertices, const bool is_closed = false, const size_t start_vertex = 0, const bool backwards = false)
        : vertices_(vertices)
        , start_vertex_(start_vertex)
        , is_closed_(is_closed)
        , backwards_(backwards)
    {
    }

    /*!
     * The vertex data of the path.
     */
    PathType vertices_;

    /*!
     * Vertex data, converted into a Polygon so that the orderer knows how
     * to deal with this data.
     */
    const PointsSet* converted_{ nullptr };

    /*!
     * Which vertex along the path to start printing with.
     *
     * If this path represents a polyline, this will always be one of the
     * endpoints of the path; either 0 or ``vertices->size() - 1``.
     */
    size_t start_vertex_;

    /*!
     * Whether the path should be closed at the ends or not.
     *
     * If this path should be closed, it represents a polygon. If it should
     * not be closed, it represents a polyline.
     */
    bool is_closed_;

    /*!
     * Whether the path should be traversed in backwards direction.
     *
     * For a polyline it may be more efficient to print the path in
     * backwards direction, if the last vertex is closer than the first.
     */
    bool backwards_;

    /*!
     * Force the start point of the path to be at a specific location.
     * Will only happen if not empty, and this point is actually on the path.
     */
    std::optional<size_t> force_start_index_;

    /*!
     * The start point calculation strategy to be used for this path
     */
    ZSeamConfig seam_config_;

    /*!
     * Indicates whether this path is an outer (or inner) wall
     */
    bool is_outer_wall{ false };

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
    const PointsSet& getVertexData();

protected:
    /*!
     * Some input data structures need to be converted to polygons before use.
     * For those, we need to store the vertex data somewhere during the lifetime
     * of the object. Store them here.
     *
     * For example, if the ``PathType`` is a list of ``ExtrusionJunction``s,
     * this will store the coordinates of those junctions.
     */
    std::optional<PointsSet> cached_vertices_;
};

} // namespace cura

#endif // PATH_ORDER_PATH_H
