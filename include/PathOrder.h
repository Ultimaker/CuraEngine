//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDER_H
#define PATHORDER_H

#include "PathOrdering.h"
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
     * After reordering, this contains the paths that need to be printed in the
     * correct order.
     *
     * Each path contains the information necessary to print the paths: A
     * pointer to the vertex data, whether to close the loop or not, the
     * direction in which to print the path and where to start the path.
     */
    std::vector<PathOrdering<PathType>> paths;

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
     * In the current set of paths, detect all loops and mark them as such.
     *
     * This will go through all polylines in the paths. If the endpoints of
     * these polylines are close enough together, it considers them a polygon.
     * It will then mark these polylines as being polygons for the future.
     */
    void detectLoops()
    {
        for(PathOrdering<PathType>& path : paths)
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