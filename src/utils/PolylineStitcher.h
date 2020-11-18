//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POLYLINE_STITCHER_H
#define UTILS_POLYLINE_STITCHER_H

#include "polygon.h"

namespace cura
{

/*!
 * Class for stitching polylines into longer polylines or into polygons
 */
class PolylineStitcher
{
public:
    /*!
     * Stitch together the separate \p lines into \p result_lines and if they can be closed into \p result_polygons.
     * Only introduce new segments shorter than \p max_stitch_distance,
     * but always try to make the shortest segment possible,
     * unless the segment is smaller than \p snap_distance.
     * When the first connection smaller than \p snap_distance is found no other nearby connections will be considered.
     * 
     * Only stitch polylines into closed polygons if they are larger than 3 * \p max_stitch_distance,
     * in order to prevent small segments to accidentally get closed into a polygon.
     * 
     * \note Resulting polylines and polygons are added onto the existing containers, so you can directly output onto a polygons container with existing polygons in it.
     * Hewever, you shouldn't call this function with the same parameter in \p lines as \p result_lines, because that would duplicate (some of) the polylines.
     */
    static void stitch(const Polygons& lines, Polygons& result_lines, Polygons& result_polygons, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10);
};

}//namespace cura
#endif//UTILS_POLYLINE_STITCHER_H

