// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POLYLINE_STITCHER_H
#define UTILS_POLYLINE_STITCHER_H

#include <cassert>

#include "SparsePointGrid.h"

namespace cura
{

template<class T>
class PathsPointIndex;

/*!
 * Class for stitching polylines into longer polylines or into polygons
 */
template<typename InputPaths, typename OutputPaths, typename Path, typename Junction>
class PolylineStitcher
{
public:
    /*!
     * Stitch together the separate \p lines into \p result_lines and if they
     * can be closed into \p result_polygons.
     *
     * Only introduce new segments shorter than \p max_stitch_distance, and
     * larger than \p snap_distance but always try to take the shortest
     * connection possible.
     *
     * Only stitch polylines into closed polygons if they are larger than 3 *
     * \p max_stitch_distance, in order to prevent small segments to
     * accidentally get closed into a polygon.
     *
     * \warning Tiny polylines (smaller than 3 * max_stitch_distance) will not
     * be closed into polygons.
     *
     * \note Resulting polylines and polygons are added onto the existing
     * containers, so you can directly output onto a polygons container with
     * existing polygons in it. However, you shouldn't call this function with
     * the same parameter in \p lines as \p result_lines, because that would
     * duplicate (some of) the polylines.
     * \param lines The lines to stitch together.
     * \param result_lines[out] The stitched parts that are not closed polygons
     * will be stored in here.
     * \param result_polygons[out] The stitched parts that were closed as
     * polygons will be stored in here.
     * \param max_stitch_distance The maximum distance that will be bridged to
     * connect two lines.
     * \param snap_distance Points closer than this distance are considered to
     * be the same point.
     */
    static void stitch(const InputPaths& lines, InputPaths& result_lines, OutputPaths& result_polygons, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10);

    /*!
     * Whether a polyline is allowed to be reversed. (Not true for wall polylines which are not odd)
     */
    static bool canReverse(const PathsPointIndex<InputPaths>& polyline);

    /*!
     * Whether two paths are allowed to be connected.
     * (Not true for an odd and an even wall.)
     */
    static bool canConnect(const Path& a, const Path& b);

    static bool isOdd(const Path& line);

private:
    static void pushToClosedResult(OutputPaths& result_polygons, const Path& polyline);
};

} // namespace cura
#endif // UTILS_POLYLINE_STITCHER_H
