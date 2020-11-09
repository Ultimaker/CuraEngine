//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BEADING_ORDER_OPTIMIZER_H
#define BEADING_ORDER_OPTIMIZER_H

#include "../utils/polygon.h"
#include "../utils/ExtrusionSegment.h"
#include "../utils/ExtrusionJunction.h"
#include "../utils/ExtrusionLine.h"

namespace cura
{

/*!
 * Connecting ExtrusionSegments together into chains / polygons
 * 
 * TODO: there's a bug which connects the wrong end of a short segment because
 * our algorithm is greedy. When adding a segment we should explicitly check
 * whether it would be better to connect the segment the other way around.
 */
class BeadingOrderOptimizer
{
public:
    /*!
     * Optimize the order in which polylines and polygons are printed.
     *
     * The result is stored in-place in the vectors of lists of extrusion lines.
     * The extrusion lines will be connected to the optimal subsequent extrusion
     * line to print after it.
     * \param[in,out] polygons_per_index The resulting polygons by connecting
     * end points of polylines together into loops.
     * \param[in,out] polylines_per_index The input polylines and the output
     * connected polylines which are not combined into polygons.
     * \param reduce_overlapping_segments Whether to compensate for overlap in
     * these line segments.
     * \param connect_odd_lines_to_polygons Whether to preferably connect an odd
     * count segment to a polygon, rather than closing the polygonal toolpath.
     */
    static void optimize(VariableWidthPaths& polygons_per_index, VariableWidthPaths& polylines_per_index, const bool reduce_overlapping_segments = true, const bool connect_odd_lines_to_polygons = true);
private:
    /*!
     * This private constructor is called by \ref optimize. It creates one
     * instance to track state while optimizing.
     * \param polylines_per_index The polylines which are not connected into
     * loops.
     */
    BeadingOrderOptimizer(VariableWidthPaths& polylines_per_index);

    /*!
     * A reference to one endpoint of a polyline.
     *
     * These endpoints have to be linked together with the smallest distance in
     * order to create the optimal printing order.
     *
     * Each endpoint is referenced by an iterator somewhere in a list of
     * ``ExtrusionLine``s, and contains additional information on whether it
     * points to the front or the end of that line, and which inset it is (in
     * order to order the outer wall correctly).
     */
    struct ExtrusionLineEndRef
    {
        /*!
         * The inset of the polyline that this is an endpoint of.
         */
        coord_t inset_idx;

        /*!
         * The polyline that this is an endpoint of.
         */
        VariableWidthLines::iterator polyline;

        /*!
         * Whether to point to the front or the end of the extrusion line.
         */
        bool front;

        ExtrusionLineEndRef(const coord_t inset_idx, const VariableWidthLines::iterator polyline, const bool front)
        : inset_idx(inset_idx)
        , polyline(polyline)
        , front(front)
        {}

        /*!
         * Get the position of this endpoint.
         */
        Point p() const
        {
            return front? polyline->junctions.front().p : polyline->junctions.back().p;
        }

        bool operator==(const ExtrusionLineEndRef& other)
        {
            return inset_idx == other.inset_idx
            && polyline == other.polyline
            && front == other.front;
        }
    };

    /*!
     * Fraction of the line length that is allowed to overlap without the
     * overlap compensation kicking in.
     *
     * This allows sharp corners to just be printed without compensating for the
     * overlap with the previous line.
     */
    static constexpr float intersection_overlap = .25;

    /*!
     * Distances smaller than this distance get neglected and printed in one go
     * without travel move.
     */
    static constexpr coord_t snap_dist = MM2INT(0.01);

    /*!
     * All of the polylines that must be optimized.
     */
    VariableWidthPaths& polylines_per_index;

    /*!
     * A mapping of where the endpoints of the polylines are.
     *
     * This allows us to look up quickly which endpoints are in each location.
     */
    std::unordered_map<Point, ExtrusionLineEndRef> polyline_end_points;

    /*!
     * Connecting polylines together, but allowing for rounding errors in the
     * end points.
     *
     * Reduce unconnected polylines away from the intersection locations as
     * well.
     * \param polygons_per_index The polygons (closed loop) that need to be
     * optimized, together with the polylines provided in the constructor.
     * \param snap_dist Distances smaller than this distance get neglected, and
     * the lines printed in one go without travel move.
     * \param reduce_overlapping_segments Compensate for overlaps in these
     * lines.
     * \param connect_odd_lines_to_polygons Should polylines (resulting from an
     * odd number of walls in a thin part) be connected to the closed-loop
     * polygons? If so, this might open those polygons, creating a seam, but it
     * will remove another seam where the polyline is adjacent to a polygon.
     */
    void fuzzyConnect(VariableWidthPaths& polygons_per_index, const coord_t snap_dist, const bool reduce_overlapping_segments, const bool connect_odd_lines_to_polygons);

    /*!
     * Compensate for overlaps between intersecting lines.
     *
     * This only checks for intersections within the same polyline. Two adjacent
     * line segments in the polyline may intersect if they make a very sharp
     * corner.
     * This is a recursive function that continues in recursion when a whole
     * line segment overlaps with another. The next line segment may still
     * overlap with the same reduction source.
     * \param polyline The polyline in which to reduce overlaps.
     * \param polyline_start_it The line segments that overlaps with a previous
     * line segment.
     * \param traveled_dist The length of line segments we've been compensating
     * so far to reduce overlap for this intersection.
     * \param reduction_length How far to compensate overlaps for this
     * intersection.
     * \param reduction_source The line segment that we're colliding with, that
     * makes us reduce these line segments to compensate for their overlap.
     */
    template<typename directional_iterator>
    void reduceIntersectionOverlap(ExtrusionLine& polyline, const directional_iterator polyline_start_it, const coord_t traveled_dist, const coord_t reduction_length, const ExtrusionLineEndRef& reduction_source);

    /*!
     * Get an iterator where we can insert new junctions.
     */
    template<typename directional_iterator>
    static std::vector<ExtrusionJunction>::iterator getInsertPosIt(const directional_iterator it);

    /*!
     * Get an iterator pointing to a certain junction.
     */
    template<typename directional_iterator>
    static std::vector<ExtrusionJunction>::iterator getSelfPosIt(const directional_iterator it);

    /*!
     * Test if an iterator points to the end of a polyline.
     */
    template<typename directional_iterator>
    bool isEnd(const directional_iterator it, const ExtrusionLine& polyline) const;
};

} // namespace cura
#endif // BEADING_ORDER_OPTIMIZER_H
