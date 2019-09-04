//Copyright (c) 2019 Ultimaker B.V.


#ifndef BEADING_ORDER_OPTIMIZER_H
#define BEADING_ORDER_OPTIMIZER_H

#include "utils/polygon.h"
#include "utils/ExtrusionSegment.h"
#include "utils/ExtrusionJunction.h"
#include "utils/ExtrusionLine.h"

namespace arachne
{

/*!
 * Connecting ExtrusionSegments together into chains / polygons
 * 
 * TODO: there's a bug which connects thw wrong end of a short segment because our algorithm is gready
 * When adding a segment we should explicitly check whether it would be better to connect the segment the other way around.
 */
class BeadingOrderOptimizer
{
public:
    /*!
     * \param[in,out] polygons_per_index the resulting polygons by connecting end poitns of polylines together into loops
     * \param[in,out] polylines_per_index The input polylines and the output connected polylines which are not combined into polygons
     */
    static void optimize(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, bool reduce_overlapping_segments = true);
private:
    BeadingOrderOptimizer(std::vector<std::list<ExtrusionLine>>& polylines_per_index);

    struct ExtrusionLineEndRef
    {
        coord_t inset_idx;
        std::list<ExtrusionLine>::iterator polyline;
        bool front;
        ExtrusionLineEndRef(coord_t inset_idx, std::list<ExtrusionLine>::iterator polyline, bool front)
        : inset_idx(inset_idx)
        , polyline(polyline)
        , front(front)
        {}
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
    
    static constexpr float intersection_overlap = .25;
    static constexpr coord_t snap_dist = 10;

    std::vector<std::list<ExtrusionLine>>& polylines_per_index;

    std::unordered_map<Point, ExtrusionLineEndRef> polyline_end_points;

    /*!
     * Connecting polylines together, but allowing for rounding erros in the end points
     * 
     * Reduce unconnected polylines away from the intersection locations as well
     */
    void fuzzyConnect(std::vector<std::list<ExtrusionLine>>& polygons_per_index, coord_t snap_dist, bool reduce_overlapping_segments);

    template<typename directional_iterator>
    void reduceIntersectionOverlap( ExtrusionLine& polyline, directional_iterator polyline_start_it, coord_t traveled_dist, coord_t reduction_length);

    template<typename directional_iterator>
    static std::list<ExtrusionJunction>::iterator getInsertPosIt(directional_iterator it);

    template<typename directional_iterator>
    static std::list<ExtrusionJunction>::iterator getSelfPosIt(directional_iterator it);

    template<typename directional_iterator>
    bool isEnd(directional_iterator it, ExtrusionLine& polyline);
    
    void debugCheck();
};




} // namespace arachne
#endif // BEADING_ORDER_OPTIMIZER_H
