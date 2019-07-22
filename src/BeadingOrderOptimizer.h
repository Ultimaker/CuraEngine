//Copyright (c) 2019 Ultimaker B.V.


#ifndef BEADING_ORDER_OPTIMIZER_H
#define BEADING_ORDER_OPTIMIZER_H

#include "utils/polygon.h"
#include "utils/ExtrusionSegment.h"
#include "utils/ExtrusionJunction.h"

namespace arachne
{

/*!
 * Connecting ExtrusionSegments together into chains / polygons
 */
class BeadingOrderOptimizer
{
public:
    static void optimize(const std::vector<ExtrusionSegment>& segments, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index);
private:
    BeadingOrderOptimizer(const std::vector<ExtrusionSegment>& segments)
    : segments(segments)
    {
        even_polyline_end_points.reserve(segments.size());
        odd_polyline_end_points.reserve(segments.size());
    }

    struct Polyline
    {
        coord_t inset_idx;
        std::list<ExtrusionJunction> junctions;
        Polyline(coord_t inset_idx)
        : inset_idx(inset_idx)
        {}
        coord_t computeLength()
        {
            if (junctions.size() <= 1) return 0;
            coord_t len = 0;
            ExtrusionJunction prev = junctions.front();
            for (ExtrusionJunction& next : junctions)
            {
                len += vSize(next.p - prev.p);
                prev = next;
            }
            return len;
        }
    };

    struct PolylineEndRef
    {
        coord_t inset_idx;
        std::list<Polyline>::iterator polyline;
        bool front;
        PolylineEndRef(coord_t inset_idx, std::list<Polyline>::iterator polyline, bool front)
        : inset_idx(inset_idx)
        , polyline(polyline)
        , front(front)
        {}
        Point p() const
        {
            return front? polyline->junctions.front().p : polyline->junctions.back().p;
        }
        bool operator==(const PolylineEndRef& other)
        {
            return inset_idx == other.inset_idx
            && polyline == other.polyline
            && front == other.front;
        }
    };
    
    static constexpr float intersection_overlap = .25;
    static constexpr coord_t snap_dist = 10;

    const std::vector<ExtrusionSegment>& segments;


    std::list<Polyline> even_polylines;
    std::list<Polyline> odd_polylines; // keep odd single bead segments separate so that polygon segments can combine together into polygons
    std::unordered_map<Point, PolylineEndRef> even_polyline_end_points;
    std::unordered_map<Point, PolylineEndRef> odd_polyline_end_points;

    void connect(std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index);

    /*!
     * Connecting polylines together, but allowing for rounding erros in the end points
     * 
     * Reduce unconnected polylines away from the intersection locations as well
     */
    void fuzzyConnect(std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index, coord_t snap_dist);

    template<typename directional_iterator>
    void reduceIntersectionOverlap(Polyline& polyline, directional_iterator polyline_start_it, coord_t traveled_dist, coord_t reduction_length);

    template<typename directional_iterator>
    static std::list<ExtrusionJunction>::iterator getInsertPosIt(directional_iterator it);

    template<typename directional_iterator>
    static std::list<ExtrusionJunction>::iterator getSelfPosIt(directional_iterator it);

    template<typename directional_iterator>
    bool isEnd(directional_iterator it, Polyline& polyline);
    
    void transferUnconnectedPolylines(std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polylines_per_index);
    
    void debugCheck();
};




} // namespace arachne
#endif // BEADING_ORDER_OPTIMIZER_H
