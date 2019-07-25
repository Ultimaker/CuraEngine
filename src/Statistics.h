//Copyright (c) 2019 Ultimaker B.V.


#ifndef STATISTICS_H
#define STATISTICS_H

#include "utils/polygon.h"
#include "utils/ExtrusionSegment.h"
#include "utils/ExtrusionJunction.h"
#include "VoronoiQuadrangulation.h"

namespace arachne
{

/*!
 * Get statistics of the resulting toolpaths
 */
class Statistics
{
public:
    Statistics(std::string filename_base, std::string output_prefix)
    : filename_base(filename_base)
    , output_prefix(output_prefix)
    , input(nullptr)
    {
    }
    void analyse(Polygons& input, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index, VoronoiQuadrangulation* vq = nullptr);
    void visualize();
private:
    struct Segment
    {
        ExtrusionSegment s;
        bool is_full;
        Segment(ExtrusionSegment s, bool is_full)
        : s(s)
        , is_full(is_full)
        {}
        Polygons toPolygons()
        {
            return s.toPolygons(!is_full);
        }
    };
    std::string filename_base;
    std::string output_prefix;
    Polygons* input;
    VoronoiQuadrangulation* vq;

    std::vector<std::vector<std::vector<ExtrusionJunction>>>* polygons_per_index;
    std::vector<std::vector<std::vector<ExtrusionJunction>>>* polylines_per_index;
    std::vector<Segment> all_segments;
    Polygons area_covered;
    Polygons overlaps;
    Polygons underfills;
    Polygons overfills;
    Polygons double_overfills;
    Polygons paths;

    void generateAllSegments(std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index);

    std::vector<Segment> discretize(const Segment& ss, coord_t step_size);
};




} // namespace arachne
#endif // STATISTICS_H
