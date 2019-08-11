//Copyright (c) 2019 Ultimaker B.V.


#ifndef STATISTICS_H
#define STATISTICS_H

#include "utils/polygon.h"
#include "utils/ExtrusionJunction.h"
#include "utils/ExtrusionSegment.h"
#include "utils/ExtrusionLine.h"
#include "VoronoiQuadrangulation.h"

namespace arachne
{

/*!
 * Get statistics of the resulting toolpaths
 */
class Statistics
{
public:
    Statistics(std::string test_type, std::string output_prefix, double processing_time)
    : processing_time(processing_time)
    , test_type(test_type)
    , output_prefix(output_prefix)
    , input(nullptr)
    {
    }
    void analyse(Polygons& input, std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, VoronoiQuadrangulation* vq = nullptr);
    void visualize(coord_t nozzle_size, bool output_vq = false, bool output_toolpaths = false, bool output_widths = true, bool include_legend = false, bool output_accuracy = true, bool visualize_pretty_paths = false);
    void saveResultsCSV();
    double processing_time;
    double overfill_area;
    double double_overfill_area;
    double total_underfill_area;
    double total_target_area;
    double total_target_area_length;
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
    std::string test_type;
    std::string output_prefix;
    Polygons* input;
    VoronoiQuadrangulation* vq;

    std::vector<std::list<ExtrusionLine>>* polygons_per_index;
    std::vector<std::list<ExtrusionLine>>* polylines_per_index;
    std::vector<Segment> all_segments;
    Polygons area_covered;
    Polygons overlaps;
    Polygons underfills;
    Polygons overfills;
    Polygons double_overfills;
    Polygons paths;

    void generateAllSegments(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index);

    std::vector<Segment> discretize(const Segment& ss, coord_t step_size);
};




} // namespace arachne
#endif // STATISTICS_H
