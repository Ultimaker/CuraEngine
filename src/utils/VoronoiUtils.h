//Copyright (c) 2018 Ultimaker B.V.


#ifndef UTILS_VORONOI_UTILS_H
#define UTILS_VORONOI_UTILS_H

#include <vector>


#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_diagram;

#include "SVG.h"
#include "PolygonsSegmentIndex.h"


namespace arachne 
{

/*!
 */
class VoronoiUtils
{
public:
    using Segment = PolygonsSegmentIndex;
    using voronoi_data_t = double;
    using vd_t = voronoi_diagram<voronoi_data_t>;
    static void debugOutput(std::string filename, vd_t& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = false, bool show_parabola_generators = false);
    static void debugOutput(SVG& svg, vd_t& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = false, bool show_parabola_generators = false);
    
    static std::vector<Point> discretizeParabola(Point generator_point, Segment generator_segment, Point start, Point end, coord_t approximate_step_size);

    static Point getSourcePoint(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    static const Segment& getSourceSegment(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    static PolygonsPointIndex getSourcePointIndex(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);

    static Point p(const vd_t::vertex_type* node);
    
    static bool isSourcePoint(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments, coord_t snap_dist = 10);
    
    static coord_t getDistance(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
};


}//namespace arachne



#endif//UTILS_VORONOI_UTILS_H
