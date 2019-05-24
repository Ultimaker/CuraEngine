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
    static void debugOutput(SVG& svg, voronoi_diagram<voronoi_data_t>& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = false, bool show_parabola_generators = false);
    static void debugOutput(std::string filename, voronoi_diagram<voronoi_data_t>& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = false, bool show_parabola_generators = false);
    
    static std::vector<Point> discretizeParabola(Point generator_point, Segment generator_segment, Point start, Point end, coord_t approximate_step_size);
};


}//namespace arachne



#endif//UTILS_VORONOI_UTILS_H
