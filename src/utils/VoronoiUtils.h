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
    static void debugOutput(std::string filename, vd_t& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = true, bool show_parabola_generators = true, bool draw_arrows = false);
    static void debugOutput(SVG& svg, vd_t& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = false, bool show_parabola_generators = false, bool draw_arrows = false);

    static Point getSourcePoint(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    static const Segment& getSourceSegment(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    static PolygonsPointIndex getSourcePointIndex(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);

    static Point p(const vd_t::vertex_type* node);
    
    static bool isSourcePoint(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments, coord_t snap_dist = 10);
    
    static coord_t getDistance(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    
    
    static std::vector<Point> discretizeParabola(const Point& source_point, const Segment& source_segment, Point start, Point end, coord_t approximate_step_size);

protected:
    /*!
     * adapted from boost::polygon::voronoi_visual_utils.cpp
     * 
     * Discretize parabolic Voronoi edge.
     * Parabolic Voronoi edges are always formed by one point and one segment
     * from the initial input set.
     * 
     * Args:
     * point: input point.
     * segment: input segment.
     * max_dist: maximum discretization distance.
     * discretization: point discretization of the given Voronoi edge.
     * 
     * Template arguments:
     * InCT: coordinate type of the input geometries (usually integer).
     * Point: point type, should model point concept.
     * Segment: segment type, should model segment concept.
     * 
     * Important:
     * discretization should contain both edge endpoints initially.
     */
    static void discretize(
        const Point& point,
        const Segment& segment,
        const coord_t max_dist,
        std::vector<Point>* discretization);

    /*!
     * adapted from boost::polygon::voronoi_visual_utils.cpp
     * Compute y(x) = ((x - a) * (x - a) + b * b) / (2 * b).
     */
    static coord_t parabola_y(coord_t x, coord_t a, coord_t b);
    /*!
     * adapted from boost::polygon::voronoi_visual_utils.cpp
     * Get normalized length of the distance between:
     *     1) point projection onto the segment
     *     2) start point of the segment
     * Return this length divided by the segment length. This is made to avoid
     * sqrt computation during transformation from the initial space to the
     * transformed one and vice versa. The assumption is made that projection of
     * the point lies between the start-point and endpoint of the segment.
     */
    static double get_point_projection(const Point& point, const Segment& segment);

};


}//namespace arachne



#endif//UTILS_VORONOI_UTILS_H
