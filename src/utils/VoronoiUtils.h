//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_VORONOI_UTILS_H
#define UTILS_VORONOI_UTILS_H

#include <vector>


#include <boost/polygon/voronoi.hpp>

#include "SVG.h"
#include "PolygonsSegmentIndex.h"


namespace cura 
{

/*!
 */
class VoronoiUtils
{
public:
    using Segment = PolygonsSegmentIndex;
    using voronoi_data_t = double;
    using vd_t = boost::polygon::voronoi_diagram<voronoi_data_t>;

    static Point getSourcePoint(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    static const Segment& getSourceSegment(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    static PolygonsPointIndex getSourcePointIndex(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);

    static Point p(const vd_t::vertex_type* node);
    
    static bool isSourcePoint(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments, coord_t snap_dist = 10);
    
    static coord_t getDistance(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments);
    
    /*!
     * Discretize a parabola based on (approximate) step size.
     * The \p approximate_step_size is measured parallel to the \p source_segment, not along the parabola.
     */
    static std::vector<Point> discretizeParabola(const Point& source_point, const Segment& source_segment, Point start, Point end, coord_t approximate_step_size, float transitioning_angle);

protected:
    /*!
     * Discretize parabola based on max absolute deviation from the parabola.
     * 
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
    static coord_t parabolaY(coord_t x, coord_t a, coord_t b);

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
    static double getPointProjection(const Point& point, const Segment& segment);

};

} // namespace cura

#endif // UTILS_VORONOI_UTILS_H
