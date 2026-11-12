// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.


#pragma once

#include <vector>

#include <boost/polygon/voronoi.hpp>

#include "types/geometry/PolygonsSegmentIndex.h"


namespace boost::polygon
{
template<>
struct voronoi_diagram_traits<::curaengine::types::coord_t>
{
    typedef ::curaengine::types::coord_t coordinate_type;
    typedef voronoi_cell<coordinate_type> cell_type;
    typedef voronoi_vertex<coordinate_type> vertex_type;
    typedef voronoi_edge<coordinate_type> edge_type;

    class vertex_equality_predicate_type
    {
    public:
        bool operator()(const vertex_type& v1, const vertex_type& v2) const
        {
            return v1.x() == v2.x() && v1.y() == v2.y();
        }
    };
};
} // namespace boost::polygon

namespace curaengine::utils::voronoi_utils
{
using Segment = types::geometry::PolygonsSegmentIndex;
using voronoi_data_t = types::coord_t;
using vd_t = boost::polygon::voronoi_diagram<voronoi_data_t>;

types::geometry::Point2LL getSourcePoint(const vd_t::cell_type& cell, const std::vector<types::geometry::Point2LL>& points, const std::vector<Segment>& segments);
const Segment& getSourceSegment(const vd_t::cell_type& cell, const std::vector<types::geometry::Point2LL>& points, const std::vector<Segment>& segments);
types::geometry::PolygonsPointIndex getSourcePointIndex(const vd_t::cell_type& cell, const std::vector<types::geometry::Point2LL>& points, const std::vector<Segment>& segments);

types::geometry::Point2LL p(const vd_t::vertex_type* node);

bool isSourcePoint(
    const types::geometry::Point2LL& p,
    const vd_t::cell_type& cell,
    const std::vector<types::geometry::Point2LL>& points,
    const std::vector<Segment>& segments,
    types::coord_t snap_dist = 10);

types::coord_t
    getDistance(const types::geometry::Point2LL& p, const vd_t::cell_type& cell, const std::vector<types::geometry::Point2LL>& points, const std::vector<Segment>& segments);

/*!
 * Discretize a parabola based on (approximate) step size.
 * The \p approximate_step_size is measured parallel to the \p source_segment, not along the parabola.
 */
std::vector<types::geometry::Point2LL> discretizeParabola(
    const types::geometry::Point2LL& source_point,
    const Segment& source_segment,
    types::geometry::Point2LL start,
    types::geometry::Point2LL end,
    types::coord_t approximate_step_size,
    double transitioning_angle);

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
void discretize(const types::geometry::Point2LL& point, const Segment& segment, const types::coord_t max_dist, std::vector<types::geometry::Point2LL>* discretization);

/*!
 * adapted from boost::polygon::voronoi_visual_utils.cpp
 * Compute y(x) = ((x - a) * (x - a) + b * b) / (2 * b).
 */
types::coord_t parabolaY(types::coord_t x, types::coord_t a, types::coord_t b);

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
double getPointProjection(const types::geometry::Point2LL& point, const Segment& segment);
} // namespace curaengine::utils::voronoi_utils
