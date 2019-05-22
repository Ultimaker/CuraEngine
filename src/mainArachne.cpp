
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>

#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/geometries/register/point.hpp> 
#include <boost/geometry/geometries/register/ring.hpp> 

#include <boost/polygon/voronoi.hpp>


#include "utils/SVG.h"

// Boost.Polygon library voronoi_basic_tutorial.cpp file

//                    Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
//        (See accompanying file LICENSE_1_0.txt or copy at
//                    http://www.boost.org/LICENSE_1_0.txt)

// See http://www.boost.org for updates, documentation, and revision history.


#include <cstdio>
#include <vector>

#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

#include "voronoi_visual_utils.hpp"


#include "utils/IntPoint.h"

#include "utils/Coord_t.h"

using pos_t = double;
using vd_t = voronoi_diagram<pos_t>;


using Point = arachne::Point;
/*
struct Point {
    int a;
    int b;
    Point(int x, int y) : a(x), b(y) {}
};
*/

struct Segment {
    Point p0;
    Point p1;
    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost {
namespace polygon {

template <>
struct geometry_concept<Point> {
    typedef point_concept type;
};

template <>
struct point_traits<Point> {
    typedef int coordinate_type;

    static inline coordinate_type get(
            const Point& point, orientation_2d orient) {
        return (orient == HORIZONTAL) ? point.X : point.Y;
    }
};

template <>
struct geometry_concept<Segment> {
    typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
    typedef int coordinate_type;
    typedef Point point_type;

    static inline point_type get(const Segment& segment, direction_1d dir) {
        return dir.to_int() ? segment.p1 : segment.p0;
    }
};
}    // polygon
}    // boost

// Traversing Voronoi edges using edge iterator.
int iterate_primary_edges1(const voronoi_diagram<pos_t>& vd) {
    int result = 0;
    for (voronoi_diagram<pos_t>::const_edge_iterator it = vd.edges().begin();
             it != vd.edges().end(); ++it) {
        if (it->is_primary())
            ++result;
    }
    return result;
}

// Traversing Voronoi edges using cell iterator.
int iterate_primary_edges2(const voronoi_diagram<pos_t> &vd) {
    int result = 0;
    for (voronoi_diagram<pos_t>::const_cell_iterator it = vd.cells().begin();
             it != vd.cells().end(); ++it) {
        const voronoi_diagram<pos_t>::cell_type& cell = *it;
        const voronoi_diagram<pos_t>::edge_type* edge = cell.incident_edge();
        // This is convenient way to iterate edges around Voronoi cell.
        do {
            if (edge->is_primary())
                ++result;
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
    return result;
}

// Traversing Voronoi edges using vertex iterator.
// As opposite to the above two functions this one will not iterate through
// edges without finite endpoints and will iterate only once through edges
// with single finite endpoint.
int iterate_primary_edges3(const voronoi_diagram<pos_t> &vd) {
    int result = 0;
    for (voronoi_diagram<pos_t>::const_vertex_iterator it =
             vd.vertices().begin(); it != vd.vertices().end(); ++it) {
        const voronoi_diagram<pos_t>::vertex_type& vertex = *it;
        const voronoi_diagram<pos_t>::edge_type* edge = vertex.incident_edge();
        // This is convenient way to iterate edges around Voronoi vertex.
        do {
            if (edge->is_primary())
                ++result;
            edge = edge->rot_next();
        } while (edge != vertex.incident_edge());
    }
    return result;
}

/*
  point_type retrieve_point(const cell_type& cell) {
    source_index_type index = cell.source_index();
    source_category_type category = cell.source_category();
    if (category == SOURCE_CATEGORY_SINGLE_POINT) {
      return point_data_[index];
    }
    index -= point_data_.size();
    if (category == SOURCE_CATEGORY_SEGMENT_START_POINT) {
      return low(segment_data_[index]);
    } else {
      return high(segment_data_[index]);
    }
  }

  segment_type retrieve_segment(const cell_type& cell) {
    source_index_type index = cell.source_index() - point_data_.size();
    return segment_data_[index];
  }
*/


namespace arachne
{
void try1(voronoi_diagram<pos_t>& vd, std::vector<Point>& points, std::vector<Segment>& segments)
{
    AABB aabb;
    for (const voronoi_diagram<pos_t>::vertex_type& vert : vd.vertices())
    {
        aabb.include(Point(vert.x(), vert.y()));
    }
    for (const Point& p : points)
    {
        aabb.include(p);
    }
    for (const Segment& s : segments)
    {
        aabb.include(s.p0);
        aabb.include(s.p1);
    }
    
    
    
    SVG svg("output/try.svg", aabb);
    
    for (const Point& p : points)
    {
        svg.writePoint(p);
    }
    for (const Segment& s : segments)
    {
        svg.writeLine(s.p0, s.p1);
        svg.writePoint(s.p0);
        svg.writePoint(s.p1);
    }
    
    
    
    
    
    for (const vd_t::edge_type& edge : vd.edges())
    {
        const vd_t::vertex_type* from = edge.vertex0();
        const vd_t::vertex_type* to = edge.vertex1();
        if (from && to)
        {
            if (edge.is_linear())
            {
                svg.writeLine(Point(from->x(), from->y()), Point(to->x(), to->y()), SVG::Color::RED);
            }
            else
            {
                const vd_t::cell_type& left_cell = *edge.cell();
                const vd_t::cell_type& right_cell = *edge.twin()->cell();
                
                const vd_t::cell_type& segment_cell = (boost::polygon::belongs(left_cell.source_category(), boost::polygon::GEOMETRY_CATEGORY_SEGMENT))? left_cell : right_cell;
                const vd_t::cell_type& point_cell = (boost::polygon::belongs(left_cell.source_category(), boost::polygon::GEOMETRY_CATEGORY_POINT))? left_cell : right_cell;
                
                Point* point = nullptr;
                Segment& segment = segments[segment_cell.source_index()];
                
                switch (point_cell.source_category())
                {
                case boost::polygon::SOURCE_CATEGORY_SINGLE_POINT:
                    point = &points[point_cell.source_index()];
                    break;
                case boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT:
                    point = &segments[point_cell.source_index()].p0;
                    break;
                case boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT:
                    point = &segments[point_cell.source_index()].p1;
                    break;
                default:
                    break;
                }
                if (!point)
                {
                    printf("Cannot make arc!\n");
                    continue;
                }
                Point mid = *point / 2 + (segment.p0 + segment.p1) / 4;
                svg.writeLine(Point(from->x(), from->y()), mid, SVG::Color::BLUE);
                svg.writeLine(mid, Point(to->x(), to->y()), SVG::Color::BLUE);
                //std::vector<Point> discretization;
                //boost::polygon::voronoi_visual_utils<pos_t>::discretize(*point, *segment, 10, &discretization)
            }
        }
    }
    
    for (const vd_t::vertex_type& vert : vd.vertices())
    {
        svg.writePoint(Point(vert.x(), vert.y()), false, 3, SVG::Color::RED);
    }
    
    for (const vd_t::cell_type& cell : vd.cells())
    {
        
    }
}
} // namespace arachne



int main() {
    // Preparing Input Geometries.
    std::vector<Point> points;
    points.push_back(Point(0, 0));
    points.push_back(Point(100, 600));
    std::vector<Segment> segments;
    segments.push_back(Segment(-400, 500, 500, -100));
    segments.push_back(Segment(300, -1100, 1300, -100));

    // Construction of the Voronoi Diagram.
    voronoi_diagram<pos_t> vd;
    construct_voronoi(points.begin(), points.end(),
                                        segments.begin(), segments.end(),
                                        &vd);
    
    

    
    
    
    
    
    
    
    
    // Traversing Voronoi Graph.
    {
        printf("Traversing Voronoi graph.\n");
        printf("Number of visited primary edges using edge iterator: %d\n",
                iterate_primary_edges1(vd));
        printf("Number of visited primary edges using cell iterator: %d\n",
                iterate_primary_edges2(vd));
        printf("Number of visited primary edges using vertex iterator: %d\n",
                iterate_primary_edges3(vd));
        printf("\n");
    }

    // Using color member of the Voronoi primitives to store the average number
    // of edges around each cell (including secondary edges).
    {
        printf("Number of edges (including secondary) around the Voronoi cells:\n");
        for (voronoi_diagram<pos_t>::const_edge_iterator it = vd.edges().begin();
                 it != vd.edges().end(); ++it) {
            std::size_t cnt = it->cell()->color();
            it->cell()->color(cnt + 1);
        }
        for (voronoi_diagram<pos_t>::const_cell_iterator it = vd.cells().begin();
                 it != vd.cells().end(); ++it) {
            printf("%lu ", it->color());
        }
        printf("\n");
        printf("\n");
    }

    // Linking Voronoi cells with input geometries.
    {
        unsigned int cell_index = 0;
        for (voronoi_diagram<pos_t>::const_cell_iterator it = vd.cells().begin();
                 it != vd.cells().end(); ++it) {
            if (it->contains_point()) {
                std::size_t index = it->source_index();
                Point p = points[index];
                printf("Cell #%ud contains a point: (%d, %d).\n",
                             cell_index, x(p), y(p));
            } else {
                std::size_t index = it->source_index() - points.size();
                Point p0 = low(segments[index]);
                Point p1 = high(segments[index]);
                if (it->source_category() ==
                        boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
                    printf("Cell #%ud contains segment start point: (%d, %d).\n",
                                 cell_index, x(p0), y(p0));
                } else if (it->source_category() ==
                                     boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
                    printf("Cell #%ud contains segment end point: (%d, %d).\n",
                                 cell_index, x(p0), y(p0));
                } else {
                    printf("Cell #%ud contains a segment: ((%d, %d), (%d, %d)). \n",
                                 cell_index, x(p0), y(p0), x(p1), y(p1));
                }
            }
            ++cell_index;
        }
    }
    return 0;
}
