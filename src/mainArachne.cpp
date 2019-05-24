
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>

#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/geometries/register/point.hpp> 
#include <boost/geometry/geometries/register/ring.hpp> 

#include <boost/polygon/voronoi.hpp>


#include "utils/polygon.h"
#include "utils/SVG.h"
#include "utils/linearAlg2D.h"
#include "utils/HalfEdgeGraph.h"


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
#include "utils/logoutput.h"

#include "utils/gettime.h"

using coord_t = arachne::coord_t;
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
    Segment(coord_t x1, coord_t y1, coord_t x2, coord_t y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost {
namespace polygon {

template <>
struct geometry_concept<Point> {
    typedef point_concept type;
};

template <>
struct point_traits<Point> {
    typedef coord_t coordinate_type;

    static inline coordinate_type get(
            const Point& point, orientation_2d orient) {
        return static_cast<coordinate_type>((orient == HORIZONTAL) ? point.X : point.Y);
    }
};

template <>
struct geometry_concept<Segment> {
    typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
    typedef coord_t coordinate_type;
    typedef Point point_type;

    static inline point_type get(const Segment& segment, direction_1d dir) {
        return dir.to_int() ? segment.p1 : segment.p0;
    }
};
}    // polygon
}    // boost


namespace arachne
{
void debugOutput(voronoi_diagram<pos_t>& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = false, bool show_parabola_generators = false)
{
    AABB aabb;
//     for (const voronoi_diagram<pos_t>::vertex_type& vert : vd.vertices())
//     {
//         aabb.include(Point(vert.x(), vert.y()));
//     }
    for (const Point& p : points)
    {
        aabb.include(p);
    }
    for (const Segment& s : segments)
    {
        aabb.include(s.p0);
        aabb.include(s.p1);
    }
    
    
    SVG svg("output/try2.svg", aabb);
    
    for (const Point& p : points)
    {
        svg.writePoint(p, show_coords, 2);
    }
    for (const Segment& s : segments)
    {
        svg.writeLine(s.p0, s.p1, SVG::Color::BLACK, 2);
        if (draw_points) svg.writePoint(s.p0, show_coords, 2);
        if (draw_points) svg.writePoint(s.p1, show_coords, 2);
    }
    
    
    printf("%zu edges\n", vd.edges().size());
    
    for (const vd_t::edge_type& edge : vd.edges())
    {
        const vd_t::vertex_type* from = edge.vertex0();
        const vd_t::vertex_type* to = edge.vertex1();
        if (!to) continue; // only process half of the half-edges
        if (from && to)
        {
            Point from_(edge.vertex0()->x(), edge.vertex0()->y());
            Point to_(edge.vertex1()->x(), edge.vertex1()->y());
//             printf("(%lld,%lld)-(%lld,%lld)\n", from_.X, from_.Y, to_.X, to_.Y);
            if (from_.X +from_.Y < to_.X + to_.Y) continue; // only process half of the half-edges
            if (edge.is_linear())
            {
                svg.writeLine(Point(from->x(), from->y()), Point(to->x(), to->y()), SVG::Color::RED);
            }
            else
            {
                const vd_t::cell_type& left_cell = *edge.cell();
                const vd_t::cell_type& right_cell = *edge.twin()->cell();
                
                assert(left_cell.contains_point() == right_cell.contains_segment());
                const vd_t::cell_type& segment_cell = (left_cell.contains_segment())? left_cell : right_cell;
                const vd_t::cell_type& point_cell = (left_cell.contains_point())? left_cell : right_cell;
                
                Point* point = nullptr;
                int segment_idx = segment_cell.source_index() - points.size();
//                 if (segment_idx >= segments.size()) continue;
                assert(segment_idx < segments.size());
                Segment& segment = segments[segment_idx];
                
                switch (point_cell.source_category())
                {
                case boost::polygon::SOURCE_CATEGORY_SINGLE_POINT:
                    point = &points[point_cell.source_index()];
                    break;
                case boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT:
                    assert(point_cell.source_index() - points.size() < segments.size());
                    point = &segments[point_cell.source_index() - points.size()].p0;
                    break;
                case boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT:
                    assert(point_cell.source_index() - points.size() < segments.size());
                    point = &segments[point_cell.source_index() - points.size()].p1;
                    break;
                default:
                    printf("WTF! Point is no point?!\n");
                    break;
                }
                if (!point)
                {
                    printf("WTF! Cannot make arc!\n");
                    continue;
                }
                Point mid;
                Point s = segment.p1 - segment.p0;
                if ((dot(from_, s) < dot(*point, s)) == (dot(to_, s) < dot(*point, s)))
                {
                    svg.writeLine(from_, to_, SVG::Color::BLUE);
                    mid = (from_ + to_) / 2;
                }
                else
                {
                    Point projected = LinearAlg2D::getClosestOnLineSegment(*point, segment.p0, segment.p1);
                    mid = (*point + projected) / 2;
                    svg.writeLine(from_, mid, SVG::Color::BLUE);
                    svg.writeLine(mid, to_, SVG::Color::BLUE);
                    //std::vector<Point> discretization;
                    //boost::polygon::voronoi_visual_utils<pos_t>::discretize(*point, *segment, 10, &discretization)
                }
                if (show_parabola_generators)
                {
                    svg.writeLine(mid, *point, SVG::Color::GRAY);
                    svg.writeLine(mid, (segment.p0 + segment.p1) / 2, SVG::Color::GRAY);
                }
            }
        }
        else 
        {
            if (edge.is_infinite())
            {
//                 printf("Edge is infinite\n");
            }
            else
            {
//                 printf("Cannot draw edge\n");
            }
            if (edge.vertex0())
            {
                if (draw_points) svg.writePoint(Point(edge.vertex0()->x(), edge.vertex0()->y()), false, 3, SVG::Color::RED);
            }
            if (edge.vertex1())
            {
                if (draw_points) svg.writePoint(Point(edge.vertex1()->x(), edge.vertex1()->y()), false, 3, SVG::Color::RED);
            }
        }
    }
    
    if (show_coords)
    {
        for (const vd_t::vertex_type& vert : vd.vertices())
        {
            svg.writePoint(Point(vert.x(), vert.y()), show_coords, 2, SVG::Color::RED);
        }
    }
    
//     for (const vd_t::cell_type& cell : vd.cells())
//     {
//         
//     }
}

Polygons generateTestPoly(size_t size, Point border)
{
    Polygons polys;
    PolygonRef poly = polys.newPoly();
    for (int i = 0; i < size; i++)
    {
        poly.emplace_back(rand() % border.X, rand() % border.Y);
    }
    
    polys = polys.unionPolygons();
//     polys = polys.offset(border.X*1.2, ClipperLib::jtRound);
    
//     polys = polys.offset(border.X*2, ClipperLib::jtRound);
//     polys = polys.offset(-border.X*1.8, ClipperLib::jtRound);
    
    polys = polys.offset(-5, ClipperLib::jtRound);
//     polys = polys.offset(10, ClipperLib::jtRound);
//     polys = polys.offset(-5, ClipperLib::jtRound);
//     polys = polys.offset(-border.X/200, ClipperLib::jtRound);
//     polys = polys.offset(border.X/100, ClipperLib::jtRound);
//     polys = polys.offset(-border.X/200, ClipperLib::jtRound);
    polys = polys.unionPolygons();
    return polys;
}
} // namespace arachne


int main() {
    // Preparing Input Geometries.
    int r;
    r = 1558617038;
    r = time(0);
//     r = 1558618076;
    r = 1558692831;
    srand(r);
    printf("random seed: %d\n", r);
    arachne::logError("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    arachne::TimeKeeper tk;
    
    int total_n = 1;
    for (int n = 0; n < total_n; n++)
    {
        Point border_size = Point(10000,10000);
        arachne::Polygons polys = arachne::generateTestPoly(40, border_size);
        {
            arachne::SVG svg("output/outline.svg", arachne::AABB(Point(0,0), border_size));
            svg.writePolygons(polys);
        }
        
        
        std::vector<Point> points;
        std::vector<Segment> segments;
//         if (false)
        {
            for (arachne::PolygonRef poly : polys)
            {
                Point last = poly.back();
                for (arachne::Point p : poly)
                {
                    float m = 1.0;
                    segments.emplace_back(last.X*m, last.Y*m, p.X*m, p.Y*m);
//                     printf("segments.emplace_back(%d, %d, %d, %d);\n", last.X, last.Y, p.X, p.Y);
                    last = p;
                }
            }
        }

        // Construction of the Voronoi Diagram.
        voronoi_diagram<pos_t> vd;
        construct_voronoi(points.begin(), points.end(),
                                            segments.begin(), segments.end(),
                                            &vd);
        
        

        
        arachne::debugOutput(vd, points, segments);
        
        if (n / 100 != (n-1)/100) arachne::logError("%f%%\n", float(n) / total_n * 100.0);
        
    }
    arachne::logError("Toal processing took %fs\n", tk.restart());
    return 0;
}
