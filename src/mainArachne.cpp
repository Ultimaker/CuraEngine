
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>


#include "utils/polygon.h"
#include "utils/SVG.h"
#include "utils/linearAlg2D.h"
#include "utils/HalfEdgeGraph.h"


#include <cstdio>
#include <vector>


#include "utils/IntPoint.h"

#include "utils/Coord_t.h"
#include "utils/logoutput.h"

#include "utils/gettime.h"

using coord_t = arachne::coord_t;
using pos_t = double;


using Point = arachne::Point;

#include <string>
#include <iostream>

#include <openvoronoi/medial_axis_filter.hpp>
#include <openvoronoi/medial_axis_walk.hpp>
#include <openvoronoi/voronoidiagram.hpp>
#include <openvoronoi/polygon_interior_filter.hpp>
#include <openvoronoi/utility/vd2svg.hpp>
#include <openvoronoi/version.hpp>


namespace arachne
{
Polygons generateTestPoly(size_t size, Point border)
{
    Polygons polys;
    PolygonRef poly = polys.newPoly();
    for (int i = 0; i < size; i++)
    {
        poly.emplace_back(rand() % border.X - border.X / 2, rand() % border.Y - border.Y / 2);
    }
    
    polys = polys.unionPolygons();
    polys = polys.offset(-border.X/100);
//     polys = polys.offset(-border.X/100, ClipperLib::jtRound);
//     polys = polys.offset(border.X/50, ClipperLib::jtRound);
    return polys;
}
} // namespace arachne


ovd::Point convert(arachne::Point p)
{
    return ovd::Point(double(p.X)/10000, double(p.Y)/10000);
}


// OpenVoronoi example program. Uses MedialAxis filter to filter the complete Voronoi diagram
// down to the medial axis.
// then uses MedialAxisWalk to walk along the medial axis and draw clearance-disks
int main() {
    ovd::VoronoiDiagram* vd = new ovd::VoronoiDiagram(1,100); // (r, bins)
    // double r: radius of circle within which all input geometry must fall. use 1 (unit-circle). Scale geometry if necessary.
    // int bins:  bins for face-grid search. roughly sqrt(n), where n is the number of sites is good according to Held.
     
    std::cout << "OpenVoronoi version: " << ovd::version() << "\n"; // the git revision-string
    arachne::logError("boost version: %s\n", BOOST_LIB_VERSION);
    
    int r;
    r = 1558617038;
    r = time(0);
//     r = 1558618076;
    srand(r);
    printf("random seed: %d\n", r);
    
    bool use_generateTestPoly = true;
    
    if (!use_generateTestPoly)
    {
        ovd::Point p0(double(rand() % 1000) / -2000, double(rand() % 1000) / -2000);
        ovd::Point p1(double(rand() % 1000) / -2000, double(rand() % 1000) / 2000);
        ovd::Point p2(double(rand() % 1000) / 2000, double(rand() % 1000) / 2000);
        ovd::Point p3(double(rand() % 1000) / 2000, double(rand() % 1000) / -2000);
        ovd::Point p4(0, double(rand() % 1000) / -2000);

        int id0 = vd->insert_point_site(p0);
        int id1 = vd->insert_point_site(p1);
        int id2 = vd->insert_point_site(p2);
        int id3 = vd->insert_point_site(p3);
        int id4 = vd->insert_point_site(p4);

        vd->insert_line_site(id0, id1);
        vd->insert_line_site(id1, id2);
        vd->insert_line_site(id2, id3);
        vd->insert_line_site(id3, id4);
        vd->insert_line_site(id4, id0);
        vd->check();
    }

    arachne::Polygons polys = arachne::generateTestPoly(5, Point(10000,10000));
    {
        arachne::SVG svg_file("test.svg", arachne::AABB(Point(-5000,-5000), Point(5000,5000)));
        svg_file.writePolygons(polys);
    }
    if (use_generateTestPoly)
    {
        for (arachne::PolygonRef poly : polys)
        {
            int first_idx = vd->insert_point_site(convert(poly.front()));
                printf("point: %f, %f\n", convert(poly.front()).x, convert(poly.front()).y);
            int prev_idx = first_idx;
            for (size_t point_idx = 1; point_idx < poly.size(); point_idx++)
            {
                arachne::Point p = poly[point_idx];
                printf("point: %f, %f\n", convert(p).x, convert(p).y);
                int idx = vd->insert_point_site(convert(p));
                vd->insert_line_site(prev_idx, idx);
                prev_idx = idx;
            }
            vd->insert_line_site(prev_idx, first_idx);
        }
        vd->check();
    }
    
    
    arachne::TimeKeeper tk;
    
    // try commenting-out the line below; massive exterior clearance discs will
    // be drawn!
//     ovd::polygon_interior_filter pi(true);
//     vd->filter(&pi);
//     ovd::medial_axis_filter ma;
//     vd->filter(&ma);

    // save drawing to svg file.
    svg::Dimensions dimensions(1024, 1024);
    svg::Document doc("medial_axis.svg", svg::Layout(dimensions, svg::Layout::BottomLeft));
    ovd::HEGraph& g = vd->get_graph_reference();
    BOOST_FOREACH( ovd::HEEdge e, g.edges() ) {
        if( g[e].valid ) write_edge_to_svg(g,doc,e);
    }

    // walk the medial axis.
//     ovd::MedialAxisWalk maw(g);
//     ovd::MedialChainList chain_list = maw.walk();
//     std::cout << "walking " << int(chain_list.size()) << " medial axis chains." << std::endl;
//     BOOST_FOREACH( ovd::MedialChain chain, chain_list ) { // loop through each chain
//         std::cout << "new chain length:" << int(chain.size()) << std::endl;
//         BOOST_FOREACH( ovd::MedialPointList pt_list, chain ) { //loop through each medial-point list
//             std::cout << "new point list length:" << int(pt_list.size()) << std::endl;
//             BOOST_FOREACH( ovd::MedialPoint pt, pt_list ) { //loop through each medial-point
//                 std::cout << "pt:p:" << pt.p << ", clearance_radius:" << pt.clearance_radius << std::endl;
//                 if (pt.clearance_radius < 0.001) {
//                     std::cout << "(the clearance radius is so small that the rendered circle will be too tiny to see.)" << std::endl;
//                 }
//                 ovd::Point ctr( scale( pt.p ) );
//                 double radius( scale( pt.clearance_radius ) );
                // draw a circle, centered on the medial-axis, with radius = clearance-disc
//                 svg::Circle clearance_disc( svg::Point( ctr.x, ctr.y ), 2*radius, svg::Fill(), svg::Stroke( 1, svg::Color::Red ) );
//                 doc << clearance_disc;
//             }
//         }
//     }

    doc.save();
    std::cout << vd->print();
    delete vd;

    return 0;
}


/*
void debugOutput(voronoi_diagram<pos_t>& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points = false, bool show_coords = true, bool show_parabola_generators = false)
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
    polys = polys.offset(-border.X/100, ClipperLib::jtRound);
    polys = polys.offset(border.X/50, ClipperLib::jtRound);
    return polys;
}
} // namespace arachne


int main() {
    // Preparing Input Geometries.
    
    int total_n = 1;
    for (int n = 0; n < total_n; n++)
    {
        arachne::Polygons polys = arachne::generateTestPoly(50, Point(1000,1000));
        
        std::vector<Point> points;
        std::vector<Segment> segments;
        segments = preconfigured();
        if (false)
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
*/
