
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


std::vector<Segment> preconfigured()
{
    std::vector<Segment> segments;
    segments.emplace_back(592, 842, 727, 928);
    segments.emplace_back(727, 928, 735, 950);
    segments.emplace_back(735, 950, 730, 971);
    segments.emplace_back(730, 971, 704, 985);
    segments.emplace_back(704, 985, 555, 925);
    segments.emplace_back(555, 925, 543, 899);
    segments.emplace_back(543, 899, 562, 852);
    segments.emplace_back(562, 852, 592, 842);
    segments.emplace_back(414, 888, 391, 901);
    segments.emplace_back(391, 901, 378, 892);
    segments.emplace_back(378, 892, 345, 840);
    segments.emplace_back(345, 840, 331, 819);
    segments.emplace_back(331, 819, 331, 818);
    segments.emplace_back(331, 818, 311, 787);
    segments.emplace_back(311, 787, 297, 798);
    segments.emplace_back(297, 798, 325, 809);
    segments.emplace_back(325, 809, 332, 842);
    segments.emplace_back(332, 842, 236, 934);
    segments.emplace_back(236, 934, 209, 936);
    segments.emplace_back(209, 936, 202, 916);
    segments.emplace_back(202, 916, 244, 707);
    segments.emplace_back(244, 707, 265, 691);
    segments.emplace_back(265, 691, 281, 700);
    segments.emplace_back(281, 700, 317, 757);
    segments.emplace_back(317, 757, 364, 721);
    segments.emplace_back(364, 721, 265, 668);
    segments.emplace_back(265, 668, 255, 647);
    segments.emplace_back(255, 647, 262, 613);
    segments.emplace_back(262, 613, 247, 646);
    segments.emplace_back(247, 646, 246, 645);
    segments.emplace_back(246, 645, 242, 652);
    segments.emplace_back(242, 652, 244, 653);
    segments.emplace_back(244, 653, 150, 857);
    segments.emplace_back(150, 857, 126, 868);
    segments.emplace_back(126, 868, 112, 851);
    segments.emplace_back(112, 851, 21, 53);
    segments.emplace_back(21, 53, 36, 32);
    segments.emplace_back(36, 32, 53, 35);
    segments.emplace_back(53, 35, 180, 133);
    segments.emplace_back(180, 133, 178, 133);
    segments.emplace_back(178, 133, 307, 101);
    segments.emplace_back(307, 101, 332, 122);
    segments.emplace_back(332, 122, 328, 176);
    segments.emplace_back(328, 176, 335, 177);
    segments.emplace_back(335, 177, 360, 202);
    segments.emplace_back(360, 202, 395, 239);
    segments.emplace_back(395, 239, 425, 162);
    segments.emplace_back(425, 162, 453, 154);
    segments.emplace_back(453, 154, 464, 131);
    segments.emplace_back(464, 131, 460, 131);
    segments.emplace_back(460, 131, 455, 144);
    segments.emplace_back(455, 144, 424, 153);
    segments.emplace_back(424, 153, 377, 117);
    segments.emplace_back(377, 117, 372, 91);
    segments.emplace_back(372, 91, 384, 82);
    segments.emplace_back(384, 82, 458, 64);
    segments.emplace_back(458, 64, 465, 67);
    segments.emplace_back(465, 67, 467, 67);
    segments.emplace_back(467, 67, 478, 56);
    segments.emplace_back(478, 56, 504, 54);
    segments.emplace_back(504, 54, 510, 78);
    segments.emplace_back(510, 78, 497, 106);
    segments.emplace_back(497, 106, 546, 100);
    segments.emplace_back(546, 100, 569, 121);
    
//     segments.emplace_back(569, 121, 564, 236);
//     segments.emplace_back(564, 236, 566, 240);
//     segments.emplace_back(566, 240, 565, 241);
//     segments.emplace_back(565, 241, 569, 262);
//     segments.emplace_back(569, 262, 565, 265);
//     segments.emplace_back(565, 265, 564, 266);
//     segments.emplace_back(564, 266, 564, 269);
//     segments.emplace_back(564, 269, 576, 261);
//     segments.emplace_back(576, 261, 599, 262);
//     segments.emplace_back(599, 262, 606, 267);
//     segments.emplace_back(606, 267, 613, 246);
//     segments.emplace_back(613, 246, 622, 236);
//     segments.emplace_back(622, 236, 652, 218);
//     segments.emplace_back(652, 218, 633, 213);
//     segments.emplace_back(633, 213, 630, 196);
//     segments.emplace_back(630, 196, 648, 142);
//     segments.emplace_back(648, 142, 648, 141);
//     segments.emplace_back(648, 141, 629, 123);
//     segments.emplace_back(629, 123, 627, 97);
//     segments.emplace_back(627, 97, 641, 89);
//     segments.emplace_back(641, 89, 875, 61);
//     segments.emplace_back(875, 61, 891, 67);
//     segments.emplace_back(891, 67, 893, 69);
//     segments.emplace_back(893, 69, 891, 83);
//     segments.emplace_back(891, 83, 905, 79);
//     segments.emplace_back(905, 79, 970, 134);
//     segments.emplace_back(970, 134, 967, 166);
//     segments.emplace_back(967, 166, 943, 179);
//     segments.emplace_back(943, 179, 974, 169);
//     segments.emplace_back(974, 169, 998, 179);
//     segments.emplace_back(998, 179, 999, 194);
//     segments.emplace_back(999, 194, 940, 376);
//     segments.emplace_back(940, 376, 907, 384);
//     segments.emplace_back(907, 384, 912, 399);
//     segments.emplace_back(912, 399, 944, 390);
//     segments.emplace_back(944, 390, 977, 421);
//     segments.emplace_back(977, 421, 979, 449);
//     segments.emplace_back(979, 449, 950, 485);
//     segments.emplace_back(950, 485, 923, 489);
//     segments.emplace_back(923, 489, 915, 477);
//     segments.emplace_back(915, 477, 911, 464);
//     segments.emplace_back(911, 464, 905, 481);
//     segments.emplace_back(905, 481, 905, 482);
//     segments.emplace_back(905, 482, 909, 489);
//     segments.emplace_back(909, 489, 906, 509);
//     segments.emplace_back(906, 509, 911, 503);
//     segments.emplace_back(911, 503, 945, 510);
//     segments.emplace_back(945, 510, 951, 530);
//     segments.emplace_back(951, 530, 940, 554);
//     segments.emplace_back(940, 554, 925, 554);
//     segments.emplace_back(925, 554, 938, 568);
//     segments.emplace_back(938, 568, 932, 583);
//     segments.emplace_back(932, 583, 896, 618);
//     segments.emplace_back(896, 618, 894, 616);
//     segments.emplace_back(894, 616, 888, 627);
//     segments.emplace_back(888, 627, 885, 628);
//     segments.emplace_back(885, 628, 844, 669);
//     segments.emplace_back(844, 669, 843, 671);
//     segments.emplace_back(843, 671, 766, 903);
//     segments.emplace_back(766, 903, 743, 917);
//     segments.emplace_back(743, 917, 727, 893);
//     segments.emplace_back(727, 893, 762, 729);
//     segments.emplace_back(762, 729, 760, 726);
//     segments.emplace_back(760, 726, 753, 722);
//     segments.emplace_back(753, 722, 756, 704);
//     segments.emplace_back(756, 704, 772, 682);
//     segments.emplace_back(772, 682, 784, 628);
//     segments.emplace_back(784, 628, 797, 621);
//     segments.emplace_back(797, 621, 790, 609);
//     segments.emplace_back(790, 609, 787, 612);
//     segments.emplace_back(787, 612, 766, 615);
//     segments.emplace_back(766, 615, 768, 627);
//     segments.emplace_back(768, 627, 631, 741);
//     segments.emplace_back(631, 741, 604, 805);
//     segments.emplace_back(604, 805, 604, 806);
//     segments.emplace_back(604, 806, 606, 808);
//     segments.emplace_back(606, 808, 606, 834);
//     segments.emplace_back(606, 834, 594, 836);
//     segments.emplace_back(594, 836, 595, 838);
//     segments.emplace_back(595, 838, 581, 843);
//     segments.emplace_back(581, 843, 580, 842);
//     segments.emplace_back(580, 842, 560, 848);
//     segments.emplace_back(560, 848, 527, 826);
//     segments.emplace_back(527, 826, 518, 834);
//     segments.emplace_back(518, 834, 488, 829);
//     segments.emplace_back(488, 829, 482, 819);
//     segments.emplace_back(482, 819, 444, 865);
//     segments.emplace_back(444, 865, 421, 870);
//     segments.emplace_back(421, 870, 414, 888);
//     segments.emplace_back(487, 832, 517, 838);
//     segments.emplace_back(517, 838, 544, 882);
//     segments.emplace_back(544, 882, 540, 908);
//     segments.emplace_back(540, 908, 519, 911);
//     segments.emplace_back(519, 911, 465, 889);
//     segments.emplace_back(465, 889, 460, 854);
//     segments.emplace_back(460, 854, 487, 832);
//     segments.emplace_back(902, 518, 889, 528);
//     segments.emplace_back(889, 528, 888, 527);

//     segments.emplace_back(888, 527, 887, 528);
//     segments.emplace_back(887, 528, 864, 548);
//     segments.emplace_back(864, 548, 870, 557);
//     segments.emplace_back(870, 557, 872, 554);
//     segments.emplace_back(872, 554, 890, 546);
//     segments.emplace_back(890, 546, 911, 548);
//     segments.emplace_back(911, 548, 907, 546);
//     segments.emplace_back(907, 546, 902, 518);
//     segments.emplace_back(799, 410, 754, 444);
//     segments.emplace_back(754, 444, 761, 448);
//     segments.emplace_back(761, 448, 767, 479);
//     segments.emplace_back(767, 479, 738, 514);
//     segments.emplace_back(738, 514, 720, 515);
//     segments.emplace_back(720, 515, 721, 516);
//     segments.emplace_back(721, 516, 738, 514);
//     segments.emplace_back(738, 514, 753, 528);
//     segments.emplace_back(753, 528, 755, 528);
//     segments.emplace_back(755, 528, 805, 535);
//     segments.emplace_back(805, 535, 813, 497);
//     segments.emplace_back(813, 497, 775, 478);
//     segments.emplace_back(775, 478, 769, 447);
//     segments.emplace_back(769, 447, 799, 410);
//     segments.emplace_back(850, 351, 837, 363);
//     segments.emplace_back(837, 363, 836, 368);
//     segments.emplace_back(836, 368, 835, 368);
//     segments.emplace_back(835, 368, 831, 386);
//     segments.emplace_back(831, 386, 838, 386);
//     segments.emplace_back(838, 386, 842, 363);
//     segments.emplace_back(842, 363, 847, 354);
//     segments.emplace_back(847, 354, 850, 351);
//     segments.emplace_back(335, 359, 333, 365);
//     segments.emplace_back(333, 365, 337, 363);
//     segments.emplace_back(337, 363, 353, 370);
//     segments.emplace_back(353, 370, 354, 368);
//     segments.emplace_back(354, 368, 336, 362);
//     segments.emplace_back(336, 362, 335, 359);
//     segments.emplace_back(240, 329, 152, 333);
//     segments.emplace_back(152, 333, 224, 363);
//     segments.emplace_back(224, 363, 240, 329);
//     segments.emplace_back(412, 289, 407, 302);
//     segments.emplace_back(407, 302, 418, 301);
//     segments.emplace_back(418, 301, 439, 318);
//     segments.emplace_back(439, 318, 428, 339);
//     segments.emplace_back(428, 339, 421, 343);
//     segments.emplace_back(421, 343, 431, 348);
//     segments.emplace_back(431, 348, 458, 332);
//     segments.emplace_back(458, 332, 457, 331);
//     segments.emplace_back(457, 331, 457, 314);
//     segments.emplace_back(457, 314, 444, 317);
//     segments.emplace_back(444, 317, 420, 293);
//     segments.emplace_back(420, 293, 417, 291);
//     segments.emplace_back(417, 291, 412, 289);
//     segments.emplace_back(322, 316, 321, 320);
//     segments.emplace_back(321, 320, 318, 324);
//     segments.emplace_back(318, 324, 317, 334);
//     segments.emplace_back(317, 334, 322, 335);
//     segments.emplace_back(322, 335, 321, 321);
//     segments.emplace_back(321, 321, 322, 318);
//     segments.emplace_back(322, 318, 325, 317);
//     segments.emplace_back(325, 317, 322, 316);
//     segments.emplace_back(785, 271, 790, 279);
//     segments.emplace_back(790, 279, 793, 278);
//     segments.emplace_back(793, 278, 785, 271);
//     segments.emplace_back(568, 48, 584, 53);
//     segments.emplace_back(584, 53, 612, 79);
//     segments.emplace_back(612, 79, 614, 106);
//     segments.emplace_back(614, 106, 601, 114);
//     segments.emplace_back(601, 114, 572, 118);
//     segments.emplace_back(572, 118, 549, 97);
//     segments.emplace_back(549, 97, 550, 67);
//     segments.emplace_back(550, 67, 568, 48);
//     segments.emplace_back(347, 68, 356, 75);
//     segments.emplace_back(356, 75, 361, 101);
//     segments.emplace_back(361, 101, 350, 110);
//     segments.emplace_back(350, 110, 340, 113);
//     segments.emplace_back(340, 113, 314, 92);
//     segments.emplace_back(314, 92, 315, 82);
//     segments.emplace_back(315, 82, 347, 68);
    return segments;
}

int main() {
    // Preparing Input Geometries.
    int r;
    r = 1558617038;
    r = time(0);
    r = 1558618076;
    srand(r);
    printf("random seed: %d\n", r);
    arachne::logError("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    arachne::TimeKeeper tk;
    
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
