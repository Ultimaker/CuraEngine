
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>

#include <boost/version.hpp>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "VoronoiQuadrangulation.h"
#include "DistributedBeadingStrategy.h"
#include "utils/VoronoiUtils.h"

using arachne::Point;

namespace arachne
{

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
    
//     polys = polys.offset(-10, ClipperLib::jtRound);
//     polys = polys.offset(20, ClipperLib::jtRound);
//     polys = polys.offset(-10, ClipperLib::jtRound);
    polys = polys.offset(-border.X/200, ClipperLib::jtRound);
    polys = polys.offset(border.X/100, ClipperLib::jtRound);
    polys = polys.offset(-border.X/200, ClipperLib::jtRound);
    polys = polys.unionPolygons();
    return polys;
}

static Polygons test_poly_1;
static Polygons squares;
static Polygons parabola_dip;
static Polygons circle;
static Polygons circle_flawed;
static Polygons gMAT_example;
static Polygons wedge;
static Polygons rounded_wedge;
static Polygons flawed_wall;
static Polygons marked_local_opt;

void generateTestPolys()
{
    PolygonRef poly = test_poly_1.newPoly();
    poly.emplace_back(0, 0);
    poly.emplace_back(10000, 0);
    poly.emplace_back(5000, 1000);
    poly.emplace_back(4000, 2000);
    poly.emplace_back(3000, 5000);
    poly.emplace_back(2000, 6000);
    poly.emplace_back(1000, 5000);
    poly.emplace_back(0, 3000);
    PolygonRef hole = test_poly_1.newPoly();
    hole.emplace_back(1000,1000);
    hole.emplace_back(1100,900);
    hole.emplace_back(1000,900);

    PolygonRef square_1 = squares.newPoly();
    square_1.emplace_back(0, 0);
    square_1.emplace_back(0, 10000);
    square_1.emplace_back(10000, 10000);
    square_1.emplace_back(10000, 0);
    
    PolygonRef parabola_dip_1 = parabola_dip.newPoly();
    parabola_dip_1.emplace_back(0, 10000);
    parabola_dip_1.emplace_back(0, 0);
    parabola_dip_1.emplace_back(10000, 0);
    parabola_dip_1.emplace_back(10000, 10000);
    parabola_dip_1.emplace_back(5500, 10000);
    parabola_dip_1.emplace_back(5000, 5000);
    parabola_dip_1.emplace_back(4500, 10000);
    Point3Matrix rot = Point3Matrix(PointMatrix(25.0)).compose(PointMatrix::scale(.7));
    parabola_dip_1.applyMatrix(rot);
    
    PolygonRef circle_1 = circle.newPoly();
    coord_t r = 10000;
    for (float a = 0; a < 360; a += 10)
    {
        float rad = a / 180 * M_PI;
        circle_1.emplace_back(r * cos(rad), r * sin(rad));
    }
    
    PolygonRef circle_flawed_1 = circle_flawed.newPoly();
    for (float a = 0; a < 360; a += 10)
    {
        r = 5000 + rand() % 2500;
        a += (rand() % 100) / 50.0;
        float rad = a / 180 * M_PI;
        circle_flawed_1.emplace_back(r * cos(rad), r * sin(rad));
    }

    PolygonRef gMAT_example_outline = gMAT_example.newPoly();
    gMAT_example_outline.emplace_back(0, 0);
    gMAT_example_outline.emplace_back(8050, 0);
    gMAT_example_outline.emplace_back(8050, 2000);
    gMAT_example_outline.emplace_back(7000, 2000);
    gMAT_example_outline.emplace_back(7000, 11500);
    gMAT_example_outline.emplace_back(6500, 12000);
    gMAT_example_outline.emplace_back(0, 12000);
    PolygonRef gMAT_example_triangle = gMAT_example.newPoly();
    gMAT_example_triangle.emplace_back(1000, 7000);
    gMAT_example_triangle.emplace_back(1000, 11000);
    gMAT_example_triangle.emplace_back(4000, 9000);
    PolygonRef gMAT_example_round = gMAT_example.newPoly();
    gMAT_example_round.emplace_back(1000, 3000);
    gMAT_example_round.emplace_back(1000, 5000);
    gMAT_example_round.emplace_back(2000, 6000);
    gMAT_example_round.emplace_back(5000, 6000);
    gMAT_example_round.emplace_back(5000, 3000);
    gMAT_example_round.emplace_back(4000, 2000);
    gMAT_example_round.emplace_back(2000, 2000);

    PolygonRef wedge_1 = wedge.newPoly();
    wedge_1.emplace_back(2500, 0);
    wedge_1.emplace_back(0, 2500);
    wedge_1.emplace_back(20000, 20000);
    PointMatrix scaler = PointMatrix::scale(.846 / 2); // .846 causes a transition which is just beyond the marked skeleton
    wedge_1.applyMatrix(scaler);

    rounded_wedge = wedge.offset(-400, ClipperLib::jtRound).offset(400, ClipperLib::jtRound); // TODO: this offset gives problems!!
//     rounded_wedge = wedge.offset(-200, ClipperLib::jtRound).offset(200, ClipperLib::jtRound); // TODO: this offset also gives problems!!
//     rounded_wedge = wedge.offset(-205, ClipperLib::jtRound).offset(205, ClipperLib::jtRound);
    
    {
        coord_t l = 10000;
        coord_t h = 1000;
        coord_t r = 100;
        coord_t step = 2000;
        PolygonRef flawed_wall_1 = flawed_wall.newPoly();
        for (coord_t x = 0; x <= l; x += step)
        {
            flawed_wall_1.emplace_back(x, h + rand() % r - r/2);
        }
        for (coord_t x = l - step / 2; x >= 0; x -= 800)
        {
            flawed_wall_1.emplace_back(x, rand() % r - r/2);
        }
        
        Point3Matrix rot = Point3Matrix(PointMatrix(60.0));
        flawed_wall_1.applyMatrix(rot);
    }
    {
        PolygonRef marked_local_opt_1 = marked_local_opt.newPoly();
        marked_local_opt_1.emplace_back(5000, 0);
        marked_local_opt_1.emplace_back(0, 400);
        marked_local_opt_1.emplace_back(5000, 610);
        marked_local_opt_1.emplace_back(10000, 400);
        Point3Matrix rot = Point3Matrix(PointMatrix(60.0));
        marked_local_opt_1.applyMatrix(rot);
    }
}

void test()
{
    
    // Preparing Input Geometries.
    int r;
    r = 1558617038;
    r = time(0);
//     r = 1558618076;
//     r = 1558692831;
//     r = 1558983814;
//     r = 1558985782;
//     r = 1559215562;
//     r = 1559224125;
//     r = 1559224469;
//     r = 68431;
    r = 1559234570;
    srand(r);
    printf("r = %d;\n", r);
    fflush(stdout);
    logDebug("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    generateTestPolys();
//     Polygons polys = generateTestPoly(6, Point(10000, 10000));
//     Polygons polys = test_poly_1;
//     Polygons polys = parabola_dip;
//     Polygons polys = squares;
//     Polygons polys = circle;
//     Polygons polys = circle_flawed;
//     Polygons polys = gMAT_example;
//     Polygons polys = wedge;
    Polygons polys = flawed_wall;
//     Polygons polys = marked_local_opt;
    polys = polys.unionPolygons();
    {
        SVG svg("output/outline.svg", AABB(Point(0,0), Point(10000, 10000)));
        svg.writePolygons(polys);
    }
    
    
    TimeKeeper tk;
    
    VoronoiQuadrangulation vq(polys);

    DistributedBeadingStrategy beading_strategy(300, 400, 600);
    Polygons paths = vq.generateToolpaths(beading_strategy);

    SVG svg("output/after.svg", AABB(polys));
    svg.writePolygons(polys, SVG::Color::GRAY, 2);
    vq.debugOutput(svg, false, false, true);
    svg.writePolygons(paths, SVG::Color::BLACK, 2);
    
    logError("Total processing took %fs\n", tk.restart());
}


} // namespace arachne


int main() {
    arachne::test();
    return 0;
}
