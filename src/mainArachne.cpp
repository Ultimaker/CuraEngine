
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
static Polygons parabola_dip;
static Polygons circle;
static Polygons circle_flawed;
static Polygons gMAT_example;

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

    PolygonRef parabola_dip_1 = parabola_dip.newPoly();
    parabola_dip_1.emplace_back(0, 1000);
    parabola_dip_1.emplace_back(0, 0);
    parabola_dip_1.emplace_back(1000, 0);
    parabola_dip_1.emplace_back(1000, 1000);
    parabola_dip_1.emplace_back(550, 1000);
    parabola_dip_1.emplace_back(500, 500);
    parabola_dip_1.emplace_back(450, 1000);
    PointMatrix rot(25.0);
    for (Point& p : parabola_dip_1)
    {
        p = rot.apply(p);
    }
    
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
        r = 10000 + rand() % 5000;
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
    srand(r);
    logError("    r = %d;\n", r);
    logError("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    generateTestPolys();
//     Polygons polys = generateTestPoly(6, Point(10000, 10000));
//     Polygons polys = test_poly_1;
//     Polygons polys = parabola_dip;
//     Polygons polys = circle;
//     Polygons polys = circle_flawed;
    Polygons polys = gMAT_example;
    {
        SVG svg("output/outline.svg", AABB(Point(0,0), Point(10000, 10000)));
        svg.writePolygons(polys);
    }
    
    
    TimeKeeper tk;
    
    VoronoiQuadrangulation vq(polys);
    
    logError("Toal processing took %fs\n", tk.restart());
}


} // namespace arachne


int main() {
    arachne::test();
    return 0;
}
