
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>

#include <boost/version.hpp>

#include <unordered_set>
#include <unordered_map>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "VoronoiQuadrangulation.h"
#include "DistributedBeadingStrategy.h"
#include "LimitedDistributedBeadingStrategy.h"
#include "utils/VoronoiUtils.h"
#include "NaiveBeadingStrategy.h"
#include "CenterDeviationBeadingStrategy.h"
#include "ConstantBeadingStrategy.h"

#include "TestGeometry/Pika.h"
#include "TestGeometry/Jin.h"
#include "TestGeometry/Moessen.h"

using arachne::Point;

namespace arachne
{

Polygons generateTestPoly(int size, Point border)
{
    Polygons polys;
    PolygonRef poly = polys.newPoly();
    for (int i = 0; i < size; i++)
    {
        poly.emplace_back(rand() % border.X, rand() % border.Y);
    }
    
    polys = polys.unionPolygons(Polygons(), ClipperLib::pftPositive);
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
static Polygons cross_shape;
static Polygons gMAT_example;
static Polygons wedge;
static Polygons flawed_wedge;
static Polygons rounded_wedge;
static Polygons flawed_wall;
static Polygons marked_local_opt;
static Polygons pikachu;
static Polygons jin;
static Polygons um;
static Polygons spikes;
static Polygons enclosed_region;

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
    Point3Matrix rot = Point3Matrix(PointMatrix(90.0)).compose(PointMatrix::scale(.7));
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
    
    {
        PolygonRef cross1 = cross_shape.newPoly();
        cross1.emplace_back(400, 0);
        cross1.emplace_back(400,400);
        cross1.emplace_back(0,400);
        cross1.emplace_back(0,500);
        cross1.emplace_back(400,500);
        cross1.emplace_back(400,900);
        cross1.emplace_back(500,900);
        cross1.emplace_back(500,500);
        cross1.emplace_back(900,500);
        cross1.emplace_back(900,400);
        cross1.emplace_back(500,400);
        cross1.emplace_back(500,0);
        PointMatrix scaler = PointMatrix::scale(13.8);
        cross1.applyMatrix(scaler);
    }

    {
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
        gMAT_example.applyMatrix(PointMatrix::scale(1.0));
    }

    {
        PolygonRef wedge_1 = wedge.newPoly();
        wedge_1.emplace_back(2500, 0);
        wedge_1.emplace_back(0, 2500);
        wedge_1.emplace_back(20000, 20000);
//         PointMatrix scaler = PointMatrix::scale(.846); // .846 causes a transition which is just beyond the marked skeleton
        PointMatrix scaler = PointMatrix::scale(.5); // .846 causes a transition which is just beyond the marked skeleton
        wedge_1.applyMatrix(scaler);
        PointMatrix rot(-45);
        wedge_1.applyMatrix(rot);
    }

    rounded_wedge = wedge.offset(-400, ClipperLib::jtRound).offset(400, ClipperLib::jtRound); // TODO: this offset gives problems!!
//     rounded_wedge = wedge.offset(-200, ClipperLib::jtRound).offset(200, ClipperLib::jtRound); // TODO: this offset also gives problems!!
//     rounded_wedge = wedge.offset(-205, ClipperLib::jtRound).offset(205, ClipperLib::jtRound);
    
    {
        coord_t l = 20000;
        coord_t h = 2000;
        coord_t r = 200;
        coord_t step = 2000;
        PolygonRef flawed_wedgel_1 = flawed_wedge.newPoly();
        for (coord_t x = 0; x <= l; x += step)
        {
            flawed_wedgel_1.emplace_back(x, (h + rand() % r - r/2) * x / l);
        }
        for (coord_t x = l - step / 2; x >= 0; x -= 800)
        {
            flawed_wedgel_1.emplace_back(x, (rand() % r - r/2) * x / l);
        }
        
        Point3Matrix rot = Point3Matrix(PointMatrix(-90.0));
        flawed_wedgel_1.applyMatrix(rot);
    }
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
    
    pikachu = generatePika();
    
    jin = generateJin();
    
    {
        coord_t r = 3600;
        coord_t inr = 2400;
        coord_t b = 10000;
        coord_t h = b;
        PolygonRef um_1 = um.newPoly();
        um_1.emplace_back(r, r);
        um_1.emplace_back(r, h);
        um_1.emplace_back(b - r, h);
        um_1.emplace_back(b - r, r);
        um = um.offset(r, ClipperLib::jtRound);
        Polygon bb;
        bb.emplace_back(inr, inr);
        bb.emplace_back(inr, h + inr);
        bb.emplace_back(b - inr, h + inr);
        bb.emplace_back(b - inr, inr);
        Polygons bs;
        bs.add(bb);
        um = um.difference(bs);
        Polygon a;
        a.emplace_back(-r, h);
        a.emplace_back(-r, h + 2 * r);
        a.emplace_back(b + r, h + 2 * r);
        a.emplace_back(b + r, h);
        Polygons as;
        as.add(a);
        um = um.difference(as);
    }
    if (false)
    {
        coord_t min_r = 3000;
        coord_t max_r = 8000;
        Polygons circles;
        PolygonUtils::makeCircle(Point(0,0), 1600, circles);
        for (int a = 0; a < 360; a += 360 / 10)
        {
            Polygons dot;
            coord_t r = min_r + (max_r - min_r) * a / 360;
            PolygonUtils::makeCircle(Point(-r * cos(a /180.0 * M_PI), r * sin(a /180.0 * M_PI)), 10, dot);
            dot = dot.unionPolygons(circles);
            dot = dot.approxConvexHull();
            spikes = spikes.unionPolygons(dot);
        }
    }
    {
        PolygonRef outline = enclosed_region.newPoly();
        outline.emplace_back(0, 0);
        outline.emplace_back(0, 10000);
        outline.emplace_back(10000, 10000);
        outline.emplace_back(10000, 0);
        PolygonRef hole = enclosed_region.newPoly();
        hole.emplace_back(1000, 1000);
        hole.emplace_back(9000, 1000);
        hole.emplace_back(9000, 9000);
        hole.emplace_back(1000, 9000);
        PolygonRef outline2 = enclosed_region.newPoly();
        outline2.emplace_back(2000, 2000);
        outline2.emplace_back(2000, 8000);
        outline2.emplace_back(8000, 8000);
        outline2.emplace_back(8000, 2000);
        PolygonRef hole2 = enclosed_region.newPoly();
        hole2.emplace_back(4000, 4000);
        hole2.emplace_back(6000, 4000);
        hole2.emplace_back(6000, 6000);
        hole2.emplace_back(4000, 6000);

        PointMatrix rot = PointMatrix(170.0);
        enclosed_region.applyMatrix(rot);
    }
}

void test()
{
    
    // Preparing Input Geometries.
    int r;
    r = time(0);
    r = 123;
//     r = 1563034632; // problem with generateTestPoly(20, Point(10000, 10000));
//     r = 1563020645;
//     r = 1562934206;
//     r = 1558617038;
//     r = 1558618076;
//     r = 1558692831;
//     r = 1558983814;
//     r = 1558985782;
//     r = 1559215562;
//     r = 1559224125;
//     r = 1559224469;
//     r = 68431;
//     r = 1559234570;
//     r = 1559564752;
//     r = 1559566333;
//     r = 1559568483;
//     r = 1559579388;
//     r = 1559580888;
    srand(r);
    printf("r = %d;\n", r);
    fflush(stdout);
    logDebug("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    generateTestPolys();
//     Polygons polys = generateTestPoly(20, Point(10000, 10000));
//     Polygons polys = test_poly_1;
//     Polygons polys = parabola_dip;
//     Polygons polys = squares;
//     Polygons polys = circle;
//     Polygons polys = circle_flawed;
//     Polygons polys = cross_shape;
//     Polygons polys = gMAT_example;
//     Polygons polys = wedge;
//     Polygons polys = flawed_wedge;
//     Polygons polys = flawed_wall;
//     Polygons polys = marked_local_opt;
//     Polygons polys = pikachu;
//     Polygons polys = um;
//     Polygons polys = spikes;
//     Polygons polys = enclosed_region;
//     Polygons polys = jin;
    Polygons polys = MoessenTests::generateCircles(Point(3, 3), 100, 400, 500, 8);
//     Polygons polys = MoessenTests::generateTriangles(Point(6, 3), 100, 400, 500);

    polys = polys.unionPolygons();

#ifdef DEBUG
    {
        SVG svg("output/outline.svg", AABB(Point(0,0), Point(10000, 10000)));
        svg.writePolygons(polys);
    }
#endif

    TimeKeeper tk;

    VoronoiQuadrangulation vq(polys);

    DistributedBeadingStrategy beading_strategy(300, 400, 600, M_PI / 4);
//     LimitedDistributedBeadingStrategy beading_strategy(300, 400, 600, 6, M_PI / 6);
//     NaiveBeadingStrategy beading_strategy(400);
//     ConstantBeadingStrategy beading_strategy(400, 4);
//     CenterDeviationBeadingStrategy beading_strategy(400, .5, 1.7);
    std::vector<ExtrusionSegment> segments = vq.generateToolpaths(beading_strategy);
    logError("Processing took %fs\n", tk.restart());


    Polygons insets;
    Polygons last_inset = polys.offset(-200);
    while (!last_inset.empty())
    {
        insets.add(last_inset);
        last_inset = last_inset.offset(-400, ClipperLib::jtRound);
    }
    logError("Naive processing took %fs\n", tk.restart());

#ifdef DEBUG
    logAlways("Generating SVGs...\n");
    Polygons paths;
    for (ExtrusionSegment& segment : segments)
    {
        PolygonRef poly = paths.newPoly();
        poly.emplace_back(segment.from);
        poly.emplace_back(segment.to);
    }

    {
        SVG svg("output/after.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::GRAY, 2);
        vq.debugOutput(svg, false, false, true);
        svg.writePolygons(paths, SVG::Color::BLACK, 2);
    }
    
    {
        std::ofstream csv("output/segments.csv", std::ofstream::out | std::ofstream::trunc);
        csv << "from_x; from_y; from_width; to_x; to_y; to_width\n";
        for (const ExtrusionSegment& segment : segments)
            csv << segment.from.X << "; " << segment.from.Y << "; " << segment.from_width << "; " << segment.to.X << "; " << segment.to.Y << "; " << segment.to_width << '\n';
        csv.close();
    }
    {
        SVG svg("output/toolpath_locations.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLACK, 2);
        for (auto poly : paths)
            for (Point p : poly)
                svg.writePoint(p, true, 1);
    }
    Polygons area_covered;
    Polygons overlaps;
    std::unordered_set<Point> points_visited;
    for (ExtrusionSegment& segment : segments)
    {
        if (segment.from == segment.to)
        {
            continue;
        }
//         segment.from_width = segment.from_width * 4 / 5;
//         segment.to_width = segment.to_width * 4 / 5;
        area_covered = area_covered.unionPolygons(segment.toPolygons());
        Polygons extruded = segment.toPolygons();
        Polygons reduction;
        if (points_visited.count(segment.from) > 0)
        {
            PolygonUtils::makeCircle(segment.from, segment.from_width / 2, reduction);
        }
        if (points_visited.count(segment.to) > 0)
        {
            PolygonUtils::makeCircle(segment.to, segment.to_width / 2, reduction);
        }
        extruded = extruded.difference(reduction);
        overlaps.add(extruded);
        points_visited.emplace(segment.from);
        points_visited.emplace(segment.to);
    }
    {
        SVG svg("output/toolpaths.svg", AABB(polys));
        for (PolygonRef poly : overlaps)
        {
            svg.writeAreas(poly, SVG::Color::GRAY);
        }
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    {
        SVG svg("output/overlaps.svg", AABB(polys));
        svg.writeAreas(overlaps.xorPolygons(area_covered), SVG::Color::GRAY);
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    {
        SVG svg("output/overlaps2.svg", AABB(polys));
        svg.writeAreas(overlaps.unionPolygons(), SVG::Color::GRAY);
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    {
        SVG svg("output/total_area.svg", AABB(polys));
        svg.writeAreas(area_covered, SVG::Color::GRAY);
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    
    {
        SVG svg("output/normal.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::RED, 2);
        for (PolygonRef poly : insets)
        {
            Point prev = poly.back();
            for (Point p : poly)
            {
                ExtrusionSegment segment(prev, 400, p, 400);
                svg.writeAreas(segment.toPolygons(), SVG::Color::GRAY);;
                prev = p;
            }
        }
        svg.writePolygons(insets, SVG::Color::BLUE, 2);
    }

    logError("Writing output files took %fs\n", tk.restart());
#endif // DEBUG
}


} // namespace arachne


int main() {
    long n = 1;
    for (int i = 0; i < n; i++)
    {
        arachne::test();
        if (++i % std::max(1l, n / 100) == 0)
            std::cerr << (i / 100) << "%\n";
    }
    return 0;
}
