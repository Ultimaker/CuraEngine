
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <sstream>
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
#include "InwardDistributedBeadingStrategy.h"
#include "LimitedDistributedBeadingStrategy.h"
#include "utils/VoronoiUtils.h"
#include "NaiveBeadingStrategy.h"
#include "CenterDeviationBeadingStrategy.h"
#include "ConstantBeadingStrategy.h"
#include "BeadingOrderOptimizer.h"
#include "GcodeWriter.h"
#include "Statistics.h"

#include "TestGeometry/Pika.h"
#include "TestGeometry/Jin.h"
#include "TestGeometry/Moessen.h"
#include "TestGeometry/Prescribed.h"
#include "TestGeometry/Spiky.h"
#include "TestGeometry/SVGloader.h"

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

void test(std::string input_outline_filename, std::string output_prefix)
{
    
    // Preparing Input Geometries.
    int r;
    r = time(0);
    r = 123;
//     r = 1563034632; // problem with generateTestPoly(20, Point(10000, 10000));
    srand(r);
    printf("r = %d;\n", r);
    fflush(stdout);
    logDebug("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    generateTestPolys();
    Polygons polys = SVGloader::load(input_outline_filename);
    AABB aabb(polys);
    polys.applyMatrix(Point3Matrix::translate(aabb.min * -1));

//     Polygons polys = generateTestPoly(40, Point(20000, 20000));
//     r = 1563833579; srand(r); Polygons polys = generateTestPoly(40, Point(20000, 20000)); // some overlapping regions where there shouldn't be

//     Polygons polys = test_poly_1;
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
//     Polygons polys = MoessenTests::generateCircles(Point(3, 3), 100, 400, 500, 52);
//     Polygons polys = MoessenTests::generateCircles(Point(2, 2), 100, 400, 500, 8);
//     r = 1563874501; Polygons polys = MoessenTests::generateCircles(Point(3, 3), 100, 400, 1000, 8);
//     Polygons polys = MoessenTests::generateTriangles(Point(4, 2), 100, 600, 1000);
//     Polygons polys = MoessenTests::generateTriangles(Point(4, 2), 300, 301, 1000);
//     Polygons polys = MoessenTests::generateTriangles(Point(4, 2), 400, 401, 1000);
//     Polygons polys = Prescribed::fromDistances({Point(0,800), Point(400,300), Point(610,610), Point(1400, 200)});
//     Polygons polys = Spiky::oneSpike(200);
//     Polygons polys = Spiky::twoSpikes();
//     Polygons polys = Spiky::fourSpikes();
//     Polygons polys = Spiky::doubleOutSpike(800, 380);

    polys = polys.unionPolygons();
    polys.simplify();

    {
        SVG svg("output/outline.svg", AABB(polys));
        svg.writeAreas(polys, SVG::Color::NONE, SVG::Color::BLACK);
    }

    TimeKeeper tk;

    float transitioning_angle = M_PI / 4;
    VoronoiQuadrangulation vq(polys, transitioning_angle);

//     DistributedBeadingStrategy beading_strategy(400, transitioning_angle);
    InwardDistributedBeadingStrategy beading_strategy(400, transitioning_angle);
//     LimitedDistributedBeadingStrategy beading_strategy(300, 400, 600, 6, transitioning_angle);
//     NaiveBeadingStrategy beading_strategy(400);
//     ConstantBeadingStrategy beading_strategy(400, 4);
//     CenterDeviationBeadingStrategy beading_strategy(400, .5, 1.7);
    std::vector<ExtrusionSegment> segments = vq.generateToolpaths(beading_strategy);

    std::vector<std::vector<std::vector<ExtrusionJunction>>> result_polygons_per_index;
    std::vector<std::vector<std::vector<ExtrusionJunction>>> result_polylines_per_index;
    BeadingOrderOptimizer::optimize(segments, result_polygons_per_index, result_polylines_per_index);
    logAlways("Processing took %fs\n", tk.restart());

    {
        GcodeWriter gcode("output/arachne_P3.gcode", GcodeWriter::type_P3);
        gcode.print(result_polygons_per_index, result_polylines_per_index, AABB(polys));
    }

    {
        GcodeWriter gcode("output/arachne_UM3.gcode", GcodeWriter::type_UM3);
        gcode.print(result_polygons_per_index, result_polylines_per_index, AABB(polys));
    }

    {
        STLwriter stl("output/mat.stl");
        vq.debugOutput(stl);
    }

    logAlways("Writing gcode took %fs\n", tk.restart());


    logAlways("Analysing...\n");

    Statistics stats("distributed", output_prefix);
    stats.analyse(polys, result_polygons_per_index, result_polylines_per_index, &vq);
    logAlways("Analysis took %fs\n", tk.restart());
    logAlways("Visualizing...\n");
    stats.visualize();
    logAlways("Visualization took %fs\n", tk.restart());


    {
        logAlways("Simulating naive method...\n");

        coord_t nozzle_size = 400;
        
        std::vector<Polygons> insets;
        Polygons last_inset = polys.offset(-nozzle_size / 2, ClipperLib::jtRound);
        while (!last_inset.empty())
        {
            insets.emplace_back(last_inset);
            last_inset = last_inset.offset(-nozzle_size, ClipperLib::jtRound);
        }
        logAlways("Naive processing took %fs\n", tk.restart());

        std::vector<std::vector<std::vector<ExtrusionJunction>>> result_polygons_per_index;
        std::vector<std::vector<std::vector<ExtrusionJunction>>> result_polylines_per_index;
        result_polygons_per_index.resize(insets.size());
        for (coord_t inset_idx = 0; inset_idx < insets.size(); inset_idx++)
        {
            for (PolygonRef poly : insets[inset_idx])
            {
                result_polygons_per_index[inset_idx].emplace_back();
                std::vector<ExtrusionJunction>& junction_poly = result_polygons_per_index[inset_idx].back();
                for (Point p : poly)
                {
                    junction_poly.emplace_back(p, nozzle_size, inset_idx);
                }
            }
        }
    
    
        {
            GcodeWriter gcode("output/naive_P3.gcode", GcodeWriter::type_P3);
            gcode.print(result_polygons_per_index, result_polylines_per_index, AABB(polys));
        }

        {
            GcodeWriter gcode("output/naive_UM3.gcode", GcodeWriter::type_UM3);
            gcode.print(result_polygons_per_index, result_polylines_per_index, AABB(polys));
        }
        
        logAlways("Analysing...\n");

        Statistics stats("naive", output_prefix);
        stats.analyse(polys, result_polygons_per_index, result_polylines_per_index);
        logAlways("Analysis took %fs\n", tk.restart());
        stats.visualize();

        logAlways("Visualization took %fs\n", tk.restart());
    
        
    }
}


} // namespace arachne

int main(int argc, char *argv[])
{
    std::string input_outline_filename;
    std::string output_prefix;
    if (argc >= 2) input_outline_filename = argv[1];
    if (argc >= 3) output_prefix = argv[2];
    long n = 1;
    for (int i = 0; i < n; i++)
    {
        arachne::test(input_outline_filename, output_prefix);
        if (++i % std::max(1l, n / 100) == 0)
            std::cerr << (i / 100) << "%\n";
    }
    return 0;
}
