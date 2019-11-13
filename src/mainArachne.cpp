
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <sstream>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>
#include <algorithm> // random_shuffle

#include <boost/version.hpp>

#include <unordered_set>
#include <unordered_map>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "SkeletalTrapezoidation.h"
#include "DistributedBeadingStrategy.h"
#include "InwardDistributedBeadingStrategy.h"
#include "LimitedDistributedBeadingStrategy.h"
#include "LimitedBeadingStrategy.h"
#include "SingleBeadBeadingStrategy.h"
#include "utils/VoronoiUtils.h"
#include "NaiveBeadingStrategy.h"
#include "CenterDeviationBeadingStrategy.h"
#include "OutlineAccuracyBeadingStrategy.h"
#include "WideningBeadingStrategy.h"
#include "ConstantBeadingStrategy.h"
#include "BeadingOrderOptimizer.h"
#include "GcodeWriter.h"
#include "Statistics.h"

#include "TestGeometry/TestPolys.h"
#include "TestGeometry/Pika.h"
#include "TestGeometry/Jin.h"
#include "TestGeometry/Moessen.h"
#include "TestGeometry/Prescribed.h"
#include "TestGeometry/Spiky.h"
#include "TestGeometry/SVGloader.h"
#include "TestGeometry/Microstructure.h"

#include "TestGeometry/VariableWidthGcodeTester.h"

using arachne::Point;

namespace arachne
{

enum class StrategyType
{
    Naive,
    NaiveStrategy,
    Constant,
    Center,
    Distributed,
    InwardDistributed,
    LimitedDistributed,
    SingleBead,
    OutlineAccuracy,
    COUNT
};

std::string to_string(StrategyType type)
{
    switch (type)
    {
        case StrategyType::Naive: return "Naive";
        case StrategyType::NaiveStrategy: return "NaiveStrategy";
        case StrategyType::Constant: return "Constant";
        case StrategyType::Center: return "Center";
        case StrategyType::Distributed: return "Distributed";
        case StrategyType::InwardDistributed: return "InwardDistributed";
        case StrategyType::LimitedDistributed: return "LimitedDistributed";
        case StrategyType::SingleBead: return "SingleBead";
        case StrategyType::OutlineAccuracy: return "OutlineAccuracy";
        default: return "unknown_strategy";
    }
}

BeadingStrategy* makeStrategy(StrategyType type, coord_t prefered_bead_width = MM2INT(0.5), float transitioning_angle = M_PI / 4, bool widening = false)
{
    BeadingStrategy* ret = nullptr;
    switch (type)
    {
        case StrategyType::NaiveStrategy:      ret = new NaiveBeadingStrategy(prefered_bead_width);                                      break;
        case StrategyType::Constant:           ret = new ConstantBeadingStrategy(prefered_bead_width, 4, .99 * M_PI);                    break;
        case StrategyType::Center:             ret = new CenterDeviationBeadingStrategy(prefered_bead_width, transitioning_angle);       break;
        case StrategyType::Distributed:        ret = new DistributedBeadingStrategy(prefered_bead_width, transitioning_angle);           break;
        case StrategyType::InwardDistributed:  ret = new InwardDistributedBeadingStrategy(prefered_bead_width, transitioning_angle);     break;
        case StrategyType::LimitedDistributed: ret = new LimitedDistributedBeadingStrategy(prefered_bead_width, 6, transitioning_angle); break;
        case StrategyType::SingleBead:         ret = new SingleBeadBeadingStrategy(prefered_bead_width, transitioning_angle);            break;
        case StrategyType::OutlineAccuracy:    ret = new LimitedBeadingStrategy(6, new OutlineAccuracyBeadingStrategy(prefered_bead_width, prefered_bead_width * 3 / 4, prefered_bead_width / 2, transitioning_angle)); break;
        default:
            logError("Cannot make strategy!\n");
            return nullptr;
    }
    if (widening)
    {
        return new WideningBeadingStrategy(ret, MM2INT(0.05), MM2INT(0.3));
    }
    else
    {
        return ret;
    }
}

void test(Polygons& polys, coord_t nozzle_size, std::string output_prefix, StrategyType type, bool generate_gcodes = true, bool analyse = false, bool generate_MAT_STL = false)
{
    std::string type_str = to_string(type);
    logAlways(">> Performing %s strategy...\n", type_str.c_str());
    float transitioning_angle = M_PI / 4; // = 180 - the "limit bisector angle" from the paper

    BeadingStrategy* beading_strategy = makeStrategy(type, nozzle_size, transitioning_angle);
    if (!beading_strategy) return;

    BeadingStrategy::checkTranisionThicknessConsistency(beading_strategy);

    TimeKeeper tk;

    coord_t discretization_step_size = 200;
    coord_t transition_filter_dist = 1000;
    coord_t beading_propagation_transition_dist = 400;
    bool reduce_overlapping_segments = true;
    bool filter_outermost_marked_edges = false;
    if (type == StrategyType::SingleBead)
    {
        transition_filter_dist = 50;
        reduce_overlapping_segments = false;
    }
    else if (type == StrategyType::Constant)
    {
        filter_outermost_marked_edges = true;
    }
    SkeletalTrapezoidation st(polys, transitioning_angle, discretization_step_size, transition_filter_dist, beading_propagation_transition_dist);

    std::vector<std::list<ExtrusionLine>> result_polylines_per_index = st.generateToolpaths(*beading_strategy, filter_outermost_marked_edges);

    std::vector<std::list<ExtrusionLine>> result_polygons_per_index;
    BeadingOrderOptimizer::optimize(result_polygons_per_index, result_polylines_per_index, reduce_overlapping_segments);
    double processing_time = tk.restart();
    logAlways("Processing took %fs\n", processing_time);

    if (generate_gcodes)
    {
        AABB aabb(polys);
        {
            std::ostringstream ss;
            ss << "output/" << output_prefix << "_" << to_string(type) << "_arachne_P3.gcode";
            GcodeWriter gcode(ss.str(), GcodeWriter::type_P3);
            gcode.printBrim(aabb, 3, nozzle_size, nozzle_size * 1.5);
            gcode.resetPrintTime();
            gcode.print(result_polygons_per_index, result_polylines_per_index, aabb);
//             std::cerr << "P3 Print time: " << gcode.getPrintTime() << "\n";
        }
//         if (false)
        {
            std::ostringstream ss;
            ss << "output/" << output_prefix << "_" << to_string(type) << "_arachne_UM3.gcode";
            GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3);
            gcode.printBrim(aabb, 3, nozzle_size, nozzle_size * 1.5);
            gcode.resetPrintTime();
            gcode.print(result_polygons_per_index, result_polylines_per_index, aabb);
            Statistics stats(to_string(type), output_prefix, polys, processing_time);
            stats.savePrintTimeCSV(gcode.getPrintTime());
            logAlways("Writing gcode took %fs\n", tk.restart());
        }
    }

    if (generate_MAT_STL)
    {
        {
            STLwriter stl("output/st_bead_count.stl");
            st.debugOutput(stl, true);
        }
        logAlways("Writing MAT STL took %fs\n", tk.restart());
    }

    if (analyse)
    {
        Statistics stats(to_string(type), output_prefix, polys, processing_time);
        stats.analyse(result_polygons_per_index, result_polylines_per_index, &st);
        logAlways("Analysis took %fs\n", tk.restart());
        stats.saveResultsCSV();
        stats.visualize(nozzle_size, true);
        logAlways("Visualization took %fs\n", tk.restart());
    }

    delete beading_strategy;

}

void testNaive(Polygons& polys, coord_t nozzle_size, std::string output_prefix, bool generate_gcodes = false, bool analyse = false)
{
    logAlways(">> Simulating naive method...\n");

    TimeKeeper tk;

    std::vector<Polygons> insets;
    Polygons last_inset = polys.offset(-nozzle_size / 2, ClipperLib::jtRound);
    while (!last_inset.empty())
    {
        insets.emplace_back(last_inset);
        last_inset = last_inset.offset(-nozzle_size, ClipperLib::jtRound);
    }
    double processing_time = tk.restart();
    logAlways("Naive processing took %fs\n", processing_time);

    std::vector<std::list<ExtrusionLine>> result_polygons_per_index;
    std::vector<std::list<ExtrusionLine>> result_polylines_per_index;
    result_polygons_per_index.resize(insets.size());
    for (coord_t inset_idx = 0; inset_idx < insets.size(); inset_idx++)
    {
        for (PolygonRef poly : insets[inset_idx])
        {
            constexpr bool is_odd = false;
            result_polygons_per_index[inset_idx].emplace_back(inset_idx, is_odd);
            ExtrusionLine& junction_poly = result_polygons_per_index[inset_idx].back();
            for (Point p : poly)
            {
                junction_poly.junctions.emplace_back(p, nozzle_size, inset_idx);
            }
        }
    }

    if (generate_gcodes)
    {
        AABB aabb(polys);
        {
            std::ostringstream ss;
            ss << "output/" << output_prefix << "_naive_arachne_P3.gcode";
            GcodeWriter gcode(ss.str(), GcodeWriter::type_P3);
            gcode.printBrim(aabb, 3, nozzle_size, nozzle_size * 1.5);
            gcode.resetPrintTime();
            gcode.print(result_polygons_per_index, result_polylines_per_index, aabb);
//             std::cerr << "P3 Print time: " << gcode.getPrintTime() << "\n";
        }
//         if (false)
        {
            std::ostringstream ss;
            ss << "output/" << output_prefix << "_naive_arachne_UM3.gcode";
            GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3);
            gcode.printBrim(aabb, 3, nozzle_size, nozzle_size * 1.5);
            gcode.resetPrintTime();
            gcode.print(result_polygons_per_index, result_polylines_per_index, aabb);
            Statistics stats("naive", output_prefix, polys, processing_time);
            stats.savePrintTimeCSV(gcode.getPrintTime());
            logAlways("Writing gcodes took %fs\n", tk.restart());
        }
    }
    
    if (analyse)
    {
        Statistics stats("naive", output_prefix, polys, processing_time);
        stats.analyse(result_polygons_per_index, result_polylines_per_index);
        stats.saveResultsCSV();
        logAlways("Analysis took %fs\n", tk.restart());
        stats.visualize(nozzle_size);
        logAlways("Visualization took %fs\n", tk.restart());
    }
    
}

void writeVarWidthTest()
{
    std::vector<std::list<ExtrusionLine>> result_polygons_per_index;
    std::vector<std::list<ExtrusionLine>> result_polylines_per_index;
    result_polylines_per_index = VariableWidthGcodeTester::zigzag();

    AABB aabb;
    for (auto ps : result_polylines_per_index)
        for (auto p : ps)
            for (ExtrusionJunction& j : p.junctions)
                aabb.include(j.p);
    Polygons fake_outline; fake_outline.add(aabb.toPolygon());
    
        
    {
        std::ostringstream ss;
        ss << "output/variable_width_test_P3.gcode";
        GcodeWriter gcode(ss.str(), GcodeWriter::type_P3, 200);
        gcode.printBrim(aabb, 3);
        gcode.resetPrintTime();
        gcode.print(result_polygons_per_index, result_polylines_per_index, aabb);
//         std::cerr << "P3 Print time: " << gcode.getPrintTime() << "\n";
    }
//     if (false)
    {
        std::ostringstream ss;
        ss << "output/variable_width_test_UM3.gcode";
        GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3, 200);
        gcode.printBrim(aabb, 3);
        gcode.resetPrintTime();
        gcode.print(result_polygons_per_index, result_polylines_per_index, aabb);
        Statistics stats("var_width", "test", fake_outline, 1.0);
        stats.savePrintTimeCSV(gcode.getPrintTime());
    }
    
    Statistics stats("var_width", "test", fake_outline, 1.0);
    stats.analyse(result_polygons_per_index, result_polylines_per_index);
    stats.visualize(400, false, true, true, false, false);
}

void test(std::string input_outline_filename, std::string output_prefix)
{
//     writeVarWidthTest();
//     std::exit(0);

    // Preparing Input Geometries.
    int r;
    r = time(0);
    r = 1566731558;
    srand(r);
//     logAlways("r = %d;\n", r);
//     logDebug("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    // problem of 2 nearby 3-way intersections I wouldn't know the solution to
    /*
    srand(1564134608);
    Polygons polys = generateTestPoly(30, Point(20000, 20000));
    AABB ab(Point(16436,6754) - Point(1000,1000), Point(16436,6754) + Point(1000,1000));
    Polygons abs; abs.add(ab.toPolygon());
    polys = polys.intersection(abs);
    */
    
    TestPolys::generateTestPolys();

    Polygons polys = SVGloader::load(input_outline_filename);
    AABB aabb(polys);
    polys.applyMatrix(Point3Matrix::translate(aabb.min * -1));
    
    /*
    AABB aabb(Point(0,0), Point(1000,1000));
    Polygons qwe;
    qwe.add(aabb.toPolygon());
    Statistics stats("asf", "asf", qwe, 1.0);
    
    std::vector<std::list<ExtrusionLine>> result_polylines_per_index;

    std::vector<std::list<ExtrusionLine>> result_polygons_per_index;
    result_polylines_per_index.emplace_back();
    ExtrusionLine line(0, false);
    line.junctions.emplace_back(ExtrusionJunction(Point(0,0), 300, 0));
    line.junctions.emplace_back(ExtrusionJunction(Point(10000,0), 500, 0));
    result_polylines_per_index.back().emplace_back(line);
    stats.analyse(result_polygons_per_index, result_polylines_per_index);
    stats.visualize(400);
    
    std::exit(0);*/
    
    
    /*
    coord_t scale = 10000;
    Point p(20000,70000);
    AABB ab(p - scale * Point(1,1), p + scale * Point(1,1));
    Polygons abs; abs.add(ab.toPolygon());
    polys = polys.intersection(abs);
    */

    /*
    Polygons polys = generateTestPoly(40, Point(20000, 20000));
    coord_t scale = 2000;
    AABB ab(Point(6000,10000) - scale * Point(1,1), Point(6000,10000) + scale * Point(1,1));
    Polygons abs; abs.add(ab.toPolygon());
    polys = polys.intersection(abs);*/

    PointMatrix mirror = PointMatrix::scale(1);
    mirror.matrix[3] = -1;

//     Polygons polys = TestPolys::test_poly_1;
//     Polygons polys = TestPolys::squares;
//     Polygons polys = TestPolys::circle;
//     Polygons polys = TestPolys::circle_flawed;
//     Polygons polys = TestPolys::cross_shape;
//     Polygons polys = TestPolys::gMAT_example; polys.applyMatrix(mirror);
//     Polygons polys = TestPolys::test_various_aspects; polys.applyMatrix(PointMatrix::scale(2.2));
//     Polygons polys = TestPolys::simple_MAT_example; polys.applyMatrix(PointMatrix::scale(3)); polys.applyMatrix(PointMatrix(-90));
//     Polygons polys = TestPolys::simple_MAT_example_rounded_corner; polys.applyMatrix(PointMatrix::scale(3)); polys.applyMatrix(PointMatrix(-90));
//     Polygons polys = TestPolys::beading_conflict;
//     Polygons polys = TestPolys::wedge; // polys.applyMatrix(PointMatrix::scale(3));
//     Polygons polys = TestPolys::wedge; polys.applyMatrix(PointMatrix::scale(3));
//     Polygons polys = TestPolys::limit_wedge; //polys.applyMatrix(PointMatrix::scale(3));
//     Polygons polys = TestPolys::double_wedge; // polys.applyMatrix(PointMatrix::scale(3));
//     Polygons polys = TestPolys::flawed_wedge;
//     Polygons polys = TestPolys::clean_and_flawed_wedge_part; polys.applyMatrix(PointMatrix::scale(2.0)); polys.applyMatrix(mirror);
//     Polygons polys = TestPolys::flawed_wall;
//     Polygons polys = TestPolys::marked_local_opt;
//     Polygons polys = TestPolys::legend;
//     Polygons polys = TestPolys::parabola;
//     Polygons polys = TestPolys::pikachu; polys.applyMatrix(PointMatrix::scale(1)); polys.applyMatrix(mirror);
//     Polygons polys = TestPolys::um;
//     Polygons polys = TestPolys::spikes; polys.applyMatrix(PointMatrix::scale(2.0));
//     Polygons polys = TestPolys::spikes_row; polys.applyMatrix(PointMatrix::scale(2.0));
//     Polygons polys = TestPolys::enclosed_region;
//     Polygons polys = TestPolys::jin;
//     Microstructure m; Polygons polys = TestPolys::m.squareGrid(Point(2,2), Point(2000,2000));
//     Microstructure m; Polygons polys = TestPolys::m.hexGrid(Point(12,12), 8000); polys.applyMatrix(PointMatrix::scale(0.5));
//     Polygons polys = MoessenTests::generateCircles(Point(3, 3), 100, 400, 500, 52);
//     Polygons polys = MoessenTests::generateCircles(Point(2, 2), 100, 400, 500, 8);
//     srand(1563874501); Polygons polys = MoessenTests::generateCircles(Point(3, 3), 100, 400, 1000, 8);
//     Polygons polys = MoessenTests::generateTriangles(Point(4, 2), 100, 600, 1000);
//     Polygons polys = MoessenTests::generateTriangles(Point(2, 1), 100, 500, 1000);
//     Polygons polys = MoessenTests::generateTriangles(Point(4, 2), 300, 301, 1000);
//     Polygons polys = MoessenTests::generateTriangles(Point(4, 2), 400, 401, 1000);
//     Polygons polys = Prescribed::fromDistances({Point(0,800), Point(400,300), Point(610,610), Point(1400, 200)});
//     Polygons polys = Spiky::oneSpike(200);
//     Polygons polys = Spiky::twoSpikes();
//     Polygons polys = Spiky::twoSpikesDiamond(MM2INT(0.8), MM2INT(4.0), MM2INT(.1)); polys.applyMatrix(PointMatrix(45.0));
//     Polygons polys = Spiky::oneSpikeOneCorner(MM2INT(0.8), MM2INT(4.0), MM2INT(.1));
//     Polygons polys = Spiky::fourSpikes();
//     Polygons polys = Spiky::doubleOutSpike(800, 380);

    polys = polys.unionPolygons();
    polys.simplify();

#ifdef DEBUG
    {
        SVG svg("output/outline_viz.svg", AABB(polys));
        svg.writeAreas(polys, SVG::Color::NONE, SVG::Color::BLACK);
    }
    {
        SVG svg("output/outline.svg", AABB(polys), INT2MM(1));
        svg.writeAreas(polys, SVG::Color::NONE, SVG::Color::BLACK);
    }
#endif

    coord_t nozzle_size = MM2INT(0.6);
    polys.applyMatrix(PointMatrix::scale(INT2MM(nozzle_size) / 0.4));

    if (false && output_prefix.compare("TEST") != 0)
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << to_string(StrategyType::InwardDistributed) << "_results.csv";
        std::ifstream file(ss.str().c_str());
        if (file.good())
        {
            logAlways("Test already has results saved\n");
            std::exit(-1);
        }
    }

    bool generate_gcodes = false;
    bool analyse = true;
    bool generate_MAT_STL = true;

//     std::vector<StrategyType> strategies({ StrategyType::Naive, StrategyType::Center, StrategyType::InwardDistributed });
//     std::vector<StrategyType> strategies({ StrategyType::Naive, StrategyType::Distributed });
//     std::vector<StrategyType> strategies({ StrategyType::Distributed });
    std::vector<StrategyType> strategies({ StrategyType::OutlineAccuracy });
//     std::vector<StrategyType> strategies({ StrategyType::InwardDistributed });
//     std::vector<StrategyType> strategies({ StrategyType::Center });
//     std::vector<StrategyType> strategies({ StrategyType::Distributed, StrategyType::InwardDistributed });
//     std::vector<StrategyType> strategies({ StrategyType::Constant, StrategyType::Center, StrategyType::Distributed, StrategyType::InwardDistributed, StrategyType::Naive });
//     std::random_shuffle(strategies.begin(), strategies.end());
    for (StrategyType type : strategies )
    {
        if (type == StrategyType::Naive)
        {
            testNaive(polys, nozzle_size, output_prefix, generate_gcodes, analyse);
        }
        else
        {
            test(polys, nozzle_size, output_prefix, type, generate_gcodes, analyse, generate_MAT_STL);
        }
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
//         if (++i % std::max(1l, n / 100) == 0)
//             std::cerr << (i / 100) << "%\n";
    }
    return 0;
}
