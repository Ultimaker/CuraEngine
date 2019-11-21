
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <sstream>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>
#include <algorithm> // random_shuffle

#include <tclap/CmdLine.h> // command line argument parser

#include <boost/version.hpp>

#include <unordered_set>
#include <unordered_map>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "SkeletalTrapezoidation.h"

#include "BeadingStrategyHelper.h"

#include "utils/VoronoiUtils.h"
#include "NaiveBeadingStrategy.h"
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

using namespace arachne;

static TCLAP::CmdLine gCmdLine(" Generate polygon inset toolpaths ", ' ', "0.1");

static TCLAP::SwitchArg cmd__generate_gcodes("g", "gcode", "Generate gcode", false);
static TCLAP::SwitchArg cmd__analyse("a", "analyse", "Analyse output paths", false);
static TCLAP::SwitchArg cmd__generate_MAT_STL("", "matstl", "Generate an stl corresponding to the medial axis transform", false);
static TCLAP::ValueArg<std::string> cmd__input_outline_filename("p", "polygon", "Input file for polygon", false /* required? */, "-", "path to file");
static TCLAP::ValueArg<std::string> cmd__output_prefix("o", "output", "Output file name prefix", false /* required? */, "TEST", "path to file");
static TCLAP::ValueArg<double> cmd__scale_amount("", "scale", "Input polygon scaler", false /* required? */, 1.0, "floating number");
static TCLAP::SwitchArg cmd__mirroring("m", "mirror", "Mirror the input file vertically", false);
static TCLAP::SwitchArg cmd__shuffle_strategies("", "shuffle", "Execute the strategies in random order", false);
static TCLAP::ValueArg<std::string> cmd__strategy_set("s", "strat", "Set of strategies to test. Each character identifies one strategy.", false /* required? */, "crdin", "n|N|c|r|d|i|n|s|o");
static TCLAP::ValueArg<double> cmd__discretization_step_size("d", "discretization", "Discretization step size", /*req=*/ false, /*default=*/0.2, "mm");
static TCLAP::ValueArg<double> cmd__transition_filter_dist("f", "transitionfilter", "Smallest distance between alternating bead counts", /*req=*/ false, /*default=*/1.0, "mm");
static TCLAP::ValueArg<double> cmd__beading_propagation_transition_dist("t", "bptd", "Beading propagation transition distance", /*req=*/ false, /*default=*/0.4, "mm");
static TCLAP::ValueArg<bool> cmd__reduce_extrusion_line_overlap("r", "reduce", "Cut off part of the ends of extrusion lines in order to reduce overlap", /*req=*/ false, /*default=*/true, "boolean");
static TCLAP::SwitchArg cmd__filter_outermost_marked_edges("", "filterouter", "Unmark all outer edges of the Voronoi Diagram, so that marked edges never touch the outline", /*req=*/ false);

bool generate_gcodes = true;
bool analyse = false;

std::string input_outline_filename;
std::string output_prefix;

double scale_amount;
bool mirroring;

bool shuffle_strategies;
std::vector<StrategyType> strategies;

coord_t discretization_step_size = 200;
coord_t transition_filter_dist = 1000;
coord_t beading_propagation_transition_dist = 400;
bool reduce_overlapping_segments = true;
bool filter_outermost_marked_edges = false;

bool readCommandLine(int argc, char **argv)
{
    try {
        gCmdLine.add(cmd__generate_gcodes);
        gCmdLine.add(cmd__analyse);
        gCmdLine.add(cmd__generate_MAT_STL);
        gCmdLine.add(cmd__input_outline_filename);
        gCmdLine.add(cmd__output_prefix);
        gCmdLine.add(cmd__scale_amount);
        gCmdLine.add(cmd__mirroring);
        gCmdLine.add(cmd__shuffle_strategies);
        gCmdLine.add(cmd__strategy_set);
        gCmdLine.add(cmd__discretization_step_size);
        gCmdLine.add(cmd__transition_filter_dist);
        gCmdLine.add(cmd__beading_propagation_transition_dist);
        gCmdLine.add(cmd__reduce_extrusion_line_overlap);
        gCmdLine.add(cmd__filter_outermost_marked_edges);

        gCmdLine.parse(argc, argv);

        generate_gcodes = cmd__generate_gcodes.getValue();
        analyse = cmd__analyse.getValue();
        generate_MAT_STL = cmd__generate_MAT_STL.getValue();
        input_outline_filename = cmd__input_outline_filename.getValue();
        output_prefix = cmd__output_prefix.getValue();
        scale_amount = cmd__scale_amount.getValue();
        mirroring = cmd__mirroring.getValue();
        
        shuffle_strategies = cmd__shuffle_strategies.getValue();
        for (char c : cmd__strategy_set.getValue())
        {
            strategies.emplace_back(toStrategyType(c));
        }
        
        discretization_step_size = MM2INT(cmd__discretization_step_size.getValue());
        transition_filter_dist = MM2INT(cmd__transition_filter_dist.getValue());
        beading_propagation_transition_dist = MM2INT(cmd__beading_propagation_transition_dist.getValue());
        reduce_overlapping_segments = cmd__reduce_extrusion_line_overlap.getValue();
        filter_outermost_marked_edges = cmd__filter_outermost_marked_edges.getValue();

        return false;
    }
    catch (const TCLAP::ArgException & e) {
        std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;
    } catch (...) { // catch any exceptions
        std::cerr << "Error: unknown exception caught" << std::endl;
    }
    return true;
}



void test(Polygons& polys, coord_t nozzle_size, std::string output_prefix, StrategyType type, bool generate_gcodes = true, bool analyse = false, bool generate_MAT_STL = false)
{
    std::string type_str = to_string(type);
    logAlways(">> Performing %s strategy...\n", type_str.c_str());
    float transitioning_angle = M_PI / 4; // = 180 - the "limit bisector angle" from the paper

    BeadingStrategy* beading_strategy = BeadingStrategyHelper::makeStrategy(type, nozzle_size, transitioning_angle);
    if (!beading_strategy) return;

    BeadingStrategy::checkTranisionThicknessConsistency(beading_strategy);

    TimeKeeper tk;

    bool reduce_overlapping_segments_now = reduce_overlapping_segments;
    bool filter_outermost_marked_edges_now = filter_outermost_marked_edges;
    if (type == StrategyType::SingleBead)
    {
        transition_filter_dist = 50;
        reduce_overlapping_segments_now = false;
    }
    else if (type == StrategyType::Constant)
    {
        filter_outermost_marked_edges_now = true;
    }
    SkeletalTrapezoidation st(polys, transitioning_angle, discretization_step_size, transition_filter_dist, beading_propagation_transition_dist);

    std::vector<std::list<ExtrusionLine>> result_polylines_per_index = st.generateToolpaths(*beading_strategy, filter_outermost_marked_edges_now);

    std::vector<std::list<ExtrusionLine>> result_polygons_per_index;
    BeadingOrderOptimizer::optimize(result_polygons_per_index, result_polylines_per_index, reduce_overlapping_segments_now);
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
    for (size_t inset_idx = 0; inset_idx < insets.size(); inset_idx++)
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
    

    PointMatrix mirror = PointMatrix::scale(1);
    if (mirroring)
    {
        mirror.matrix[3] = -1;
    }
    PointMatrix scaler = PointMatrix::scale(scale_amount);

    Polygons polys = SVGloader::load(input_outline_filename);
    polys.applyMatrix(Point3Matrix(scaler).compose(mirror));
    
    AABB aabb(polys);
    polys.applyMatrix(Point3Matrix::translate(aabb.min * -1));
    polys = polys.unionPolygons();
    
    
    polys.simplify();

#ifdef DEBUG
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

    if (shuffle_strategies)
    {
        std::random_shuffle(strategies.begin(), strategies.end());
    }
    for (StrategyType type : strategies )
    {
        if (type == StrategyType::Naive)
        {
            testNaive(polys, nozzle_size, output_prefix, generate_gcodes, analyse);
        }
        else if (type == StrategyType::COUNT)
        {
            std::cerr << "Trying to perform unknown strategy type!\n";
        }
        else
        {
            test(polys, nozzle_size, output_prefix, type, generate_gcodes, analyse, generate_MAT_STL);
        }
    }
}



int main(int argc, char *argv[])
{
    if( readCommandLine(argc, argv) ) exit(EXIT_FAILURE);

    long n = 1;
    for (int i = 0; i < n; i++)
    {
        test(input_outline_filename, output_prefix);
    }
    return 0;
}
