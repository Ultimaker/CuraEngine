
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <sstream>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>
#include <algorithm> // random_shuffle

#include <tclap/CmdLine.h> // command line argument parser

#include "BoostInterface.hpp"

#include <unordered_set>
#include <unordered_map>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"
#include "utils/ToolpathVisualizer.h"

#include "SkeletalTrapezoidation.h"

#include "BeadingStrategyHelper.h"

#include "utils/VoronoiUtils.h"
#include "NaiveBeadingStrategy.h"
#include "BeadingOrderOptimizer.h"
#include "GcodeWriter.h"
#include "ToolpathWriter.h"
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
static TCLAP::SwitchArg cmd__generate_toolpaths("", "toolpaths", "Generate toolpaths file", false);
static TCLAP::SwitchArg cmd__analyse("", "analyse", "Analyse output paths", false);
static TCLAP::SwitchArg cmd__visualize("", "visualize", "Visualize output paths", false);
static TCLAP::SwitchArg cmd__generate_MAT_STL("", "matstl", "Generate an stl corresponding to the medial axis transform", false);
static TCLAP::ValueArg<std::string> cmd__input_outline_filename("p", "polygon", "Input file for polygon", false /* required? */, "-", "path to file");
static TCLAP::ValueArg<std::string> cmd__output_prefix("o", "output", "Output file name prefix", false /* required? */, "TEST", "path to file");
static TCLAP::ValueArg<double> cmd__scale_amount("", "scale", "Input polygon scaler", false /* required? */, 1.0, "floating number");
static TCLAP::SwitchArg cmd__mirroring("m", "mirror", "Mirror the input file vertically", false);
static TCLAP::SwitchArg cmd__shuffle_strategies("", "shuffle", "Execute the strategies in random order", false);
static TCLAP::ValueArg<std::string> cmd__strategy_set("s", "strat", "Set of strategies to test. Each character identifies one strategy.", false /* required? */, "crdin", "n|N|c|r|d|i|n|s|l|o");
static TCLAP::ValueArg<double> cmd__discretization_step_size("d", "discretization", "Discretization step size", /*req=*/ false, /*default=*/0.2, "mm");
static TCLAP::ValueArg<double> cmd__transition_filter_dist("f", "transitionfilter", "Smallest distance between alternating bead counts", /*req=*/ false, /*default=*/1.0, "mm");
static TCLAP::ValueArg<double> cmd__beading_propagation_transition_dist("t", "bptd", "Beading propagation transition distance", /*req=*/ false, /*default=*/0.4, "mm");
static TCLAP::ValueArg<double> cmd__default_transition_length("l", "transitionlength", "Default bead transition length.", /*req=*/ false, /*default=*/0.4, "mm");
static TCLAP::ValueArg<bool> cmd__reduce_extrusion_line_overlap("r", "reduce", "Cut off part of the ends of extrusion lines in order to reduce overlap", /*req=*/ false, /*default=*/true, "boolean");
static TCLAP::ValueArg<bool> cmd__connect_odd_lines_to_polygons("", "connect_odd", "Connect odd polyline toolpaths to normal toolpaths at 3-way intersections, rather than closing polygonal toolpaths", /*req=*/ false, /*default=*/true, "boolean");
static TCLAP::SwitchArg cmd__filter_outermost_marked_edges("", "filterouter", "Unmark all outer edges of the Voronoi Diagram, so that marked edges never touch the outline", /*req=*/ false);
static TCLAP::ValueArg<double> cmd__transitioning_angle("a", "limit_bisector_angle", "Geometry angle for which we have to start introducing controlled beading", /*req=*/ false, /*default=*/45, "degree");
static TCLAP::ValueArg<double> cmd__min_bead_width("", "minw", "Minimal toolpath bead width for geometry with a diameter smaller than the nozzle size", /*req=*/ false, /*default=*/0, "mm");
static TCLAP::ValueArg<double> cmd__min_feature_size("", "mins", "Minimal geometry diameter for which to generate a bead", /*req=*/ false, /*default=*/0, "mm");
static TCLAP::ValueArg<double> cmd__inward_distributed_center_size("n", "centersize", "(Half of ) the number of beads over which to distribute the discrepancy", /*req=*/ false, /*default=*/2, "");
static TCLAP::ValueArg<int> cmd__max_bead_count("b", "beadcount", "Number of beads to generate. -1 = infinity", /*req=*/ false, /*default=*/-1, "");
static TCLAP::ValueArg<double> cmd__nozzle_size("w", "width", "Preferred bead width, middle of range of possible widths", /*req=*/ false, /*default=*/0.5, "");

bool generate_gcodes = true;
bool generate_toolpaths = true;
bool analyse = false;
bool visualize = false;

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
bool connect_odd_lines_to_polygons = true;
bool filter_outermost_marked_edges = false;

double transitioning_angle = M_PI / 4;

std::optional<coord_t> min_bead_width; // for if we want to deal with geometry smaller than the nozzle size separately
std::optional<coord_t> min_feature_size; // for if we want to deal with geometry smaller than the nozzle size separately

coord_t nozzle_size;

bool readCommandLine(int argc, char **argv)
{
    try {
        gCmdLine.add(cmd__generate_gcodes);
        gCmdLine.add(cmd__generate_toolpaths);
        gCmdLine.add(cmd__analyse);
        gCmdLine.add(cmd__visualize);
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
        gCmdLine.add(cmd__default_transition_length);
        gCmdLine.add(cmd__reduce_extrusion_line_overlap);
        gCmdLine.add(cmd__connect_odd_lines_to_polygons);
        gCmdLine.add(cmd__filter_outermost_marked_edges);
        gCmdLine.add(cmd__transitioning_angle);
        gCmdLine.add(cmd__min_bead_width);
        gCmdLine.add(cmd__min_feature_size);
        gCmdLine.add(cmd__inward_distributed_center_size);
        gCmdLine.add(cmd__max_bead_count);
        gCmdLine.add(cmd__nozzle_size);

        gCmdLine.parse(argc, argv);

        generate_gcodes = cmd__generate_gcodes.getValue();
        generate_toolpaths = cmd__generate_toolpaths.getValue();
        analyse = cmd__analyse.getValue();
        visualize = cmd__visualize.getValue();
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
        default_transition_length = MM2INT(cmd__default_transition_length.getValue());
        reduce_overlapping_segments = cmd__reduce_extrusion_line_overlap.getValue();
        connect_odd_lines_to_polygons = cmd__connect_odd_lines_to_polygons.getValue();
        filter_outermost_marked_edges = cmd__filter_outermost_marked_edges.getValue();
        transitioning_angle = cmd__transitioning_angle.getValue() / 180 * M_PI;
        if (cmd__min_bead_width.getValue() > 0.0) min_bead_width = MM2INT(cmd__min_bead_width.getValue());
        if (cmd__min_feature_size.getValue() > 0.0) min_feature_size = MM2INT(cmd__min_feature_size.getValue());

        inward_distributed_center_size = cmd__inward_distributed_center_size.getValue();
        max_bead_count = cmd__max_bead_count.getValue();
        nozzle_size = MM2INT(cmd__nozzle_size.getValue());

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

    BeadingStrategy* beading_strategy = BeadingStrategyHelper::makeStrategy(type, nozzle_size, transitioning_angle, min_bead_width, min_feature_size);
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

    if (max_bead_count > 0)
    { // throw away generated odd beads in the middle
        result_polylines_per_index.resize(max_bead_count / 2);
    }

    std::vector<std::list<ExtrusionLine>> result_polygons_per_index;
    BeadingOrderOptimizer::optimize(result_polygons_per_index, result_polylines_per_index, reduce_overlapping_segments_now, connect_odd_lines_to_polygons);
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
    if (generate_toolpaths)
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << to_string(type) << "_toolpaths.txt";
        ToolpathWriter toolpather(ss.str());
        toolpather.write(result_polygons_per_index, result_polylines_per_index);
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
        if (visualize)
        {
            stats.visualize(nozzle_size, true);
            logAlways("Visualization took %fs\n", tk.restart());
        }
    }
    else if (visualize)
    {
        static std::vector<ExtrusionSegment> all_segments = Statistics::generateAllSegments(result_polygons_per_index, result_polylines_per_index);
        AABB aabb(polys);
        bool rounded_visualization = true;
        {
            std::ostringstream ss;
            ss << "output/" << output_prefix << "_" << to_string(type) << "_viz.svg";
            SVG svg(ss.str(), aabb);
            ToolpathVisualizer viz(svg);
            viz.outline(polys);
            viz.toolpaths(all_segments, rounded_visualization);
        }
        bool include_legend = false;
        bool output_widths = false;
        if (output_widths)
        {
            std::ostringstream ss;
            ss << "output/" << output_prefix << "_" << to_string(type) << "_widths.svg";
            SVG svg(ss.str(), aabb);
            ToolpathVisualizer viz(svg);
            viz.outline(polys);

            coord_t max_dev = nozzle_size / 2;
            coord_t min_w = 30;

            if (include_legend)
            {
                viz.width_legend(polys, nozzle_size, max_dev, min_w, rounded_visualization);
            }

            viz.widths(all_segments, nozzle_size, max_dev, min_w, rounded_visualization);
        }
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
    if (generate_toolpaths)
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_naive_toolpaths.txt";
        ToolpathWriter toolpather(ss.str());
        toolpather.write(result_polygons_per_index, result_polylines_per_index);
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
//     polys.applyMatrix(Point3Matrix::translate(aabb.min * -1));
    
    // prevent any problem in the input polygon
    // prevent single polygon to be two areas connected only by a zero width channel, i.e. the polygon has two points at the same location
//     boost::polygon::polygon_90_concept::
    
    polys.simplify();
    polys = polys.processEvenOdd();
    bool boost_fixing = true;
    if (boost_fixing)
    {
        CPolygonSet boost_polys = toBoostType(polys); // immediately automatically performs a manifoldness fix
        polys = toPolygons(boost_polys);
        polys.simplify();
    }
    else
    {
        polys.simplify();
        polys = polys.unionPolygons();
        polys = polys.processEvenOdd();
    //     polys = polys.offset(-20).offset(40).offset(-20);
    //     polys.removeSmallAreas(INT2MM(nozzle_size) * INT2MM(nozzle_size));
    //     polys = polys.offset(20);
    //     polys.simplify();
    //     polys.processEvenOdd();
        polys.removeDegenerateVerts();
        {
            std::unordered_set<Point> poly_locations;
            for (size_t poly_idx = 0; poly_idx < polys.size(); poly_idx++)
            {
                PolygonRef poly = polys[poly_idx];
                for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
                {
                    Point p = poly[point_idx];
                    if (poly_locations.find(p) != poly_locations.end())
                    {
                        PolygonsPointIndex here(&polys, poly_idx, point_idx);
                        PolygonsPointIndex prev = here.prev();
                        PolygonsPointIndex next = here.next();
                        Point bit_back = here.p() + normal(prev.p() - here.p(), 5);
                        Point bit_forward = here.p() + normal(next.p() - here.p(), 5);
                        poly[point_idx] = bit_forward;
                        poly.insert(point_idx, bit_back);
                    }
                    poly_locations.emplace(p);
                }
            }
        }
        polys.removeDegenerateVerts();
        polys.simplify();
    }

#ifdef DEBUG
    {
        SVG svg("output/outline.svg", AABB(polys), INT2MM(1));
        svg.writeAreas(polys, SVG::Color::NONE, SVG::Color::BLACK);
    }
#endif

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
