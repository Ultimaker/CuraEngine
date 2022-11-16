// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <benchmark/benchmark.h>

#include "InsetOrderOptimizer.h"
#include "WallsComputation.h"
#include "settings/Settings.h"
#include "sliceDataStorage.h"
#include "utils/polygon.h"


using namespace cura;

class WallsComputationTest : public benchmark::Fixture
{
public:
    /*!
     * Settings to slice with. This is linked in the walls_computation fixture.
     */
    Settings settings{};

    /*!
     * WallsComputation instance to test with. The layer index will be 100.
     */
    WallsComputation walls_computation{ settings, LayerIndex(100) };

    /*!
     * Basic 10x10mm square shape to work with.
     */
    Polygons square_shape;

    /*!
     * A rectangle enclosing two triangular holes;
     */
    Polygons ff_holes;

    void SetUp(const ::benchmark::State& state)
    {
        square_shape.emplace_back();
        square_shape.back().emplace_back(0, 0);
        square_shape.back().emplace_back(MM2INT(10), 0);
        square_shape.back().emplace_back(MM2INT(10), MM2INT(10));
        square_shape.back().emplace_back(0, MM2INT(10));

        ff_holes.emplace_back();
        ff_holes.back().emplace_back(0, 0);
        ff_holes.back().emplace_back(10000, 0);
        ff_holes.back().emplace_back(10000, 5000);
        ff_holes.back().emplace_back(0, 5000);
        ff_holes.emplace_back();
        ff_holes.back().emplace_back(1000, 1000);
        ff_holes.back().emplace_back(1000, 4000);
        ff_holes.back().emplace_back(4000, 2500);
        ff_holes.emplace_back();
        ff_holes.back().emplace_back(6000, 1000);
        ff_holes.back().emplace_back(6000, 4000);
        ff_holes.back().emplace_back(9000, 2500);

        // Settings for a simple 2 walls, about as basic as possible.
        settings.add("alternate_extra_perimeter", "false");
        settings.add("fill_outline_gaps", "false");
        settings.add("initial_layer_line_width_factor", "100");
        settings.add("magic_spiralize", "false");
        settings.add("meshfix_maximum_deviation", "0.1");
        settings.add("meshfix_maximum_extrusion_area_deviation", "0.01");
        settings.add("meshfix_maximum_resolution", "0.01");
        settings.add("min_bead_width", "0");
        settings.add("min_feature_size", "0");
        settings.add("wall_0_extruder_nr", "0");
        settings.add("wall_0_inset", "0");
        settings.add("wall_line_count", "2");
        settings.add("wall_line_width_0", "0.4");
        settings.add("wall_line_width_x", "0.4");
        settings.add("min_even_wall_line_width", "0.34");
        settings.add("min_odd_wall_line_width", "0.34");
        settings.add("wall_transition_angle", "10");
        settings.add("wall_transition_filter_distance", "1");
        settings.add("wall_transition_filter_deviation", ".2");
        settings.add("wall_transition_length", "1");
        settings.add("wall_x_extruder_nr", "0");
        settings.add("wall_distribution_count", "2");
        settings.add("wall_line_count", "5");
    }

    void TearDown(const ::benchmark::State& state)
    {
    }
};

BENCHMARK_F(WallsComputationTest, centerpoint_boundingbox)(benchmark::State& st)
{
    SliceLayer layer;
    layer.parts.emplace_back();
    SliceLayerPart& part = layer.parts.back();
    part.outline.add(ff_holes);

    // Run the test.
    walls_computation.generateWalls(&layer);

    const bool outer_to_inner = false;
    std::vector<ExtrusionLine> all_paths;
    for (auto& inset : part.wall_toolpaths)
    {
        for (auto& line : inset)
        {
            all_paths.emplace_back(line);
        }
    }
    for (auto _ : st)
    {
        auto order = InsetOrderOptimizer::getRegionOrder(all_paths, outer_to_inner);
    }
}

// Run the benchmark
BENCHMARK_MAIN();