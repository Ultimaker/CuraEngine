// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_WALL_BENCHMARK_H
#define CURAENGINE_WALL_BENCHMARK_H

#include <string>

#include <benchmark/benchmark.h>
#include <range/v3/view/join.hpp>

#include "InsetOrderOptimizer.h"
#include "WallsComputation.h"
#include "settings/Settings.h"
#include "sliceDataStorage.h"
#include "utils/polygon.h"

namespace cura
{
class WallTestFixture : public benchmark::Fixture
{
public:
    Settings settings{};
    WallsComputation walls_computation{ settings, LayerIndex(100) };
    Polygons square_shape;
    Polygons ff_holes;
    bool outer_to_inner;
    SliceLayer layer;


    void SetUp(const ::benchmark::State& state)
    {
        square_shape.emplace_back();
        square_shape.back().emplace_back(0, 0);
        square_shape.back().emplace_back(MM2INT(100), 0);
        square_shape.back().emplace_back(MM2INT(100), MM2INT(100));
        square_shape.back().emplace_back(0, MM2INT(100));

        ff_holes.emplace_back();
        ff_holes.back().emplace_back(0, 0);
        ff_holes.back().emplace_back(MM2INT(90), 0);
        ff_holes.back().emplace_back(MM2INT(90), MM2INT(50));
        ff_holes.back().emplace_back(0, MM2INT(50));
        ff_holes.emplace_back();
        ff_holes.back().emplace_back(MM2INT(90), MM2INT(90));
        ff_holes.back().emplace_back(MM2INT(90), MM2INT(40));
        ff_holes.back().emplace_back(MM2INT(40), MM2INT(25));
        ff_holes.emplace_back();
        ff_holes.back().emplace_back(MM2INT(60), MM2INT(90));
        ff_holes.back().emplace_back(MM2INT(60), MM2INT(40));
        ff_holes.back().emplace_back(MM2INT(90), MM2INT(25));

        // Settings for a simple 2 walls, about as basic as possible.
        settings.add("alternate_extra_perimeter", "false");
        settings.add("fill_outline_gaps", "false");
        settings.add("initial_layer_line_width_factor", "100");
        settings.add("magic_spiralize", "false");
        settings.add("meshfix_maximum_deviation", "0.1");
        settings.add("meshfix_maximum_extrusion_area_deviation", "0.01");
        settings.add("meshfix_maximum_resolution", "0.01");
        settings.add("meshfix_fluid_motion_enabled", "false");
        settings.add("min_wall_line_width", "0.3");
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
        settings.add("wall_line_count", std::to_string(state.range(0)));
        outer_to_inner = false;
        layer.parts.emplace_back();

        SliceLayerPart& part = layer.parts.back();
        part.outline.add(ff_holes);
    }

    void TearDown(const ::benchmark::State& state)
    {
    }
};

BENCHMARK_DEFINE_F(WallTestFixture, generateWalls)(benchmark::State& st)
{
    for (auto _ : st)
    {
        walls_computation.generateWalls(&layer, SectionType::WALL);
    }
}

BENCHMARK_REGISTER_F(WallTestFixture, generateWalls)->Arg(3)->Arg(15)->Arg(9999)->Unit(benchmark::kMillisecond);

BENCHMARK_DEFINE_F(WallTestFixture, InsetOrderOptimizer_getRegionOrder)(benchmark::State& st)
{
    walls_computation.generateWalls(&layer, SectionType::WALL);
    std::vector<ExtrusionLine> all_paths;
    for (auto& line : layer.parts.back().wall_toolpaths | ranges::views::join )
    {
        all_paths.emplace_back(line);
    }
    for (auto _ : st)
    {
        auto order = InsetOrderOptimizer::getRegionOrder(all_paths, outer_to_inner);
    }
}

BENCHMARK_REGISTER_F(WallTestFixture, InsetOrderOptimizer_getRegionOrder)->Arg(3)->Arg(15)->Arg(9999)->Unit(benchmark::kMillisecond);

BENCHMARK_DEFINE_F(WallTestFixture, InsetOrderOptimizer_getInsetOrder)(benchmark::State& st)
{
    walls_computation.generateWalls(&layer, SectionType::WALL);
    std::vector<ExtrusionLine> all_paths;
    for (auto& line : layer.parts.back().wall_toolpaths | ranges::views::join )
    {
        all_paths.emplace_back(line);
    }
    for (auto _ : st)
    {
        auto order = InsetOrderOptimizer::getInsetOrder(all_paths, outer_to_inner);
    }
}

BENCHMARK_REGISTER_F(WallTestFixture, InsetOrderOptimizer_getInsetOrder)->Arg(3)->Arg(15)->Arg(9999)->Unit(benchmark::kMillisecond);

} // namespace cura
#endif // CURAENGINE_WALL_BENCHMARK_H
