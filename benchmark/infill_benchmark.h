// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INFILL_BENCHMARK_H
#define CURAENGINE_INFILL_BENCHMARK_H

#include <benchmark/benchmark.h>

#include "infill.h"

namespace cura
{
class InfillTest : public benchmark::Fixture
{
public:
    Settings settings{};
    Polygons square_shape;
    Polygons ff_holes;


    std::vector<ExtrusionLine> all_paths;


    Polygons outline_polygons;
    EFillMethod pattern{ EFillMethod::LINES };
    bool zig_zagify{ true };
    bool connect_polygons{ true };
    coord_t line_distance{ 800 };

    coord_t INFILL_LINE_WIDTH = 350;
    coord_t INFILL_OVERLAP = 0;
    size_t INFILL_MULTIPLIER = 1;
    AngleDegrees FILL_ANGLE = 0.;
    coord_t Z = 100; // Future improvement: Also take an uneven layer, so we get the alternate.
    coord_t SHIFT = 0;
    coord_t MAX_RESOLUTION = 10;
    coord_t MAX_DEVIATION = 5;

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

        outline_polygons.add(square_shape);
        outline_polygons.add(ff_holes);

        settings.add("fill_outline_gaps", "false");
        settings.add("meshfix_maximum_deviation", "0.1");
        settings.add("meshfix_maximum_extrusion_area_deviation", "0.01");
        settings.add("meshfix_maximum_resolution", "0.01");
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

        connect_polygons = state.range(0);
        line_distance = state.range(1);
    }

    void TearDown(const ::benchmark::State& state)
    {
    }
};

BENCHMARK_DEFINE_F(InfillTest, Infill_generate_connect)(benchmark::State& st)
{
    Infill infill(pattern,
                  zig_zagify,
                  connect_polygons,
                  outline_polygons,
                  INFILL_LINE_WIDTH,
                  line_distance,
                  INFILL_OVERLAP,
                  INFILL_MULTIPLIER,
                  FILL_ANGLE,
                  Z,
                  SHIFT,
                  MAX_RESOLUTION,
                  MAX_DEVIATION); // There are some optional parameters, but these will do for now (future improvement?).

    for (auto _ : st)
    {
        std::vector<VariableWidthLines> result_paths;
        Polygons result_polygons;
        Polygons result_lines;
        infill.generate(result_paths, result_polygons, result_lines, settings, 0, SectionType::INFILL, nullptr, nullptr);
    }
}

BENCHMARK_REGISTER_F(InfillTest, Infill_generate_connect)->ArgsProduct({{true, false}, {400, 800, 1200}})->Unit(benchmark::kMillisecond);
} // namespace cura
#endif // CURAENGINE_INFILL_BENCHMARK_H
