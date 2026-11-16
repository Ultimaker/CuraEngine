// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INFILL_BENCHMARK_H
#define CURAENGINE_INFILL_BENCHMARK_H

#include <benchmark/benchmark.h>

#include "infill/Infill.h"
#include "types/geometry/LinesSet.h"
#include "types/geometry/OpenLinesSet.h"
#include "types/geometry/OpenPolyline.h"

namespace curaengine
{
class InfillTest : public benchmark::Fixture
{
public:
    Settings settings{};
    types::geometry::Shape square_shape;
    types::geometry::Shape ff_holes;


    std::vector<types::geometry::ExtrusionLine> all_paths;


    types::geometry::Shape outline_polygons;
    types::EFillMethod pattern{ types::EFillMethod::Lines };
    bool zig_zagify{ true };
    bool connect_polygons{ true };
    types::coord_t line_distance{ 800 };

    types::coord_t infill_line_width = 350;
    types::coord_t infill_overlap = 0;
    size_t infill_multiplier = 1;
    types::AngleDegrees fill_angle = 0.;
    types::coord_t z = 100; // Future improvement: Also take an uneven layer, so we get the alternate.
    types::coord_t shift = 0;
    types::coord_t max_resolution = 10;
    types::coord_t max_deviation = 5;

    void SetUp(const ::benchmark::State& state)
    {
        square_shape.emplace_back();
        square_shape.back().emplace_back(0, 0);
        square_shape.back().emplace_back(types::mmToInt(100), 0);
        square_shape.back().emplace_back(types::mmToInt(100), types::mmToInt(100));
        square_shape.back().emplace_back(0, types::mmToInt(100));

        ff_holes.emplace_back();
        ff_holes.back().emplace_back(0, 0);
        ff_holes.back().emplace_back(types::mmToInt(90), 0);
        ff_holes.back().emplace_back(types::mmToInt(90), types::mmToInt(50));
        ff_holes.back().emplace_back(0, types::mmToInt(50));
        ff_holes.emplace_back();
        ff_holes.back().emplace_back(types::mmToInt(90), types::mmToInt(90));
        ff_holes.back().emplace_back(types::mmToInt(90), types::mmToInt(40));
        ff_holes.back().emplace_back(types::mmToInt(40), types::mmToInt(25));
        ff_holes.emplace_back();
        ff_holes.back().emplace_back(types::mmToInt(60), types::mmToInt(90));
        ff_holes.back().emplace_back(types::mmToInt(60), types::mmToInt(40));
        ff_holes.back().emplace_back(types::mmToInt(90), types::mmToInt(25));

        outline_polygons.push_back(square_shape);
        outline_polygons.push_back(ff_holes);

        settings.add("fill_outline_gaps", "false");
        settings.add("meshfix_maximum_deviation", "0.1");
        settings.add("meshfix_maximum_extrusion_area_deviation", "0.01");
        settings.add("meshfix_maximum_resolution", "0.01");
        settings.add("min_wall_line_width", "0.3");
        settings.add("min_bead_width", "0");
        settings.add("min_feature_size", "0");
        settings.add("wall_0_extruder_nr", "0");
        settings.add("wall_0_inset", "0");
        settings.add("wall_x_inset", "0");
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
    infill::Infill infill(
        pattern,
        zig_zagify,
        connect_polygons,
        outline_polygons,
        infill_line_width,
        line_distance,
        infill_overlap,
        infill_multiplier,
        fill_angle,
        z,
        shift,
        max_resolution,
        max_deviation); // There are some optional parameters, but these will do for now (future improvement?).

    for (auto _ : st)
    {
        std::vector<types::geometry::VariableWidthLines> result_paths;
        types::geometry::Shape result_polygons;
        types::geometry::OpenLinesSet result_lines;
        infill.generate(result_paths, result_polygons, result_lines, settings, 0, types::SectionType::Infill, nullptr, nullptr);
    }
}

BENCHMARK_REGISTER_F(InfillTest, Infill_generate_connect)->ArgsProduct({ { true, false }, { 400, 800, 1200 } })->Unit(benchmark::kMillisecond);
} // namespace curaengine
#endif // CURAENGINE_INFILL_BENCHMARK_H
