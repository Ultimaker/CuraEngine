// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_BENCHMARK_SIMPLIFY_BENCHMARK_H
#define CURAENGINE_BENCHMARK_SIMPLIFY_BENCHMARK_H

#include <benchmark/benchmark.h>
#include <filesystem>

#include <fmt/format.h>

#include "../tests/ReadTestPolygons.h"
#include "types/geometry/Simplify.h"
#include "utils/channel.h"

#ifdef ENABLE_PLUGINS
#include "plugins/Slots.h"
#include <grpcpp/create_channel.h>
#endif

namespace curaengine
{
class SimplifyTestFixture : public benchmark::Fixture
{
public:
    const std::vector<std::string> polygon_filenames = { std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_concave.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_concave_hole.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_square.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_square_hole.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_triangle.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_two_squares.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_1.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_2.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_3.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_4.txt").string() };

    std::vector<types::geometry::Shape> shapes;

    void SetUp(const ::benchmark::State& state)
    {
        readTestPolygons(polygon_filenames, shapes);
    }

    void TearDown(const ::benchmark::State& state)
    {
    }
};

BENCHMARK_DEFINE_F(SimplifyTestFixture, simplify_local)(benchmark::State& st)
{
    types::geometry::Simplify simplify(types::mmToInt(0.25), types::mmToInt(0.025), 50000);
    for (auto _ : st)
    {
        types::geometry::Shape simplified;
        for (const auto& polys : shapes)
        {
            benchmark::DoNotOptimize(simplified = simplify.polygon(polys));
        }
    }
}

BENCHMARK_REGISTER_F(SimplifyTestFixture, simplify_local);

#ifdef ENABLE_PLUGINS
BENCHMARK_DEFINE_F(SimplifyTestFixture, simplify_slot_noplugin)(benchmark::State& st)
{
    for (auto _ : st)
    {
        types::geometry::Shape simplified;
        for (const auto& polys : shapes)
        {
            benchmark::DoNotOptimize(simplified = plugins::slots::instance().modify<plugins::v0::SlotID::SIMPLIFY_MODIFY>(polys, types::mmToInt(0.25), types::mmToInt(0.025), 50000));
        }
    }
}

BENCHMARK_REGISTER_F(SimplifyTestFixture, simplify_slot_noplugin);
#endif

} // namespace curaengine
#endif // CURAENGINE_BENCHMARK_SIMPLIFY_BENCHMARK_H
