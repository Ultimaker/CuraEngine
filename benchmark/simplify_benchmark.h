// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_BENCHMARK_SIMPLIFY_BENCHMARK_H
#define CURAENGINE_BENCHMARK_SIMPLIFY_BENCHMARK_H

#include "../tests/ReadTestPolygons.h"
#include "plugins/slots.h"
#include "utils/Simplify.h"
#include "utils/channel.h"

#include <fmt/format.h>

#include <benchmark/benchmark.h>
#include <filesystem>
#include <grpcpp/create_channel.h>

namespace cura
{
class SimplifyTestFixture : public benchmark::Fixture
{
public:
    const std::vector<std::string> POLYGON_FILENAMES = { std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_concave.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_concave_hole.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_square.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_square_hole.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_triangle.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/polygon_two_squares.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_1.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_2.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_3.txt").string(),
                                                         std::filesystem::path(__FILE__).parent_path().append("tests/resources/slice_polygon_4.txt").string() };

    std::vector<Polygons> shapes;

    void SetUp(const ::benchmark::State& state)
    {
        readTestPolygons(POLYGON_FILENAMES, shapes);
    }

    void TearDown(const ::benchmark::State& state)
    {
    }
};

BENCHMARK_DEFINE_F(SimplifyTestFixture, simplify_local)(benchmark::State& st)
{
    Simplify simplify(MM2INT(0.25), MM2INT(0.025), 50000);
    for (auto _ : st)
    {
        Polygons simplified;
        for (const auto& polys : shapes)
        {
            benchmark::DoNotOptimize(simplified = simplify.polygon(polys));
        }
    }
}

BENCHMARK_REGISTER_F(SimplifyTestFixture, simplify_local);

BENCHMARK_DEFINE_F(SimplifyTestFixture, simplify_slot_noplugin)(benchmark::State& st)
{
    for (auto _ : st)
    {
        Polygons simplified;
        for (const auto& polys : shapes)
        {
            benchmark::DoNotOptimize(simplified = slots::instance().modify<plugins::v0::SlotID::SIMPLIFY_MODIFY>(polys, MM2INT(0.25), MM2INT(0.025), 50000));
        }
    }
}

BENCHMARK_REGISTER_F(SimplifyTestFixture, simplify_slot_noplugin);

} // namespace cura
#endif // CURAENGINE_BENCHMARK_SIMPLIFY_BENCHMARK_H
