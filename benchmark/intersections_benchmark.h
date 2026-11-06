// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <benchmark/benchmark.h>

#include "utils/polygon_utils.h"
#include "types/geometry/Shape.h"


namespace curaengine
{

class IntersectionsTestFixture : public benchmark::Fixture
{
public:
    static constexpr types::coord_t square_side = 1000000;
    types::geometry::Shape shape;

    void SetUp(const ::benchmark::State& state) override
    {
        shape = types::geometry::Shape(utils::polygon_utils::makeDisc(types::geometry::Point2LL(square_side / 2, square_side / 2), square_side, state.range(0)));
    }

    void TearDown(const ::benchmark::State& state) override
    {
    }
};

BENCHMARK_DEFINE_F(IntersectionsTestFixture, IntersectionsTestFixture_WorstCase)(benchmark::State& st)
{
    for (auto _ : st)
    {
        benchmark::DoNotOptimize(shape.intersectionsWithSegment(types::geometry::Point2LL(0, 0), types::geometry::Point2LL(square_side, square_side)));
    }
}

BENCHMARK_REGISTER_F(IntersectionsTestFixture, IntersectionsTestFixture_WorstCase)->Arg(100000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(IntersectionsTestFixture, IntersectionsTestFixture_WorstCase)->Arg(10000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(IntersectionsTestFixture, IntersectionsTestFixture_WorstCase)->Arg(1000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(IntersectionsTestFixture, IntersectionsTestFixture_WorstCase)->Arg(100)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(IntersectionsTestFixture, IntersectionsTestFixture_WorstCase)->Arg(10)->Unit(benchmark::kMillisecond);

} // namespace curaengine
