// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_SCORING_BENCHMARK_H
#define CURAENGINE_SCORING_BENCHMARK_H

#include <benchmark/benchmark.h>

#include "types/geometry/PointsSet.h"
#include "types/scoring/BestElementFinder.h"
#include "types/scoring/CornerScoringCriterion.h"
#include "types/scoring/DistanceScoringCriterion.h"

namespace curaengine
{
class ScoringTestFixture : public benchmark::Fixture
{
public:
    types::geometry::PointsSet points;

    void SetUp(const ::benchmark::State& state) override
    {
        points.resize(state.range(0));
        constexpr double radius = 5000.0;

        for (size_t i = 0; i < points.size(); ++i)
        {
            const double angle = (i * std::numbers::pi * 2.0) / points.size();
            points[i] = types::geometry::Point2LL(std::cos(angle) * radius, std::sin(angle) * radius);
        }
    }

    void TearDown(const ::benchmark::State& state) override
    {
    }
};

BENCHMARK_DEFINE_F(ScoringTestFixture, ScoringTest_WorstCase)(benchmark::State& st)
{
    for (auto _ : st)
    {
        types::scoring::BestElementFinder best_element_finder;

        // Pass 1 : find corners
        types::scoring::BestElementFinder::CriteriaPass main_criteria_pass;
        main_criteria_pass.outsider_delta_threshold = 0.05;

        types::scoring::BestElementFinder::WeighedCriterion main_criterion;
        main_criterion.criterion = std::make_shared<types::scoring::CornerScoringCriterion>(points, types::EZSeamCornerPrefType::ZSeamCornerPrefWeighted);
        main_criteria_pass.criteria.push_back(main_criterion);

        best_element_finder.appendCriteriaPass(main_criteria_pass);

        // Pass 2 : fallback to distance calculation
        types::scoring::BestElementFinder::CriteriaPass fallback_criteria_pass;
        types::scoring::BestElementFinder::WeighedCriterion fallback_criterion;
        fallback_criterion.criterion = std::make_shared<types::scoring::DistanceScoringCriterion>(points, types::geometry::Point2LL(1000, 0));

        fallback_criteria_pass.criteria.push_back(fallback_criterion);
        best_element_finder.appendCriteriaPass(fallback_criteria_pass);

        best_element_finder.findBestElement(points.size());
    }
}

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(10000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(1000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(10)->Unit(benchmark::kMillisecond);

} // namespace curaengine
#endif // CURAENGINE_SCORING_BENCHMARK_H
