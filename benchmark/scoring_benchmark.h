// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_SCORING_BENCHMARK_H
#define CURAENGINE_SCORING_BENCHMARK_H

#include <benchmark/benchmark.h>

#include "geometry/PointsSet.h"
#include "utils/scoring/BestElementFinder.h"
#include "utils/scoring/CornerScoringCriterion.h"
#include "utils/scoring/DistanceScoringCriterion.h"

namespace cura
{
class ScoringTestFixture : public benchmark::Fixture
{
public:
    PointsSet points;

    void SetUp(const ::benchmark::State& state) override
    {
        points.resize(state.range(0));
        constexpr double radius = 5000.0;

        for (size_t i = 0; i < points.size(); ++i)
        {
            const double angle = (i * std::numbers::pi * 2.0) / points.size();
            points[i] = Point2LL(std::cos(angle) * radius, std::sin(angle) * radius);
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
        BestElementFinder best_element_finder;

        // Pass 1 : find corners
        BestElementFinder::CriteriaPass main_criteria_pass;
        main_criteria_pass.outsider_delta_threshold = 0.05;

        BestElementFinder::WeighedCriterion main_criterion;
        main_criterion.criterion = std::make_shared<CornerScoringCriterion>(points, EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED);
        main_criteria_pass.criteria.push_back(main_criterion);

        best_element_finder.appendCriteriaPass(main_criteria_pass);

        // Pass 2 : fallback to distance calculation
        BestElementFinder::CriteriaPass fallback_criteria_pass;
        BestElementFinder::WeighedCriterion fallback_criterion;
        fallback_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, Point2LL(1000, 0));

        fallback_criteria_pass.criteria.push_back(fallback_criterion);
        best_element_finder.appendCriteriaPass(fallback_criteria_pass);

        best_element_finder.findBestElement(points.size());
    }
}

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(10000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(1000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(10)->Unit(benchmark::kMillisecond);

} // namespace cura
#endif // CURAENGINE_SCORING_BENCHMARK_H
