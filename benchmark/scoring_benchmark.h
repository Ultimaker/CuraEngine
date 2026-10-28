// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_SCORING_BENCHMARK_H
#define CURAENGINE_SCORING_BENCHMARK_H

#include <benchmark/benchmark.h>

#include "geometry/PointsSet.h"
#include "utils/scoring/BestElementFinder.h"
#include "utils/scoring/CornerScoringCriterion.h"
#include "utils/scoring/DistanceScoringCriterion.h"

namespace curaengine
{
class ScoringTestFixture : public benchmark::Fixture
{
public:
    ::curaengine::geometry::PointsSet points;

    void SetUp(const ::benchmark::State& state) override
    {
        points.resize(state.range(0));
        constexpr double radius = 5000.0;

        for (size_t i = 0; i < points.size(); ++i)
        {
            const double angle = (i * std::numbers::pi * 2.0) / points.size();
            points[i] = ::curaengine::geometry::Point2LL(std::cos(angle) * radius, std::sin(angle) * radius);
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
        ::curaengine::utils::scoring::BestElementFinder best_element_finder;

        // Pass 1 : find corners
        ::curaengine::utils::scoring::BestElementFinder::CriteriaPass main_criteria_pass;
        main_criteria_pass.outsider_delta_threshold = 0.05;

        ::curaengine::utils::scoring::BestElementFinder::WeighedCriterion main_criterion;
        main_criterion.criterion = std::make_shared<::curaengine::utils::scoring::CornerScoringCriterion>(points, ::curaengine::types::EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED);
        main_criteria_pass.criteria.push_back(main_criterion);

        best_element_finder.appendCriteriaPass(main_criteria_pass);

        // Pass 2 : fallback to distance calculation
        ::curaengine::utils::scoring::BestElementFinder::CriteriaPass fallback_criteria_pass;
        ::curaengine::utils::scoring::BestElementFinder::WeighedCriterion fallback_criterion;
        fallback_criterion.criterion = std::make_shared<::curaengine::utils::scoring::DistanceScoringCriterion>(points, ::curaengine::geometry::Point2LL(1000, 0));

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
