// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_SCORING_BENCHMARK_H
#define CURAENGINE_SCORING_BENCHMARK_H

#include <benchmark/benchmark.h>

#include "seam/seam_corners.h"
#include "types/geometry/ExtrusionLine.h"
#include "types/scoring/BestElementFinder.h"
#include "types/scoring/CornerScoringCriterion.h"
#include "types/scoring/DistanceScoringCriterion.h"
#include "types/scoring/WeighedScoringCriteriaPass.h"

namespace curaengine
{
class ScoringTestFixture : public benchmark::Fixture
{
public:
    types::geometry::ExtrusionLine extrusion_line{ 0 };
    std::vector<seam::SeamCandidate> candidates;

    void SetUp(const ::benchmark::State& state) override
    {
        const size_t points_count = state.range(0);
        constexpr double radius = 5000.0;

        extrusion_line = types::geometry::ExtrusionLine(0);
        for (size_t i = 0; i < points_count; ++i)
        {
            const double angle = (i * std::numbers::pi * 2.0) / points_count;
            extrusion_line.emplace_back(types::geometry::Point2LL(std::cos(angle) * radius, std::sin(angle) * radius), 400, 0);
        }

        candidates.clear();
        candidates.reserve(points_count);
        for (size_t i = 0; i < points_count; ++i)
        {
            candidates.emplace_back(extrusion_line, i);
            candidates.back().score = std::abs(std::sin(static_cast<double>(i)));
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
        types::scoring::BestElementFinder<seam::SeamCandidate> best_element_finder;

        // Pass 1 : find corners
        auto main_criteria_pass = std::make_shared<types::scoring::WeighedScoringCriteriaPass<seam::SeamCandidate>>();
        main_criteria_pass->outsider_delta_threshold = 0.05;

        types::scoring::WeighedScoringCriteriaPass<seam::SeamCandidate>::WeighedCriterion main_criterion;
        main_criterion.criterion = std::make_shared<types::scoring::CornerScoringCriterion>(types::EZSeamCornerPrefType::ZSeamCornerPrefWeighted);
        main_criteria_pass->criteria.push_back(main_criterion);

        best_element_finder.appendCriteriaPass(main_criteria_pass);

        // Pass 2 : fallback to distance calculation
        best_element_finder.appendSingleCriterionPass(
            std::make_shared<types::scoring::DistanceScoringCriterion<seam::SeamCandidate>>(types::geometry::Point2LL(1000, 0)));

        best_element_finder.findBestElement(candidates);
    }
}

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(10000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(1000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ScoringTestFixture, ScoringTest_WorstCase)->Arg(10)->Unit(benchmark::kMillisecond);

} // namespace curaengine
#endif // CURAENGINE_SCORING_BENCHMARK_H
