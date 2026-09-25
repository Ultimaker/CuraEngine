#include "bridge/BridgeAngleScoringCriterion.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cura
{

BridgeAngleScoringCriterion::BridgeAngleScoringCriterion(std::vector<double> scores)
    : scores_(std::move(scores))
{
}

double BridgeAngleScoringCriterion::computeScore(const size_t candidate_index) const
{
    return scores_.at(candidate_index);
}

AngleDegrees BridgeAngleScoringCriterion::candidateIndexToEvaluationAngle(const size_t candidate_index)
{
    return static_cast<double>(candidate_index);
}

AngleDegrees BridgeAngleScoringCriterion::candidateIndexToExtrusionAngle(const size_t candidate_index)
{
    return candidateIndexToEvaluationAngle(candidate_index) + 90;
}

double BridgeAngleScoringCriterion::extrusionAngleDistance(const AngleDegrees& first, const AngleDegrees& second)
{
    double delta = std::fabs(static_cast<double>(first) - static_cast<double>(second));
    delta = std::fmod(delta, 360.0);
    if (delta > 180.0)
    {
        delta = 360.0 - delta;
    }
    if (delta > 90.0)
    {
        delta = 180.0 - delta;
    }

    return delta;
}

std::vector<double> BridgeAngleScoringCriterion::normalizeScores(const std::vector<double>& raw_scores)
{
    if (raw_scores.empty())
    {
        return {};
    }

    const auto [minimum_it, maximum_it] = std::minmax_element(raw_scores.begin(), raw_scores.end());
    const double minimum = *minimum_it;
    const double maximum = *maximum_it;
    if (maximum - minimum <= std::numeric_limits<double>::epsilon())
    {
        return std::vector<double>(raw_scores.size(), 1.0);
    }

    std::vector<double> normalized_scores;
    normalized_scores.reserve(raw_scores.size());
    for (const double raw_score : raw_scores)
    {
        normalized_scores.push_back((raw_score - minimum) / (maximum - minimum));
    }

    return normalized_scores;
}

} // namespace cura
