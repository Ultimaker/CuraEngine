#ifndef BRIDGE_BRIDGEANGLESCORINGCRITERION_H
#define BRIDGE_BRIDGEANGLESCORINGCRITERION_H

#include <vector>

#include "settings/types/Angle.h"
#include "utils/scoring/ScoringCriterion.h"

namespace cura
{

class BridgeAngleScoringCriterion : public ScoringCriterion
{
public:
    explicit BridgeAngleScoringCriterion(std::vector<double> scores);

    [[nodiscard]] virtual double computeScore(const size_t candidate_index) const override;

    [[nodiscard]] static constexpr size_t candidatesCount()
    {
        return 180;
    }

    [[nodiscard]] static AngleDegrees candidateIndexToEvaluationAngle(size_t candidate_index);
    [[nodiscard]] static AngleDegrees candidateIndexToExtrusionAngle(size_t candidate_index);
    [[nodiscard]] static double extrusionAngleDistance(const AngleDegrees& first, const AngleDegrees& second);
    [[nodiscard]] static std::vector<double> normalizeScores(const std::vector<double>& raw_scores);

private:
    std::vector<double> scores_;
};

} // namespace cura

#endif // BRIDGE_BRIDGEANGLESCORINGCRITERION_H
