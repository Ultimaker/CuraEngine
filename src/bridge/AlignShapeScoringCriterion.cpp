#include "bridge/AlignShapeScoringCriterion.h"

#include <range/v3/numeric/accumulate.hpp>
#include <spdlog/spdlog.h>

#include "bridge/TransformedSegment.h"
#include "bridge/TransformedShape.h"


namespace cura
{

AlignShapeScoringCriterion::AlignShapeScoringCriterion(const std::vector<AngleDegrees>& candidates, const Shape& skin_outline, const Shape& supported_regions)
    : BridgeAngleScoringCriterion(std::move(candidates), skin_outline, supported_regions)
{
}

double AlignShapeScoringCriterion::computeScore(const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area) const
{
    if (! total_segments_length_.has_value())
    {
        const_cast<AlignShapeScoringCriterion*>(this)->total_segments_length_ = ranges::accumulate(
            transformed_skin_area.getSegments(),
            coord_t{ 0 },
            [](const coord_t accumulated_length, const TransformedSegment& segment)
            {
                return accumulated_length + segment.length();
            });
    }

    double score = 0.0;
    for (const TransformedSegment& segment : transformed_skin_area.getSegments())
    {
        const Point2LL segment_vector = segment.getEnd() - segment.getStart();
        const double segment_weight = static_cast<double>(segment.length()) / total_segments_length_.value();

        // Segment angle is made absolute so that it is contained in [0, π]
        AngleRadians segment_angle{ std::abs(std::atan2(segment_vector.Y, segment_vector.X)) };

        // We want segments that have an angle far from π/2 (close to horizontal) to have a high score
        const double angle_score = std::abs(std::numbers::pi / 2 - segment_angle) / (std::numbers::pi / 2);

        // Segments that are perfectly horizontal get a much higher score
        const double weighed_score = std::pow(angle_score, 3);

        // Segments get a score relative to their length
        score += segment_weight * weighed_score;
    }

    return score;
}

} // namespace cura
