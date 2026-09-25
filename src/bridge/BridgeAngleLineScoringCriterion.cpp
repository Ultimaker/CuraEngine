#include "bridge/BridgeAngleLineScoringCriterion.h"

#include <cmath>
#include <limits>
#include <optional>

#include <range/v3/action/stable_sort.hpp>

#include "bridge/TransformedShape.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "utils/linearAlg2D.h"

namespace cura
{
namespace
{

std::vector<coord_t> shapeLineIntersections(const coord_t line_y, const TransformedShape& transformed_shape)
{
    std::vector<coord_t> intersections;

    for (const TransformedSegment& transformed_segment : transformed_shape.getSegments())
    {
        if (transformed_segment.minY() > line_y || transformed_segment.maxY() < line_y)
        {
            continue;
        }

        const std::optional<coord_t> intersection = LinearAlg2D::lineHorizontalLineIntersection(transformed_segment.getStart(), transformed_segment.getEnd(), line_y);
        if (intersection.has_value())
        {
            intersections.push_back(intersection.value());
        }
    }

    return intersections;
}

coord_t evaluateBridgeLine(const coord_t line_y, const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area)
{
    std::vector<coord_t> skin_outline_intersections = shapeLineIntersections(line_y, transformed_skin_area);
    if (skin_outline_intersections.size() < 2)
    {
        return 0;
    }
    ranges::stable_sort(skin_outline_intersections);

    std::vector<coord_t> supported_regions_intersections = shapeLineIntersections(line_y, transformed_supported_area);
    ranges::stable_sort(supported_regions_intersections);

    enum class BridgeStatus
    {
        Outside,
        Hanging,
        Anchored,
        Supported,
    };

    bool inside_skin_area = false;
    bool inside_supported_area = false;
    coord_t last_position;
    coord_t segment_score = 0;
    BridgeStatus bridge_status = BridgeStatus::Outside;
    while (! skin_outline_intersections.empty() || ! supported_regions_intersections.empty())
    {
        bool next_intersection_is_skin_area = false;
        bool next_intersection_is_supported_area = false;
        if (skin_outline_intersections.empty())
        {
            next_intersection_is_supported_area = true;
        }
        else if (supported_regions_intersections.empty())
        {
            next_intersection_is_skin_area = true;
        }
        else
        {
            const double next_intersection_skin_area = skin_outline_intersections.front();
            const double next_intersection_supported_area = supported_regions_intersections.front();

            if (is_zero(next_intersection_skin_area - next_intersection_supported_area))
            {
                next_intersection_is_skin_area = true;
                next_intersection_is_supported_area = true;
            }
            else if (next_intersection_skin_area <= next_intersection_supported_area)
            {
                next_intersection_is_skin_area = true;
                if (inside_skin_area && inside_supported_area)
                {
                    next_intersection_is_supported_area = true;
                }
            }
            else
            {
                next_intersection_is_supported_area = true;
                if (! inside_supported_area && ! inside_skin_area)
                {
                    next_intersection_is_skin_area = true;
                }
            }
        }

        bool next_inside_skin_area = inside_skin_area;
        bool next_inside_supported_area = inside_supported_area;
        coord_t next_intersection;
        if (next_intersection_is_skin_area)
        {
            next_intersection = skin_outline_intersections.front();
            skin_outline_intersections.erase(skin_outline_intersections.begin());
            next_inside_skin_area = ! next_inside_skin_area;
        }
        if (next_intersection_is_supported_area)
        {
            next_intersection = supported_regions_intersections.front();
            supported_regions_intersections.erase(supported_regions_intersections.begin());
            next_inside_supported_area = ! next_inside_supported_area;
        }

        const bool leaving_skin = next_intersection_is_skin_area && ! next_inside_skin_area;
        const bool reaching_supported = next_intersection_is_supported_area && next_inside_supported_area;
        double add_segment_score_weight = 0.0;

        switch (bridge_status)
        {
        case BridgeStatus::Outside:
            bridge_status = reaching_supported ? BridgeStatus::Supported : BridgeStatus::Hanging;
            break;

        case BridgeStatus::Supported:
            bridge_status = leaving_skin ? BridgeStatus::Outside : BridgeStatus::Anchored;
            add_segment_score_weight = -0.1;
            break;

        case BridgeStatus::Hanging:
            add_segment_score_weight = -1.0;
            bridge_status = reaching_supported ? BridgeStatus::Supported : BridgeStatus::Outside;
            break;

        case BridgeStatus::Anchored:
            if (reaching_supported)
            {
                add_segment_score_weight = 1.0;
                bridge_status = BridgeStatus::Supported;
            }
            else if (leaving_skin)
            {
                add_segment_score_weight = -1.0;
                bridge_status = BridgeStatus::Outside;
            }
            break;
        }

        if (add_segment_score_weight != 0.0)
        {
            const coord_t segment_length = next_intersection - last_position;
            segment_score += std::llrint(segment_length * add_segment_score_weight);
        }

        last_position = next_intersection;
        inside_skin_area = next_inside_skin_area;
        inside_supported_area = next_inside_supported_area;
    }

    return segment_score;
}

coord_t evaluateBridgeLines(const Shape& skin_outline, const Shape& supported_regions, const coord_t line_width, const AngleDegrees& angle)
{
    const PointMatrix matrix(angle);
    const TransformedShape transformed_skin_area(skin_outline, matrix);
    const TransformedShape transformed_supported_area(supported_regions, matrix);

    if (transformed_skin_area.minY() >= transformed_skin_area.maxY() || transformed_supported_area.minY() >= transformed_supported_area.maxY())
    {
        return std::numeric_limits<coord_t>::lowest();
    }

    const size_t bridge_lines_count = (transformed_skin_area.maxY() - transformed_skin_area.minY()) / line_width;
    if (bridge_lines_count == 0)
    {
        return std::numeric_limits<coord_t>::lowest();
    }

    const coord_t line_min = transformed_skin_area.minY() + line_width * 0.5;

    coord_t line_score = 0;
    const TransformedShape empty_transformed_shape;
    for (size_t i = 0; i < bridge_lines_count; ++i)
    {
        const coord_t line_y = line_min + i * line_width;
        const bool has_supports = line_y >= transformed_supported_area.minY() && line_y <= transformed_supported_area.maxY();
        line_score += evaluateBridgeLine(line_y, transformed_skin_area, has_supports ? transformed_supported_area : empty_transformed_shape);
    }

    return line_score;
}

std::vector<double> buildLineScores(const Shape& skin_outline, const Shape& supported_regions, const coord_t line_width)
{
    std::vector<double> raw_scores;
    raw_scores.reserve(BridgeAngleScoringCriterion::candidatesCount());
    for (size_t candidate_index = 0; candidate_index < BridgeAngleScoringCriterion::candidatesCount(); ++candidate_index)
    {
        raw_scores.push_back(static_cast<double>(
            evaluateBridgeLines(skin_outline, supported_regions, line_width, BridgeAngleScoringCriterion::candidateIndexToEvaluationAngle(candidate_index))));
    }

    return BridgeAngleScoringCriterion::normalizeScores(raw_scores);
}

} // namespace

BridgeAngleLineScoringCriterion::BridgeAngleLineScoringCriterion(const Shape& skin_outline, const Shape& supported_regions, const coord_t line_width)
    : BridgeAngleScoringCriterion(buildLineScores(skin_outline, supported_regions, line_width))
{
}

} // namespace cura
