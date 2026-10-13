#include "bridge/LineAnchoringScoringCriterion.h"

#include <range/v3/algorithm/stable_sort.hpp>

#include "bridge/TransformedShape.h"
#include "utils/linearAlg2D.h"


namespace cura
{

LineAnchoringScoringCriterion::LineAnchoringScoringCriterion(
    const std::vector<AngleDegrees>& candidates,
    const Shape& skin_outline,
    const Shape& supported_regions,
    const coord_t line_width)
    : BridgeAngleScoringCriterion(std::move(candidates), skin_outline, supported_regions)
    , line_width_(line_width)
{
}

double LineAnchoringScoringCriterion::computeScore(const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area) const
{
    if (transformed_skin_area.minY() >= transformed_skin_area.maxY() || transformed_supported_area.minY() >= transformed_supported_area.maxY())
    {
        return 0.0;
    }

    const size_t bridge_lines_count = (transformed_skin_area.maxY() - transformed_skin_area.minY()) / line_width_;
    if (bridge_lines_count == 0)
    {
        // We cannot fit a single line in this direction, give up
        return 0.0;
    }

    const coord_t line_min = transformed_skin_area.minY() + line_width_ * 0.5;

    // Evaluated lines that could be properly bridging
    coord_t total_line_score = 0;
    coord_t total_line_length = 0;
    const TransformedShape empty_transformed_shape;
    for (size_t i = 0; i < bridge_lines_count; ++i)
    {
        const coord_t line_y = line_min + i * line_width_;
        const bool has_supports = line_y >= transformed_supported_area.minY() && line_y <= transformed_supported_area.maxY();
        const auto [line_length, line_score] = evaluateBridgeLine(line_y, transformed_skin_area, has_supports ? transformed_supported_area : empty_transformed_shape);
        total_line_length += line_length;
        total_line_score += line_score;
    }

    if (total_line_length == 0)
    {
        return 0.0;
    }

    return std::max(static_cast<double>(total_line_score) / total_line_length, 0.0);
}

std::tuple<coord_t, coord_t>
    LineAnchoringScoringCriterion::evaluateBridgeLine(const coord_t line_y, const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area)
{
    // Calculate intersections with skin outline to see which segments should actually be printed
    std::vector<coord_t> skin_outline_intersections = shapeLineIntersections(line_y, transformed_skin_area);
    if (skin_outline_intersections.size() < 2)
    {
        // We need to enter the skin at some point to bridge inside
        return { 0, 0 };
    }
    ranges::stable_sort(skin_outline_intersections);

    // Calculate intersections with supported regions to see which segments are anchored
    std::vector<coord_t> supported_regions_intersections = shapeLineIntersections(line_y, transformed_supported_area);
    ranges::stable_sort(supported_regions_intersections);

    enum class BridgeStatus
    {
        Outside, // Segment is outside the skin
        Hanging, // Segment has started to extrude over air
        Anchored, // Segment has been anchored to a supported area
        Supported, // Segment is being extruded over a supported area
    };

    // Loop through intersections with skin and supported regions to see which parts of the line are hanging/bridging/supported
    bool inside_skin_area = false;
    bool inside_supported_area = false;
    coord_t last_position;
    coord_t segment_score = 0;
    coord_t total_bridge_length = 0;
    BridgeStatus bridge_status = BridgeStatus::Outside;
    while (! skin_outline_intersections.empty() || ! supported_regions_intersections.empty())
    {
        // See what is the next intersection: skin, supported or both
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
                    // When leaving skin, assume also leaving supported. This should always happen naturally, but may not due to rounding errors.
                    next_intersection_is_supported_area = true;
                }
            }
            else
            {
                next_intersection_is_supported_area = true;
                if (! inside_supported_area && ! inside_skin_area)
                {
                    // When reaching supported, assume also reaching skin. This should always happen naturally, but may not due to rounding errors.
                    next_intersection_is_skin_area = true;
                }
            }
        }

        // Get new insideness states
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
        double add_segment_score_weight = 0;
        const coord_t segment_length = next_intersection - last_position;

        if (bridge_status != BridgeStatus::Outside)
        {
            total_bridge_length += segment_length;
        }

        switch (bridge_status)
        {
        case BridgeStatus::Outside:
            bridge_status = reaching_supported ? BridgeStatus::Supported : BridgeStatus::Hanging;
            break;

        case BridgeStatus::Supported:
            bridge_status = leaving_skin ? BridgeStatus::Outside : BridgeStatus::Anchored;
            // Negatively account for fully supported lines to avoid lonely line parts over the supported areas
            add_segment_score_weight -= 0.1;
            break;

        case BridgeStatus::Hanging:
            add_segment_score_weight -= 1.0;
            bridge_status = reaching_supported ? BridgeStatus::Supported : BridgeStatus::Outside;
            break;

        case BridgeStatus::Anchored:
            if (reaching_supported)
            {
                add_segment_score_weight += 1.0;
                bridge_status = BridgeStatus::Supported;
            }
            else if (leaving_skin)
            {
                add_segment_score_weight -= 1.0;
                bridge_status = BridgeStatus::Outside;
            }
            break;
        }

        if (add_segment_score_weight != 0.0)
        {
            segment_score += std::llrint(segment_length * add_segment_score_weight);
        }

        last_position = next_intersection;
        inside_skin_area = next_inside_skin_area;
        inside_supported_area = next_inside_supported_area;
    }

    return { total_bridge_length, segment_score };
}

std::vector<coord_t> LineAnchoringScoringCriterion::shapeLineIntersections(const coord_t line_y, const TransformedShape& transformed_shape)
{
    std::vector<coord_t> intersections;

    for (const TransformedSegment& transformed_segment : transformed_shape.getSegments())
    {
        if (transformed_segment.minY() > line_y || transformed_segment.maxY() < line_y)
        {
            // Segment is fully over or under the line, skip
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

} // namespace cura
