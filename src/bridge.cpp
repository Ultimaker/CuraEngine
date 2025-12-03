// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge.h"

#include <range/v3/action/stable_sort.hpp>
#include <range/v3/algorithm/reverse.hpp>

#include "LayerPlan.h"
#include "geometry/Point2D.h"
#include "geometry/PointMatrix.h"
#include "geometry/Polygon.h"
#include "settings/EnumSettings.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "utils/AABB.h"
#include "utils/linearAlg2D.h"
#include "utils/math.h"


namespace cura
{

struct TransformedSegment
{
    Point2LL transformed_start;
    Point2LL transformed_end;
    coord_t min_y;
    coord_t max_y;

    TransformedSegment(const Point2LL& transformed_start, const Point2LL& transformed_end)
        : transformed_start(transformed_start)
        , transformed_end(transformed_end)
        , min_y(std::min(transformed_start.Y, transformed_end.Y))
        , max_y(std::max(transformed_start.Y, transformed_end.Y))
    {
    }

    TransformedSegment(const Point2LL& start, const Point2LL& end, const PointMatrix& matrix)
        : TransformedSegment(matrix.apply(start), matrix.apply(end))
    {
    }
};

struct TransformedShape
{
    std::vector<TransformedSegment> segments;
    coord_t min_y{ std::numeric_limits<coord_t>::max() };
    coord_t max_y{ std::numeric_limits<coord_t>::lowest() };
};

/*!
 * Calculates all the intersections between a horizontal line and the given transformed shape
 * @param line_y The horizontal line Y coordinate
 * @param transformed_shape The shape to intersect with
 * @return The list of X coordinates of the intersections, unsorted
 */
std::vector<coord_t> shapeLineIntersections(const coord_t line_y, const TransformedShape& transformed_shape)
{
    std::vector<coord_t> intersections;

    for (const TransformedSegment& transformed_segment : transformed_shape.segments)
    {
        if (transformed_segment.min_y > line_y || transformed_segment.max_y < line_y)
        {
            // Segment is fully over or under the line, skip
            continue;
        }

        const std::optional<coord_t> intersection = LinearAlg2D::lineHorizontalLineIntersection(transformed_segment.transformed_start, transformed_segment.transformed_end, line_y);
        if (intersection.has_value())
        {
            intersections.push_back(intersection.value());
        }
    }

    return intersections;
}

/*!
 * Evaluates a potential bridging line to see if it can actually bridge between two supported regions
 * @param line_y The Y coordinate of the horizontal line
 * @param transformed_skin_area The skin outline, transformed so that the bridging line is horizontal
 * @param transformed_supported_area The supported regions, transformed so that the bridging line is horizontal
 * @return The score of the line regarding bridging, which can be positive if it is mostly bridging, or negative if it is mostly hanging
 *
 * The score is based on the following criteria:
 *   - Properly bridging segments, i.e. between two supported areas, add their length to the score
 *   - Hanging segments, i.e. supported on one side but not the other (or not at all), subtract their length to the score
 *   - Segments that lie on a supported area are not accounted for  */
coord_t evaluateBridgeLine(const coord_t line_y, const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area)
{
    // Calculate intersections with skin outline to see which segments should actually be printed
    std::vector<coord_t> skin_outline_intersections = shapeLineIntersections(line_y, transformed_skin_area);
    if (skin_outline_intersections.size() < 2)
    {
        // We need to enter the skin at some point to bridge inside
        return 0;
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

            if (is_null(next_intersection_skin_area - next_intersection_supported_area))
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
        bool add_bridging_segment = false;
        bool add_hanging_segment = false;

        switch (bridge_status)
        {
        case BridgeStatus::Outside:
            bridge_status = reaching_supported ? BridgeStatus::Supported : BridgeStatus::Hanging;
            break;

        case BridgeStatus::Supported:
            bridge_status = leaving_skin ? BridgeStatus::Outside : BridgeStatus::Anchored;
            break;

        case BridgeStatus::Hanging:
            add_hanging_segment = true;
            bridge_status = reaching_supported ? BridgeStatus::Supported : BridgeStatus::Outside;
            break;

        case BridgeStatus::Anchored:
            if (reaching_supported)
            {
                add_bridging_segment = true;
                bridge_status = BridgeStatus::Supported;
            }
            else if (leaving_skin)
            {
                add_hanging_segment = true;
                bridge_status = BridgeStatus::Outside;
            }
            break;
        }

        if (add_bridging_segment || add_hanging_segment)
        {
            const coord_t segment_length = next_intersection - last_position;
            segment_score += add_bridging_segment ? segment_length : -segment_length;
        }

        last_position = next_intersection;
        inside_skin_area = next_inside_skin_area;
        inside_supported_area = next_inside_supported_area;
    }

    return segment_score;
}

void addTransformSegment(const Point2LL& start, const Point2LL& end, const PointMatrix& matrix, TransformedShape& transformed_shape)
{
    TransformedSegment transformed_segment(start, end, matrix);
    transformed_shape.segments.push_back(transformed_segment);
    transformed_shape.min_y = std::min(transformed_shape.min_y, transformed_segment.min_y);
    transformed_shape.max_y = std::max(transformed_shape.max_y, transformed_segment.max_y);
}

void transformPolygon(const Polygon& polygon, const PointMatrix& matrix, TransformedShape& transformed_shape)
{
    for (auto iterator = polygon.beginSegments(); iterator != polygon.endSegments(); ++iterator)
    {
        addTransformSegment((*iterator).start, (*iterator).end, matrix, transformed_shape);
    }
}

/*!
 * Pre-transforms a shape according to the given matrix, so that the bridging angle ends up being horizontal
 * @param shape The shape to be transformed
 * @param matrix The transform matrix
 * @return The shape transformed to the bridging orientation
 */
TransformedShape transformShape(const Shape& shape, const PointMatrix& matrix)
{
    TransformedShape transformed_shape;
    transformed_shape.segments.reserve(shape.pointCount());

    for (const Polygon& polygon : shape)
    {
        transformPolygon(polygon, matrix, transformed_shape);
    }

    return transformed_shape;
}

/*!
 * Evaluate the bridging lines scoring for the given angle
 * @param skin_outline The skin outline to be filled
 * @param supported_regions The supported regions areas
 * @param line_width The bridging line width
 * @param angle The current angle to be tested
 * @return The global bridging score for this angle */
coord_t evaluateBridgeLines(const Shape& skin_outline, const Shape& supported_regions, const coord_t line_width, const AngleDegrees& angle)
{
    // Transform the skin outline and supported regions according to the angle to speedup intersections calculations
    const PointMatrix matrix(angle);
    const TransformedShape transformed_skin_area = transformShape(skin_outline, matrix);
    const TransformedShape transformed_supported_area = transformShape(supported_regions, matrix);

    if (transformed_skin_area.min_y >= transformed_skin_area.max_y || transformed_supported_area.min_y >= transformed_supported_area.max_y)
    {
        return std::numeric_limits<coord_t>::lowest();
    }

    const size_t bridge_lines_count = (transformed_skin_area.max_y - transformed_skin_area.min_y) / line_width;
    if (bridge_lines_count == 0)
    {
        // We cannot fit a single line in this direction, give up
        return std::numeric_limits<coord_t>::lowest();
    }

    const coord_t line_min = transformed_skin_area.min_y + line_width * 0.5;

    // Evaluated lines that could be properly bridging
    coord_t line_score = 0;
    const TransformedShape empty_transformed_shape;
    for (size_t i = 0; i < bridge_lines_count; ++i)
    {
        const coord_t line_y = line_min + i * line_width;
        const bool has_supports = line_y >= transformed_supported_area.min_y && line_y <= transformed_supported_area.max_y;
        line_score += evaluateBridgeLine(line_y, transformed_skin_area, has_supports ? transformed_supported_area : empty_transformed_shape);
    }

    return line_score;
}

/*!
 * Gets the proper angle when bridging over infill
 * @param mesh The mesh being processed, which contains the actual infill angles
 * @param layer_nr The number of the layer being processed
 * @return The angle to be applied to bridging over infill on this layer
 */
AngleDegrees bridgeOverInfillAngle(const SliceMeshStorage& mesh, const unsigned layer_nr)
{
    if (layer_nr == 0)
    {
        // Obviously there is no infill below
        return 0;
    }

    const std::vector<AngleDegrees>& infill_angles = mesh.infill_angles;
    assert(! infill_angles.empty());

    const AngleDegrees infill_angle_below = infill_angles[(layer_nr - 1) % infill_angles.size()];
    const auto infill_pattern = mesh.settings.get<EFillMethod>("infill_pattern");

    AngleDegrees bridge_angle;

    switch (infill_pattern)
    {
    case EFillMethod::CROSS:
    case EFillMethod::CROSS_3D:
        bridge_angle = 22.5;
        break;
    case EFillMethod::GYROID:
    case EFillMethod::CONCENTRIC:
    case EFillMethod::LIGHTNING:
    case EFillMethod::PLUGIN:
    case EFillMethod::NONE:
        bridge_angle = 45;
        break;
    case EFillMethod::CUBICSUBDIV:
        bridge_angle = infill_angle_below + 45;
        break;
    case EFillMethod::TRIANGLES:
    case EFillMethod::LINES:
    case EFillMethod::TRIHEXAGON:
    case EFillMethod::CUBIC:
    case EFillMethod::ZIG_ZAG:
        bridge_angle = infill_angle_below + 90;
        break;
    case EFillMethod::QUARTER_CUBIC:
    case EFillMethod::TETRAHEDRAL:
    case EFillMethod::GRID:
        bridge_angle = infill_angle_below;
        break;
    }

    return bridge_angle;
}

std::optional<AngleDegrees> bridgeAngle(
    const SliceMeshStorage& mesh,
    const Shape& skin_outline,
    const SliceDataStorage& storage,
    const unsigned layer_nr,
    const unsigned bridge_layer,
    const SupportLayer* support_layer,
    Shape& supported_regions)
{
    const Settings& settings = mesh.settings;
    const bool bridge_settings_enabled = settings.get<bool>("bridge_settings_enabled");
    if (! bridge_settings_enabled)
    {
        return std::nullopt;
    }

    AABB boundary_box(skin_outline);

    const coord_t line_width = settings.get<coord_t>("skin_line_width");

    // To detect if we have a bridge, first calculate the intersection of the current layer with the previous layer.
    //  This gives us the islands that the layer rests on.
    Shape islands;

    Shape prev_layer_outline; // we also want the complete outline of the previous layer
    Shape prev_layer_infill;

    const Ratio sparse_infill_max_density = settings.get<Ratio>("bridge_sparse_infill_max_density");

    // include parts from all meshes
    for (const std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
    {
        const auto& mesh = *mesh_ptr;
        if (mesh.isPrinted())
        {
            const coord_t infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
            const coord_t infill_line_width = mesh.settings.get<coord_t>("infill_line_width");
            double density = static_cast<double>(infill_line_width) / static_cast<double>(infill_line_distance);
            const bool part_has_sparse_infill = (infill_line_distance == 0) || density <= sparse_infill_max_density;

            for (const SliceLayerPart& prev_layer_part : mesh.layers[layer_nr - bridge_layer].parts)
            {
                Shape prev_layer_part_infill = prev_layer_part.getOwnInfillArea();
                prev_layer_infill = prev_layer_infill.unionPolygons(prev_layer_part_infill);

                Shape solid_below(prev_layer_part.outline);
                if (bridge_layer == 1 && part_has_sparse_infill)
                {
                    solid_below = solid_below.difference(prev_layer_part_infill);
                }
                prev_layer_outline.push_back(solid_below); // not intersected with skin

                if (! boundary_box.hit(prev_layer_part.boundaryBox))
                    continue;

                islands.push_back(skin_outline.intersection(solid_below));
            }
        }
    }
    supported_regions = islands;

    if (support_layer)
    {
        // add the regions of the skin that have support below them to supportedRegions
        // but don't add these regions to islands because that can actually cause the code
        // below to consider the skin a bridge when it isn't (e.g. a skin that is supported by
        // the model on one side but the remainder of the skin is above support would look like
        // a bridge because it would have two islands) - FIXME more work required here?

        if (! support_layer->support_roof.empty())
        {
            AABB support_roof_bb(support_layer->support_roof);
            if (boundary_box.hit(support_roof_bb))
            {
                prev_layer_outline.push_back(support_layer->support_roof); // not intersected with skin

                Shape supported_skin(skin_outline.intersection(support_layer->support_roof));
                if (! supported_skin.empty())
                {
                    supported_regions.push_back(supported_skin);
                }
            }
        }
        else
        {
            for (const SupportInfillPart& support_part : support_layer->support_infill_parts)
            {
                AABB support_part_bb(support_part.getInfillArea());
                if (boundary_box.hit(support_part_bb))
                {
                    prev_layer_outline.push_back(support_part.getInfillArea()); // not intersected with skin

                    Shape supported_skin(skin_outline.intersection(support_part.getInfillArea()));
                    if (! supported_skin.empty())
                    {
                        supported_regions.push_back(supported_skin);
                    }
                }
            }
        }
    }

    const Ratio support_threshold = settings.get<Ratio>("bridge_skin_support_threshold");
    if (support_threshold == 0 || (supported_regions.area() / (skin_outline.area() + 1)) >= support_threshold)
    {
        // as the proportion of the skin region that is supported is >= supportThreshold, it's not
        // considered to be a bridge and the original bridge detection code below is skipped
        return std::nullopt;
    }

    prev_layer_infill = skin_outline.intersection(prev_layer_infill);
    const Ratio infill_ratio = prev_layer_infill.area() / (skin_outline.area() + 1);
    if (infill_ratio > 0.5) // In practice, the ratio should always be close to 0 or 1, so 0.5 should be good enough
    {
        // We are doing bridging over infill, so use the infill angle instead of trying to calculate a proper angle
        return bridgeOverInfillAngle(mesh, layer_nr);
    }

    struct FitAngle
    {
        coord_t score;
        std::optional<AngleDegrees> angle;
    };

    FitAngle best_angle{ std::numeric_limits<coord_t>::lowest(), std::nullopt };
    for (AngleDegrees angle = 0; angle < 180; angle += 1)
    {
        const coord_t score = evaluateBridgeLines(skin_outline, supported_regions, line_width, angle);
        if (score > best_angle.score)
        {
            best_angle = { score, angle + 90 };
        }
    }

    return best_angle.angle;
}

void expandSegment(const TransformedSegment& segment, const std::vector<TransformedSegment>& infill_lines_below, ClosedPolyline& expanded_infill_below_skin_area, const SVG* svg);

void expandHorizontalSegment(const TransformedSegment& segment, const std::vector<TransformedSegment>& infill_lines_on_same_band, ClosedPolyline& expanded_infill_below_skin_area)
{
}

void expandNonHorizontalSegment(
    const TransformedSegment& segment,
    const std::vector<TransformedSegment>& infill_lines_on_same_band,
    ClosedPolyline& expanded_infill_below_skin_area,
    const SVG* svg)
{
    std::vector<float> intersections;
    for (const TransformedSegment& infill_line_on_same_band : infill_lines_on_same_band)
    {
        float intersection_t;
        float intersection_u;
        if (LinearAlg2D::segmentSegmentIntersection(
                segment.transformed_start,
                segment.transformed_end,
                infill_line_on_same_band.transformed_start,
                infill_line_on_same_band.transformed_end,
                intersection_t,
                intersection_u)
            && intersection_t > 0.0 && intersection_t < 1.0)
        {
            intersections.push_back(intersection_t);
        }
    }

    if (! intersections.empty())
    {
        bool sub_segments_processed = false;
        ranges::stable_sort(intersections);
        intersections.insert(intersections.begin(), 0.0);
        intersections.push_back(1.0);

        const Point2LL segment_vector = segment.transformed_end - segment.transformed_start;

        for (const auto& sub_segment_t : intersections | ranges::views::sliding(2))
        {
            const TransformedSegment sub_segment(segment.transformed_start + segment_vector * sub_segment_t[0], segment.transformed_start + segment_vector * sub_segment_t[1]);
            if (sub_segment.transformed_start == sub_segment.transformed_end
                || (sub_segment.transformed_start == segment.transformed_start && sub_segment.transformed_end == segment.transformed_end))
            {
                continue;
            }

            expandSegment(sub_segment, infill_lines_on_same_band, expanded_infill_below_skin_area, svg);
            sub_segments_processed = true;
        }

        if (sub_segments_processed)
        {
            return;
        }
    }

    const int8_t expand_direction = segment.transformed_end.Y > segment.transformed_start.Y ? 1 : -1;

    svg->write(segment.transformed_start, segment.transformed_end, { .line = { SVG::Color::BLACK, 0.35 } });

    std::vector<TransformedSegment> infill_lines_on_expansion_side;
    for (const TransformedSegment& infill_line_on_same_band : infill_lines_on_same_band)
    {
#warning optimize by doing those 2 calculations at once
        const coord_t y_min = std::max(segment.min_y, infill_line_on_same_band.min_y);
        const coord_t y_max = std::min(segment.max_y, infill_line_on_same_band.max_y);

        if (y_min == y_max)
        {
        }
        else
        {
            const coord_t segment_x_min = LinearAlg2D::lineHorizontalLineIntersection(segment.transformed_start, segment.transformed_end, y_min).value();
            const coord_t segment_x_max = LinearAlg2D::lineHorizontalLineIntersection(segment.transformed_start, segment.transformed_end, y_max).value();
            const coord_t infill_line_x_min
                = LinearAlg2D::lineHorizontalLineIntersection(infill_line_on_same_band.transformed_start, infill_line_on_same_band.transformed_end, y_min).value();
            const coord_t infill_line_x_max
                = LinearAlg2D::lineHorizontalLineIntersection(infill_line_on_same_band.transformed_start, infill_line_on_same_band.transformed_end, y_max).value();

            if ((infill_line_x_min != segment_x_min || infill_line_x_max != segment_x_max)
                && ((expand_direction > 0 && infill_line_x_min >= segment_x_min && infill_line_x_max >= segment_x_max)
                    || (expand_direction < 0 && infill_line_x_min <= segment_x_min && infill_line_x_max <= segment_x_max)))
            {
                infill_lines_on_expansion_side.push_back(infill_line_on_same_band);
            }
        }
    }

    std::set<coord_t> scan_positions_y_set;
    scan_positions_y_set.insert(segment.transformed_start.Y);
    scan_positions_y_set.insert(segment.transformed_end.Y);
    for (const TransformedSegment& infill_line_on_expansion_side : infill_lines_on_expansion_side)
    {
        if (infill_line_on_expansion_side.transformed_start.Y > segment.min_y && infill_line_on_expansion_side.transformed_start.Y < segment.max_y)
        {
            scan_positions_y_set.insert(infill_line_on_expansion_side.transformed_start.Y);
        }
        if (infill_line_on_expansion_side.transformed_end.Y > segment.min_y && infill_line_on_expansion_side.transformed_end.Y < segment.max_y)
        {
            scan_positions_y_set.insert(infill_line_on_expansion_side.transformed_end.Y);
        }
    }

    if (infill_lines_on_expansion_side.empty())
    {
        expanded_infill_below_skin_area.push_back(segment.transformed_start);
        expanded_infill_below_skin_area.push_back(segment.transformed_end);
        return;
    }

    // for (const TransformedSegment& infill_line_on_expansion_side : infill_lines_on_expansion_side)
    // {
    //     svg->write(infill_line_on_expansion_side.transformed_start, infill_line_on_expansion_side.transformed_end, { .line = { SVG::Color::YELLOW, 0.2 } });
    // }

    std::vector<coord_t> scan_positions_y(scan_positions_y_set.begin(), scan_positions_y_set.end());
    if (expand_direction < 0)
    {
        ranges::reverse(scan_positions_y);
    }

    const TransformedSegment* current_infill_line_segment = nullptr;
    constexpr bool only_if_forming_segment = true;
    for (const coord_t scan_line_y : scan_positions_y)
    {
        const coord_t scan_line_segment_x = LinearAlg2D::lineHorizontalLineIntersection(segment.transformed_start, segment.transformed_end, scan_line_y).value();
        std::optional<coord_t> current_infill_line_segment_intersection;
        std::optional<coord_t> closest_intersection;
        const TransformedSegment* closest_intersection_segment = nullptr;
        // std::vector<const TransformedSegment*> closest_intersection_segments;
        for (const TransformedSegment& infill_line_on_expansion_side : infill_lines_on_expansion_side)
        {
            const std::optional<coord_t> intersection
                = LinearAlg2D::segmentHorizontalLineIntersection(infill_line_on_expansion_side.transformed_start, infill_line_on_expansion_side.transformed_end, scan_line_y);
            if (intersection.has_value())
            {
                if (&infill_line_on_expansion_side == current_infill_line_segment)
                {
                    current_infill_line_segment_intersection = *intersection;
                }

                if (expand_direction > 0)
                {
                    if (*intersection >= scan_line_segment_x && (! closest_intersection.has_value() || *intersection < *closest_intersection))
                    {
                        closest_intersection = *intersection;
                        closest_intersection_segment = &infill_line_on_expansion_side;
                    }
                }
                else
                {
                    if (*intersection <= scan_line_segment_x && (! closest_intersection.has_value() || *intersection > *closest_intersection))
                    {
                        closest_intersection = *intersection;
                        closest_intersection_segment = &infill_line_on_expansion_side;
                    }
                }
            }
        }

        if (! closest_intersection.has_value())
        {
            spdlog::warn("No intersection found while expanding infill");
            expanded_infill_below_skin_area.push_back(segment.transformed_start);
            expanded_infill_below_skin_area.push_back(segment.transformed_end);
            break;
        }

        if (current_infill_line_segment != nullptr)
        {
            if (current_infill_line_segment_intersection.has_value())
            {
                if (expanded_infill_below_skin_area.push_back(Point2LL(*current_infill_line_segment_intersection, scan_line_y), only_if_forming_segment) && svg != nullptr
                    && expanded_infill_below_skin_area.isValid())
                {
                    svg->write(
                        expanded_infill_below_skin_area[expanded_infill_below_skin_area.size() - 2],
                        expanded_infill_below_skin_area.back(),
                        { .line = { SVG::Color::GREEN, 0.3 } });
                }
            }
            else
            {
                spdlog::warn("No intersection found with current segment");
            }
        }

        if (expanded_infill_below_skin_area.push_back(Point2LL(*closest_intersection, scan_line_y), only_if_forming_segment) && svg != nullptr
            && expanded_infill_below_skin_area.isValid())
        {
            svg->write(expanded_infill_below_skin_area[expanded_infill_below_skin_area.size() - 2], expanded_infill_below_skin_area.back(), { .line = { SVG::Color::BLUE, 0.3 } });
        }

        current_infill_line_segment = closest_intersection_segment;
    }
}

void expandSegment(const TransformedSegment& segment, const std::vector<TransformedSegment>& infill_lines_below, ClosedPolyline& expanded_infill_below_skin_area, const SVG* svg)
{
    std::vector<TransformedSegment> infill_lines_on_same_band;
    for (const TransformedSegment& infill_line_below : infill_lines_below)
    {
        if (infill_line_below.min_y <= segment.max_y && infill_line_below.max_y >= segment.min_y)
        {
            infill_lines_on_same_band.push_back(infill_line_below);
        }
    }

    if (segment.min_y == segment.max_y)
    {
        expandHorizontalSegment(segment, infill_lines_on_same_band, expanded_infill_below_skin_area);
    }
    else
    {
        expandNonHorizontalSegment(segment, infill_lines_on_same_band, expanded_infill_below_skin_area, svg);
    }
}

std::tuple<Shape, AngleDegrees> makeBridgeOverInfillPrintable(
    const Shape& infill_contour,
    const Shape& infill_below_skin_area,
    const SliceMeshStorage& mesh,
    const LayerPlan* completed_layer_plan_below,
    const unsigned layer_nr)
{
    if (layer_nr == 0 || infill_below_skin_area.empty())
    {
        return {};
    }

    SVG svg(fmt::format("/tmp/bridge_infill_lines_{}.svg", layer_nr), AABB(infill_contour));
    // svg.write(infill_contour, { .line = { SVG::Color::RED, 0.35 } });
    // svg.write(completed_layer_plan_below->getGeneratedInfillLines(), { .line = { 0.3 } });

    const AngleDegrees bridge_angle = bridgeOverInfillAngle(mesh, layer_nr);
    const PointMatrix matrix(bridge_angle + 90);
    const MixedLinesSet& infill_lines_below = completed_layer_plan_below->getGeneratedInfillLines();
    TransformedShape filtered_infill_lines_below;
    constexpr AngleDegrees min_angle_delta(1);

    for (const PolylinePtr& infill_line_below : infill_lines_below)
    {
        for (auto iterator = infill_line_below->cbeginSegments(); iterator != infill_line_below->cendSegments(); ++iterator)
        {
            const Point2LL& start = (*iterator).start;
            const Point2LL& end = (*iterator).end;
            const AngleDegrees segment_angle = vAngle(end - start) + AngleDegrees(90);
            if (nonOrientedDelta(segment_angle, bridge_angle + 90) >= min_angle_delta)
            {
                addTransformSegment(start, end, matrix, filtered_infill_lines_below);
            }
        }
    }

    for (const Polygon& polygon : infill_contour)
    {
        for (auto iterator = polygon.beginSegments(); iterator != polygon.endSegments(); ++iterator)
        {
            addTransformSegment((*iterator).start, (*iterator).end, matrix, filtered_infill_lines_below);
        }
    }
    for (const TransformedSegment& filtered_infill_line_below : filtered_infill_lines_below.segments)
    {
        svg.write(filtered_infill_line_below.transformed_start, filtered_infill_line_below.transformed_end, { .line = { SVG::Color::MAGENTA, 0.4 } });
    }

    Shape transformed_expanded_infill_below_skin_area;
    for (const Polygon& infill_below_skin_polygon : infill_below_skin_area)
    {
        Polygon expanded_polygon;
        for (auto iterator = infill_below_skin_polygon.beginSegments(); iterator != infill_below_skin_polygon.endSegments(); ++iterator)
        {
            TransformedSegment transformed_infill_below_skin_segment((*iterator).start, (*iterator).end, matrix);
            expandSegment(transformed_infill_below_skin_segment, filtered_infill_lines_below.segments, expanded_polygon, &svg);
        }
        transformed_expanded_infill_below_skin_area.push_back(std::move(expanded_polygon));
    }
    svg.write(transformed_expanded_infill_below_skin_area, { .surface = { SVG::Color::RED, 0.3 } });

    // for (Polygon& polygon : transformed_expanded_infill_below_skin_area)
    // {
    //     for (Point2LL& point : polygon)
    //     {
    //         point = matrix.unapply(point);
    //     }
    // }
    transformed_expanded_infill_below_skin_area.applyMatrix(matrix.inverse());

    // transformed_expanded_infill_below_skin_area = transformed_expanded_infill_below_skin_area.offset(5).offset(-5);

    svg.write(transformed_expanded_infill_below_skin_area, { .surface = { SVG::Color::GREEN, 0.3 } });

    return { transformed_expanded_infill_below_skin_area, bridge_angle };
}

} // namespace cura
