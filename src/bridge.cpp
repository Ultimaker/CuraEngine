// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge.h"

#include <range/v3/action/stable_sort.hpp>
#include <range/v3/algorithm/reverse.hpp>
#include <spdlog/stopwatch.h>

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
#include "utils/types/geometry.h"


namespace cura
{

enum class SegmentOverlappingType
{
    Full,
    Bottom,
    Top,
    Middle,
};

struct SegmentOverlapping;

struct TransformedSegment
{
    Point2LL start;
    Point2LL end;
    coord_t min_y{ 0 };
    coord_t max_y{ 0 };

    TransformedSegment() = default;

    TransformedSegment(const Point2LL& transformed_start, const Point2LL& transformed_end)
        : start(transformed_start)
        , end(transformed_end)
    {
        updateMinMax();
    }

    TransformedSegment(const Point2LL& start, const Point2LL& end, const PointMatrix& matrix)
        : TransformedSegment(matrix.apply(start), matrix.apply(end))
    {
    }

    void setStart(const Point2LL& transformed_start)
    {
        start = transformed_start;
        updateMinMax();
    }

    void setEnd(const Point2LL& transformed_end)
    {
        end = transformed_end;
        updateMinMax();
    }

    void updateMinMax()
    {
        min_y = std::min(start.Y, end.Y);
        max_y = std::max(start.Y, end.Y);
    }

    std::optional<SegmentOverlapping> calculateOverlapping(const TransformedSegment& other, const int8_t expand_direction) const;

    void cropTop(const coord_t new_max_y)
    {
        setEnd(Point2LL(LinearAlg2D::lineHorizontalLineIntersection(start, end, new_max_y).value_or(0), new_max_y));
    }

    void cropBottom(const coord_t new_min_y)
    {
        setStart(Point2LL(LinearAlg2D::lineHorizontalLineIntersection(start, end, new_min_y).value_or(0), new_min_y));
    }

    static SegmentOverlappingType makeNonInteresectingOverlapping(const bool overlap_top, const bool overlap_bottom)
    {
        if (overlap_top && overlap_bottom)
        {
            return SegmentOverlappingType::Full;
        }
        else if (overlap_bottom)
        {
            return SegmentOverlappingType::Bottom;
        }
        else if (overlap_top)
        {
            return SegmentOverlappingType::Top;
        }
        else
        {
            return SegmentOverlappingType::Middle;
        }
    }
};

struct TransformedShape
{
    std::vector<TransformedSegment> segments;
    coord_t min_y{ std::numeric_limits<coord_t>::max() };
    coord_t max_y{ std::numeric_limits<coord_t>::lowest() };

    void addSegment(TransformedSegment&& segment)
    {
        segments.push_back(segment);
        min_y = std::min(min_y, segment.min_y);
        max_y = std::max(max_y, segment.max_y);
    }
};

struct SegmentOverlappingData
{
    coord_t y_min;
    coord_t y_max;
    coord_t this_x_min;
    coord_t this_x_max;
    coord_t other_x_min;
    coord_t other_x_max;

    SegmentOverlappingData(
        const coord_t this_min_y,
        const coord_t this_max_y,
        const coord_t other_min_y,
        const coord_t other_max_y,
        const Point2LL* this_start,
        const Point2LL* this_end,
        const Point2LL& other_start,
        const Point2LL& other_end)
        : y_min(std::max(this_min_y, other_min_y))
        , y_max(std::min(this_max_y, other_max_y))
        , this_x_min(this_start != nullptr ? LinearAlg2D::lineHorizontalLineIntersection(*this_start, *this_end, y_min).value() : 0)
        , this_x_max(this_start != nullptr ? LinearAlg2D::lineHorizontalLineIntersection(*this_start, *this_end, y_max).value() : 0)
        , other_x_min(LinearAlg2D::lineHorizontalLineIntersection(other_start, other_end, y_min).value())
        , other_x_max(LinearAlg2D::lineHorizontalLineIntersection(other_start, other_end, y_max).value())
    {
    }

    TransformedSegment makeOtherOverlappingPart() const
    {
        return TransformedSegment(Point2LL(other_x_min, y_min), Point2LL(other_x_max, y_max));
    }
};

struct SegmentOverlapping
{
    SegmentOverlappingType type;
    TransformedSegment other_overlapping_part;
};

std::optional<SegmentOverlapping> TransformedSegment::calculateOverlapping(const TransformedSegment& other, const int8_t expand_direction) const
{
    if (fuzzy_is_greater_or_equal(other.min_y, max_y) || fuzzy_is_lesser_or_equal(other.max_y, min_y))
    {
        // Not on the same horizontal band, or very slightly overlapping , discard
        return std::nullopt;
    }

    SegmentOverlappingData overlapping(min_y, max_y, other.min_y, other.max_y, &start, &end, other.start, other.end);

    if (fuzzy_equal(overlapping.this_x_min, overlapping.other_x_min) && fuzzy_equal(overlapping.this_x_max, overlapping.other_x_max))
    {
        // Segments are on top of each other, discard
        return std::nullopt;
    }

    int8_t sign_min = sign(overlapping.other_x_min - overlapping.this_x_min);
    int8_t sign_max = sign(overlapping.other_x_max - overlapping.this_x_max);
    const bool overlap_top = (overlapping.y_max == max_y);
    const bool overlap_bottom = (overlapping.y_min == min_y);

    TransformedSegment line_part = overlapping.makeOtherOverlappingPart();

    if (sign_min != sign_max)
    {
        // Segments are apparently intersecting each other
        float intersection_segment;
        float intersection_infill_line;
        if (LinearAlg2D::segmentSegmentIntersection(start, end, other.start, other.end, intersection_segment, intersection_infill_line))
        {
            const Point2LL intersection = lerp(start, end, intersection_segment);
            if (intersection.Y >= overlapping.y_max - EPSILON)
            {
                // Intersection is very close from top, consider as if not intersecting
                sign_max = sign_min;
            }
            else if (intersection.Y <= overlapping.y_min + EPSILON)
            {
                // Intersection is very close from bottom, consider as if not intersecting
                sign_min = sign_max;
            }
            else
            {
                SegmentOverlappingType type;

                if (sign_max == expand_direction)
                {
                    type = overlap_top ? SegmentOverlappingType::Top : SegmentOverlappingType::Middle;
                    line_part.setStart(intersection);
                }
                else
                {
                    type = overlap_bottom ? SegmentOverlappingType::Bottom : SegmentOverlappingType::Middle;
                    line_part.setEnd(intersection);
                }

                return SegmentOverlapping{ type, line_part };
            }
        }
    }

    if (sign_min != expand_direction)
    {
        // Segment is on the non-expanding side, discard
        return std::nullopt;
    }
    else
    {
        return SegmentOverlapping{ makeNonInteresectingOverlapping(overlap_top, overlap_bottom), line_part };
    }
}

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

        const std::optional<coord_t> intersection = LinearAlg2D::lineHorizontalLineIntersection(transformed_segment.start, transformed_segment.end, line_y);
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

void transformPolygon(const Polygon& polygon, const PointMatrix& matrix, TransformedShape& transformed_shape)
{
    for (auto iterator = polygon.beginSegments(); iterator != polygon.endSegments(); ++iterator)
    {
        transformed_shape.addSegment(TransformedSegment((*iterator).start, (*iterator).end, matrix));
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

struct ExpansionRange
{
    union
    {
        TransformedSegment segment;
        struct
        {
            coord_t min_y;
            coord_t max_y;
        } range;
    } data;

    const TransformedSegment* supporting_segment;
    bool is_set;

    ExpansionRange(const TransformedSegment& segment, const TransformedSegment* supporting_segment)
        : data{ .segment = segment }
        , supporting_segment(supporting_segment)
        , is_set(true)
    {
    }

    ExpansionRange(const coord_t min_y, const coord_t max_y, const TransformedSegment* supporting_segment)
        : data{ .range = { .min_y = min_y, .max_y = max_y } }
        , is_set(false)
        , supporting_segment(supporting_segment)
    {
    }

    coord_t y_min() const
    {
        return is_set ? data.segment.min_y : data.range.min_y;
    }

    coord_t y_max() const
    {
        return is_set ? data.segment.max_y : data.range.max_y;
    }

    std::optional<SegmentOverlapping> calculateOverlapping(const TransformedSegment& other, const int8_t expand_direction) const
    {
        if (is_set)
        {
            return data.segment.calculateOverlapping(other, expand_direction);
        }

        if (other.min_y > (y_max() - EPSILON) || other.max_y < (y_min() - EPSILON))
        {
            // Not on the same horizontal band, or very slightly overlapping , discard
            return std::nullopt;
        }

        const SegmentOverlappingData overlapping(data.range.min_y, data.range.max_y, other.min_y, other.max_y, nullptr, nullptr, other.start, other.end);

        const bool overlap_top = (other.max_y == data.range.max_y);
        const bool overlap_bottom = (other.min_y == data.range.min_y);

        return SegmentOverlapping{ TransformedSegment::makeNonInteresectingOverlapping(overlap_top, overlap_bottom), overlapping.makeOtherOverlappingPart() };
    }

    void cropTop(const coord_t new_max_y)
    {
        if (is_set)
        {
            data.segment.cropTop(new_max_y);
        }
        else
        {
            data.range.max_y = new_max_y;
        }
    }

    void cropBottom(const coord_t new_min_y)
    {
        if (is_set)
        {
            data.segment.cropBottom(new_min_y);
        }
        else
        {
            data.range.min_y = new_min_y;
        }
    }

    bool isValid() const
    {
        return fuzzy_not_equal(y_max(), y_min());
    }
};

/*!
 * Expands a segment of a polygon so that bridging will always have supporting infill lines below for proper anchoring
 * @param segment The current segment to be expanded
 * @param infill_lines_below The infill lines on the layer below
 * @param expanded_polygon The resulting expanded polygon
 * @param current_supporting_infill_line The infill line below currently supporting the bridging, which will be updated over iterations when necessary
 */
void expandSegment(
    const TransformedSegment& segment,
    const std::vector<TransformedSegment>& infill_lines_below,
    Polygon& expanded_polygon,
    const TransformedSegment*& current_supporting_infill_line)
{
    if (fuzzy_equal(segment.min_y, segment.max_y))
    {
        // Skip horizontal segments, holes will be filled by expanding their previous and next segments
        return;
    }

    // 1 means expand to the right, -1 expand to the left
    const int8_t expand_direction = sign(segment.end.Y - segment.start.Y);

    std::vector<ExpansionRange> expanded_ranges;
    expanded_ranges.push_back(ExpansionRange(segment.min_y, segment.max_y, &segment));

    for (const TransformedSegment& infill_line_below : infill_lines_below)
    {
        const std::optional<SegmentOverlapping> overlapping = segment.calculateOverlapping(infill_line_below, expand_direction);
        if (! overlapping.has_value())
        {
            continue;
        }

        const TransformedSegment& line_part = overlapping->other_overlapping_part;

        std::vector<ExpansionRange> new_expanded_ranges;
        std::optional<ExpansionRange> replacing_range;

        const auto commit_replacing_range = [&replacing_range, &new_expanded_ranges]()
        {
            if (replacing_range.has_value())
            {
                if (replacing_range->isValid())
                {
                    new_expanded_ranges.push_back(*replacing_range);
                }
                replacing_range.reset();
            }
        };

        for (const ExpansionRange& expanded_range : expanded_ranges)
        {
            const std::optional<SegmentOverlapping> range_overlapping = expanded_range.calculateOverlapping(line_part, -expand_direction);
            if (! range_overlapping.has_value())
            {
                commit_replacing_range();
                new_expanded_ranges.push_back(std::move(expanded_range));
                continue;
            }

            if (range_overlapping->type != SegmentOverlappingType::Bottom && range_overlapping->type != SegmentOverlappingType::Full)
            {
                commit_replacing_range();
                ExpansionRange cropped_range_bottom = std::move(expanded_range);
                cropped_range_bottom.cropTop(range_overlapping->other_overlapping_part.min_y);
                if (cropped_range_bottom.isValid())
                {
                    new_expanded_ranges.push_back(std::move(cropped_range_bottom));
                }
            }

            if (! replacing_range.has_value())
            {
                replacing_range = ExpansionRange(range_overlapping->other_overlapping_part, &infill_line_below);
            }
            else
            {
                replacing_range->data.segment.setEnd(range_overlapping->other_overlapping_part.end);
            }

            if (range_overlapping->type != SegmentOverlappingType::Top && range_overlapping->type != SegmentOverlappingType::Full)
            {
                commit_replacing_range();

                ExpansionRange cropped_range_top = std::move(expanded_range);
                cropped_range_top.cropBottom(range_overlapping->other_overlapping_part.max_y);
                if (cropped_range_top.isValid())
                {
                    new_expanded_ranges.push_back(std::move(cropped_range_top));
                }
            }
        }

        commit_replacing_range();

        if (! new_expanded_ranges.empty())
        {
            expanded_ranges = std::move(new_expanded_ranges);
        }
    }

    if (expand_direction < 0)
    {
        ranges::reverse(expanded_ranges);
    }

    constexpr bool only_if_forming_segment = true;
    for (ExpansionRange& expanded_range : expanded_ranges)
    {
        if (expanded_range.supporting_segment == current_supporting_infill_line)
        {
            continue;
        }

        if (current_supporting_infill_line == nullptr)
        {
            // This is the first iteration for this polygon, add the initial point
            if (! expanded_range.is_set)
            {
                // This is an unitialized range, use the raw segment
                expanded_polygon.push_back(segment.start);
            }
            else
            {
                expanded_polygon.push_back(expand_direction > 0 ? expanded_range.data.segment.start : expanded_range.data.segment.end, only_if_forming_segment);
            }
        }
        else
        {
            Point2LL next_start_position;

            if (! expanded_range.is_set)
            {
                // This is an unitialized range, use the raw segment
                const coord_t position_switch_y = expand_direction > 0 ? expanded_range.data.range.min_y : expanded_range.data.range.max_y;
                next_start_position = Point2LL(LinearAlg2D::lineHorizontalLineIntersection(segment.start, segment.end, position_switch_y).value_or(0), position_switch_y);
            }
            else
            {
                next_start_position = expand_direction > 0 ? expanded_range.data.segment.start : expanded_range.data.segment.end;
            }

            // Add point to close anchoring to previous infill line
            expanded_polygon.push_back(
                Point2LL(
                    LinearAlg2D::lineHorizontalLineIntersection(current_supporting_infill_line->start, current_supporting_infill_line->end, next_start_position.Y).value_or(0),
                    next_start_position.Y),
                only_if_forming_segment);

            // Add point to start anchoring on new infill line
            expanded_polygon.push_back(next_start_position);
        }

        current_supporting_infill_line = expanded_range.supporting_segment;
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

    // Calculate the proper bridging angle, according to the type of infill below
    const AngleDegrees bridge_angle = bridgeOverInfillAngle(mesh, layer_nr);

    // We will expand the bridging area following the lines direction
    const AngleDegrees expansion_angle = bridge_angle + 90;
    const MixedLinesSet& infill_lines_below = completed_layer_plan_below->getGeneratedInfillLines();
    TransformedShape filtered_infill_lines_below;

    // Rotate all the infill lines below so that they are in a space where the under skin area should be expanded horizontally
    const PointMatrix matrix(expansion_angle);
    const auto add_infill_line = [&matrix, &filtered_infill_lines_below](const Point2LL& start, const Point2LL& end) -> void
    {
        TransformedSegment transformed_segment(start, end, matrix);
        if (fuzzy_not_equal(transformed_segment.min_y, transformed_segment.max_y)) // Do not include horizontal segments
        {
            filtered_infill_lines_below.addSegment(std::move(transformed_segment));
        }
    };

    for (const PolylinePtr& infill_line_below : infill_lines_below)
    {
        for (auto iterator = infill_line_below->cbeginSegments(); iterator != infill_line_below->cendSegments(); ++iterator)
        {
            add_infill_line((*iterator).start, (*iterator).end);
        }
    }

    for (const Polygon& polygon : infill_contour)
    {
        for (auto iterator = polygon.beginSegments(); iterator != polygon.endSegments(); ++iterator)
        {
            add_infill_line((*iterator).start, (*iterator).end);
        }
    }

    // Now expand each polygon by projecting its segments horizontally according to the supporting infill line
    Shape transformed_expanded_infill_below_skin_area;
    for (const Polygon& infill_below_skin_polygon : infill_below_skin_area)
    {
        TransformedShape transformed_infill_below_skin_polygon;
        transformPolygon(infill_below_skin_polygon, matrix, transformed_infill_below_skin_polygon);

        Polygon expanded_polygon;
        const TransformedSegment* current_supporting_infill_line = nullptr;
        for (const TransformedSegment& transformed_infill_below_skin_segment : transformed_infill_below_skin_polygon.segments)
        {
            expandSegment(transformed_infill_below_skin_segment, filtered_infill_lines_below.segments, expanded_polygon, current_supporting_infill_line);
        }
        transformed_expanded_infill_below_skin_area.push_back(std::move(expanded_polygon));
    }

    // Move output polygons back to original transform
    transformed_expanded_infill_below_skin_area.applyMatrix(matrix.inverse());

    // Perform a morphological closing to remove overlapping lines
    transformed_expanded_infill_below_skin_area = transformed_expanded_infill_below_skin_area.offset(EPSILON).offset(-EPSILON).intersection(infill_contour);

    return { transformed_expanded_infill_below_skin_area, bridge_angle };
}

} // namespace cura
