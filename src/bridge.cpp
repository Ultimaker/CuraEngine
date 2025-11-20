// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge.h"

#include <unordered_set>

#include <range/v3/action/stable_sort.hpp>
#include <range/v3/view/cartesian_product.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Point2D.h"
#include "geometry/Polygon.h"
#include "geometry/conversions/Point2D_Point2LL.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "utils/AABB.h"
#include "utils/SVG.h"
#include "utils/math.h"

namespace cura
{

size_t findFitAnchoredLines(const Shape& skin_outline, const Shape& supported_regions, const coord_t line_width, const AngleRadians& angle, const SVG* svg)
{
    // Project the anchor regions along the angle direction to get the bounding line
    const Point2D direction(std::cos(angle), std::sin(angle));
    const Point2D left_direction = direction.rotated90CCW();
    double min_axial = std::numeric_limits<double>::max();
    double max_axial = std::numeric_limits<double>::lowest();
    coord_t min_radial = std::numeric_limits<coord_t>::max();
    coord_t max_radial = std::numeric_limits<coord_t>::lowest();
    for (const Polygon& polygon : supported_regions)
    {
        for (const Point2LL& point : polygon)
        {
            const double axial_projection = Point2D::dot(direction, toPoint2D(point));
            min_axial = std::min(min_axial, axial_projection);
            max_axial = std::max(max_axial, axial_projection);

            const Point2LL projected_point = toPoint2LL(direction * axial_projection);
            const Point2LL projected_to_point = point - projected_point;
            const coord_t distance_to_projected_point = vSize(projected_to_point);
            const coord_t sign = Point2D::dot(left_direction, toPoint2D(projected_to_point)) > 0 ? 1 : -1;
            const coord_t radial_projection = sign * distance_to_projected_point;

            max_radial = std::max(max_radial, radial_projection);
            min_radial = std::min(min_radial, radial_projection);
        }
    }

    // Create a set of lines that could be properly bridging
    const double delta_axial = max_axial - min_axial;
    if (delta_axial < line_width)
    {
        return {};
    }

    const Point2LL delta_axial_min = toPoint2LL(left_direction * min_radial);
    const Point2LL delta_axial_max = toPoint2LL(left_direction * max_radial);
    OpenLinesSet bridge_lines;
    for (double axial = min_axial; axial < max_axial; axial += line_width)
    {
        const Point2LL point_on_axis = toPoint2LL(direction * axial);
        bridge_lines.push_back(OpenPolyline({ point_on_axis + delta_axial_min, point_on_axis + delta_axial_max }));
    }

    constexpr bool restitch = false;
    bridge_lines = skin_outline.intersection(bridge_lines, restitch);
    coord_t total_supported_length = 0;

    const coord_t max_hanging_distance_squared = square(line_width);
    const auto check_tip_supported
        = [&supported_regions,
           &max_hanging_distance_squared](const double intersection_start, const double intersection_end, const Point2LL& segment_start, const Point2LL& segment) -> bool
    {
        const Point2LL tip_start = segment_start + intersection_start * segment;
        const Point2LL tip_end = segment_start + intersection_end * segment;

        constexpr bool border_result = true;
        if (supported_regions.inside(tip_end, border_result))
        {
            return true;
        }

        return vSize2(tip_end - tip_start) <= max_hanging_distance_squared;
    };

    // For each line, evaluate if it is a proper bridging line, i.e. it doesn't have hanging tips longer that line_width
    for (const OpenPolyline& bridge_line : bridge_lines)
    {
        assert(bridge_line.size() == 2);

        const Point2LL& segment_start = bridge_line[0];
        const Point2LL& segment_end = bridge_line[1];
        const Point2LL segment = segment_end - segment_start;

        std::vector<float> intersections = supported_regions.intersectionsWithSegment(segment_start, segment_end);
        ranges::actions::stable_sort(intersections);

        if (intersections.empty() || ! is_null(intersections.front()))
        {
            intersections.insert(intersections.begin(), 0.0);
        }
        if (! is_null(1.0 - intersections.back()))
        {
            intersections.push_back(1.0);
        }

        const bool start_tip_supported = check_tip_supported(intersections[1], intersections[0], segment_start, segment);
        const bool end_tip_supported
            = intersections.size() > 2 ? check_tip_supported(intersections[intersections.size() - 2], intersections.back(), segment_start, segment) : start_tip_supported;

        if (start_tip_supported && end_tip_supported)
        {
            total_supported_length += vSize(segment);
            if (svg)
            {
                svg->write(bridge_line, { .line = { SVG::Color::RED, 0.6 } });
            }
        }
    }

    return total_supported_length;
    // return score;
}

double bridgeAngle(
    const Settings& settings,
    const Shape& skin_outline,
    const SliceDataStorage& storage,
    const unsigned layer_nr,
    const unsigned bridge_layer,
    const SupportLayer* support_layer,
    Shape& supported_regions)
{
    assert(! skin_outline.empty());
    AABB boundary_box(skin_outline);

    spdlog::stopwatch timer;

    const coord_t line_width = settings.get<coord_t>("skin_line_width");
    // SVG svg(fmt::format("/tmp/bridge_regions_{}.svg", layer_nr), boundary_box);
    // svg.write(skin_outline, { .surface = { SVG::Color::GRAY } });

    // To detect if we have a bridge, first calculate the intersection of the current layer with the previous layer.
    //  This gives us the islands that the layer rests on.
    Shape islands;

    Shape prev_layer_outline; // we also want the complete outline of the previous layer

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
                Shape solid_below(prev_layer_part.outline);
                if (bridge_layer == 1 && part_has_sparse_infill)
                {
                    solid_below = solid_below.difference(prev_layer_part.getOwnInfillArea());
                }
                prev_layer_outline.push_back(solid_below); // not intersected with skin

                if (! boundary_box.hit(prev_layer_part.boundaryBox))
                    continue;

                islands.push_back(skin_outline.intersection(solid_below));
            }
        }
    }
    supported_regions = islands;

    // svg.write(islands, { .surface = { SVG::Color::RED } });
    // svg.write(prev_layer_outline, { .line = { SVG::Color::GREEN, 0.3 } });

    if (support_layer)
    {
        // svg.write(support_layer->support_roof, { .line = { SVG::Color::BLUE, 0.25 } });
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

    // svg.write(supported_regions, { .surface = { SVG::Color::BLUE } });

    // Shape anchor_region = skin_outline.offset(-line_width / 2).intersection(supported_regions);

    // svg.write(anchor_region, { .line = { SVG::Color::GREEN } });
    //
    // std::set<Point2LL> anchor_outer_points;
    // for (const Polygon& anchor_polygon : anchor_region)
    // {
    //     for (const Point2LL& anchor_point : anchor_polygon)
    //     {
    //         anchor_outer_points.emplace(anchor_point);
    //     }
    // }

    struct FitAngle
    {
        coord_t total_supported_length{ 0 };
        AngleDegrees angle{ -1.0 };
    };

    FitAngle best_angle{};
    for (AngleDegrees angle = 0; angle < 180; angle += 1)
    {
        coord_t total_supported_length = findFitAnchoredLines(skin_outline, supported_regions, line_width, angle, nullptr);
        if (total_supported_length > best_angle.total_supported_length)
        {
            best_angle = { total_supported_length, angle };
        }
    }

    // findFitAnchoredLines(skin_outline, anchor_region, supported_regions, line_width, best_angle.angle, &svg);

    spdlog::info("Bridging calculation time {}", timer.elapsed_ms().count());

    return best_angle.angle > -0.5 ? 180 - best_angle.angle.value_ : -1.0;

    const bool bridge_settings_enabled = settings.get<bool>("bridge_settings_enabled");
    const Ratio support_threshold = bridge_settings_enabled ? settings.get<Ratio>("bridge_skin_support_threshold") : 0.0_r;

    // if the proportion of the skin region that is supported is less than supportThreshold, it's considered a bridge and we
    // determine the best angle for the skin lines - the current heuristic is that the skin lines should be parallel to the
    // direction of the skin area's longest unsupported edge - if the skin has no unsupported edges, we fall through to the
    // original code

    if (support_threshold > 0 && (supported_regions.area() / (skin_outline.area() + 1)) < support_threshold)
    {
        Shape bb_poly;
        bb_poly.push_back(boundary_box.toPolygon());

        // airBelow is the region below the skin that is not supported, it extends well past the boundary of the skin.
        // It needs to be shrunk slightly so that the vertices of the skin polygon that would otherwise fall exactly on
        // the air boundary do appear to be supported

        const coord_t bb_max_dim = std::max(boundary_box.max_.X - boundary_box.min_.X, boundary_box.max_.Y - boundary_box.min_.Y);
        const Shape air_below(bb_poly.offset(bb_max_dim).difference(prev_layer_outline).offset(-10));

        // svg.write(air_below, { .surface = { SVG::Color::MAGENTA } });

        OpenLinesSet skin_perimeter_lines;
        for (const Polygon& poly : skin_outline)
        {
            if (! poly.empty())
            {
                skin_perimeter_lines.emplace_back(poly.toPseudoOpenPolyline());
            }
        }

        OpenLinesSet skin_perimeter_lines_over_air(air_below.intersection(skin_perimeter_lines));

        // svg.write(skin_perimeter_lines_over_air, { .line = { SVG::Color::ORANGE, 0.25 } });

        if (skin_perimeter_lines_over_air.size())
        {
            // one or more edges of the skin region are unsupported, determine the longest
            coord_t max_dist2 = 0;
            double line_angle = -1;
            for (const OpenPolyline& air_line : skin_perimeter_lines_over_air)
            {
                for (auto iterator = air_line.beginSegments(); iterator != air_line.endSegments(); ++iterator)
                {
                    const Point2LL vector = (*iterator).start - (*iterator).end;
                    coord_t dist2 = vSize2(vector);
                    if (dist2 > max_dist2)
                    {
                        max_dist2 = dist2;
                        line_angle = angle(vector);
                    }
                }
            }
            return line_angle;
        }
    }
    else
    {
        // as the proportion of the skin region that is supported is >= supportThreshold, it's not
        // considered to be a bridge and the original bridge detection code below is skipped
        return -1.0;
    }

    if (islands.size() > 5 || islands.size() < 1)
    {
        return -1.0;
    }

    // Next find the 2 largest islands that we rest on.
    double area1 = 0;
    double area2 = 0;
    std::optional<size_t> idx1;
    std::optional<size_t> idx2;
    for (size_t n = 0; n < islands.size(); n++)
    {
        // Skip internal holes
        if (! islands[n].orientation())
            continue;
        double area = std::abs(islands[n].area());
        if (area > area1)
        {
            if (area1 > area2)
            {
                area2 = area1;
                idx2 = idx1;
            }
            area1 = area;
            idx1 = n;
        }
        else if (area > area2)
        {
            area2 = area;
            idx2 = n;
        }
    }

    if (! idx1.has_value() || ! idx2.has_value())
        return -1.0;

    Point2LL center1 = islands[idx1.value()].centerOfMass();
    Point2LL center2 = islands[idx2.value()].centerOfMass();

    return angle(center2 - center1);
}

} // namespace cura
