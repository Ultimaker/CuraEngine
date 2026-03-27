// Copyright (c) 2026 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "InfillOrderOptimizer.h"

#include <range/v3/algorithm/all_of.hpp>
#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/algorithm/sort.hpp>

#include "LayerPlan.h"
#include "geometry/OpenLinesSet.h"


namespace cura
{

InfillOrderOptimizer::InfillOrderOptimizer()
{
}

void InfillOrderOptimizer::addPart(InfillPartArea part_area, OpenLinesSet& lines)
{
    if (! lines.empty())
    {
        infill_parts_.push_back({ part_area, InfillPartType::Lines, { .lines = &lines } });
    }
}

void InfillOrderOptimizer::addPart(InfillPartArea part_area, const Shape& polygons)
{
    if (! polygons.empty())
    {
        infill_parts_.push_back({ part_area, InfillPartType::Polygons, { .polygons = &polygons } });
    }
}

void InfillOrderOptimizer::addPart(InfillPartArea part_area, const std::vector<std::vector<VariableWidthLines>>& walls)
{
    const bool has_walls = ranges::any_of(
        walls,
        [](const std::vector<VariableWidthLines>& tp)
        {
            return ! (
                tp.empty()
                || ranges::all_of(
                    tp,
                    [](const VariableWidthLines& vwl)
                    {
                        return vwl.empty();
                    }));
        });
    if (has_walls)
    {
        infill_parts_.push_back({ part_area, InfillPartType::ExtrusionLines, { .extrusion_lines = &walls } });
    }
}

void InfillOrderOptimizer::optimize(const bool skin_support_interlace_lines, const std::optional<Point2LL>& near_end_location)
{
    if (infill_parts_.empty())
    {
        return;
    }

    // First order the parts in preferred order
    ranges::sort(infill_parts_, &InfillOrderOptimizer::shouldPrintBefore);

    if (! near_end_location.has_value())
    {
        // We can't really optimize the infill ordering, print it as is
        return;
    }

    // Find the infill part that should be printed at last
    auto closest_infill_part_iterator = infill_parts_.end();
    std::optional<std::pair<size_t, size_t>> closest_line_point;
    coord_t closest_distance_squared;
    const auto walls_iterator = ranges::find_if(
        infill_parts_,
        [](const InfillPart& infill_part)
        {
            return infill_part.area == InfillPartArea::Infill && infill_part.type == InfillPartType::ExtrusionLines;
        });
    if (walls_iterator != infill_parts_.end())
    {
        // If we have walls, always print them at the end
        closest_infill_part_iterator = walls_iterator;
    }
    else if (infill_parts_.size() > 1)
    {
        for (auto iterator = infill_parts_.begin(); iterator != infill_parts_.end(); ++iterator)
        {
            InfillPart& infill_part = *iterator;
            switch (infill_part.type)
            {
            case InfillPartType::Lines:
                if (skin_support_interlace_lines && infill_part.area == InfillPartArea::SkinSupport)
                {
                    // Since we cannot order the skin support lines, don't account them for the global plan
                    break;
                }

                if (isCloserTo(*infill_part.paths.lines, *near_end_location, closest_line_point, closest_distance_squared))
                {
                    closest_infill_part_iterator = iterator;
                }
                break;
            case InfillPartType::Polygons:
                if (isCloserTo(*infill_part.paths.polygons, *near_end_location, closest_line_point, closest_distance_squared))
                {
                    closest_infill_part_iterator = iterator;
                }
                break;
            case InfillPartType::ExtrusionLines:
                assert(false && "We should never have extrusion lines at this points because having walls is filtered out above");
                break;
            }
        }
    }

    if (closest_infill_part_iterator != infill_parts_.end() && closest_infill_part_iterator != std::prev(infill_parts_.end()))
    {
        // Move the part that is closest at the end of the list, if not already there
        InfillPart closest_infill_part = *closest_infill_part_iterator;
        infill_parts_.erase(closest_infill_part_iterator);
        infill_parts_.push_back(closest_infill_part);
    }

    // If needed and possible, split the last part so that we provide an actual position to end close to the seam
    InfillPart& last_part = infill_parts_.back();
    if (last_part.type == InfillPartType::Lines && last_part.area == InfillPartArea::Infill)
    {
        if (! closest_line_point.has_value())
        {
            // Closest point was not previously calculated, maybe because there is only one element in the list, so do it now
            isCloserTo(*last_part.paths.lines, *near_end_location, closest_line_point, closest_distance_squared);
        }

        if (closest_line_point.has_value())
        {
            last_part.paths.lines->split(closest_line_point->first, closest_line_point->second);
        }
    }
    // Other parts don't require splitting: either polygons or skin support, which is only separate segments already
}

bool InfillOrderOptimizer::addToLayer(
    LayerPlan& layer_plan,
    const Settings& settings,
    const std::optional<Point2LL>& near_end_location,
    const EFillMethod infill_pattern,
    const MeshPathConfigs& mesh_config,
    const SliceDataStorage& storage,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const coord_t start_move_inwards_length,
    const coord_t end_move_inwards_length,
    const Shape& infill_inner_contour,
    const coord_t skin_support_line_distance,
    const Shape& infill_below_skin,
    const AngleDegrees& skin_support_angle) const
{
    if (infill_parts_.empty())
    {
        return false;
    }

    layer_plan.setIsInside(true); // going to print stuff inside print object
    std::optional<Point2LL> near_start_location;
    if (settings.get<InfillStartEndPreference>("infill_start_end_preference") == InfillStartEndPreference::START_RANDOM)
    {
        srand(layer_plan.getLayerNr());
        const InfillPart& first_part = infill_parts_.front();
        if (first_part.area == InfillPartArea::Infill)
        {
            switch (first_part.type)
            {
            case InfillPartType::Lines:
                near_start_location = first_part.paths.lines->at(rand() % first_part.paths.lines->size())[0];
                break;
            case InfillPartType::Polygons:
            {
                const Polygon& start_poly = first_part.paths.polygons->at(rand() % first_part.paths.polygons->size());
                near_start_location = start_poly[rand() % start_poly.size()];
                break;
            }
            case InfillPartType::ExtrusionLines:
            {
                const std::vector<VariableWidthLines>* start_paths;
                do
                {
                    start_paths = &first_part.paths.extrusion_lines->at(rand() % first_part.paths.extrusion_lines->size());
                } while (start_paths->empty() || start_paths->front().empty()); // We know for sure that one of them is not empty. So randomise until we hit
                                                                                // it. Should almost always be very quick.
                near_start_location = (*start_paths)[0][0].junctions_[0].p_;
                break;
            }
            }
        }
    }

    MixedLinesSet all_infill_lines;

    for (const auto& [part_index, part] : infill_parts_ | ranges::views::enumerate)
    {
        const bool is_first = part_index == 0;
        const bool is_last = part_index == infill_parts_.size() - 1;

        bool reverse_print_direction = false;
        std::optional<Point2LL> near_start_location_here;
        if (is_last && near_end_location.has_value())
        {
            near_start_location_here = near_end_location;
            reverse_print_direction = true;
        }
        else if (is_first)
        {
            near_start_location_here = near_start_location;
        }

        addToLayer(
            part,
            layer_plan,
            settings,
            near_start_location_here,
            reverse_print_direction,
            infill_pattern,
            mesh_config,
            storage,
            mesh,
            extruder_nr,
            start_move_inwards_length,
            end_move_inwards_length,
            infill_inner_contour,
            skin_support_line_distance,
            infill_below_skin,
            skin_support_angle);

        if (part.area == InfillPartArea::Infill)
        {
            switch (part.type)
            {
            case InfillPartType::Lines:
                all_infill_lines.push_back(*part.paths.lines);
                break;
            case InfillPartType::Polygons:
                all_infill_lines.push_back(*part.paths.polygons);
                break;
            case InfillPartType::ExtrusionLines:
                break;
            }
        }
    }

    layer_plan.setGeneratedInfillLines(&mesh, all_infill_lines);

    return true;
}

void InfillOrderOptimizer::addToLayer(
    const InfillPart& part,
    LayerPlan& layer_plan,
    const Settings& settings,
    const std::optional<Point2LL>& near_start_location,
    const bool reverse_print_direction,
    const EFillMethod infill_pattern,
    const MeshPathConfigs& mesh_config,
    const SliceDataStorage& storage,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const coord_t start_move_inwards_length,
    const coord_t end_move_inwards_length,
    const Shape& infill_inner_contour,
    const coord_t skin_support_line_distance,
    const Shape& infill_below_skin,
    const AngleDegrees& skin_support_angle) const
{
    const bool enable_travel_optimization = settings.get<bool>("infill_enable_travel_optimization");
    constexpr Ratio flow_ratio = 1.0_r;
    constexpr double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;
    const std::unordered_multimap<const Polyline*, const Polyline*> order_requirements = PathOrderOptimizer<const Polyline*>::no_order_requirements_;

    switch (part.area)
    {
    case InfillPartArea::Infill:
        switch (part.type)
        {
        case InfillPartType::Lines:
        {
            addInfillLinesToLayer(
                *part.paths.lines,
                layer_plan,
                settings,
                near_start_location,
                reverse_print_direction,
                infill_pattern,
                mesh_config,
                start_move_inwards_length,
                end_move_inwards_length,
                infill_inner_contour,
                enable_travel_optimization,
                flow_ratio,
                fan_speed,
                order_requirements);
            break;
        }

        case InfillPartType::Polygons:
        {
            OpenLinesSet remaining_lines;
            layer_plan.addInfillPolygonsByOptimizer(
                *part.paths.polygons,
                remaining_lines,
                mesh_config.infill_config[0],
                settings,
                start_move_inwards_length > 0 || end_move_inwards_length > 0,
                near_start_location);
            if (! remaining_lines.empty())
            {
                addInfillLinesToLayer(
                    remaining_lines,
                    layer_plan,
                    settings,
                    near_start_location,
                    reverse_print_direction,
                    infill_pattern,
                    mesh_config,
                    start_move_inwards_length,
                    end_move_inwards_length,
                    infill_inner_contour,
                    enable_travel_optimization,
                    flow_ratio,
                    fan_speed,
                    order_requirements);
            }
            break;
        }

        case InfillPartType::ExtrusionLines:
        {
            for (const std::vector<VariableWidthLines>& tool_paths : *part.paths.extrusion_lines)
            {
                constexpr bool retract_before_outer_wall = false;
                constexpr coord_t wipe_dist = 0;
                const ZSeamConfig z_seam_config(EZSeamType::USER_SPECIFIED, near_start_location.value_or(mesh.getZSeamHint()));
                InsetOrderOptimizer wall_orderer(
                    storage,
                    layer_plan,
                    settings,
                    extruder_nr,
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    retract_before_outer_wall,
                    wipe_dist,
                    wipe_dist,
                    extruder_nr,
                    extruder_nr,
                    z_seam_config,
                    tool_paths,
                    mesh.bounding_box.flatten().getMiddle());
                wall_orderer.addToLayer();
            }
            break;
        }
        }
        break;

    case InfillPartArea::SkinSupport:
    {
        switch (part.type)
        {
        case InfillPartType::Lines:
        {
            addSkinSupportLinesToLayer(
                *part.paths.lines,
                layer_plan,
                settings,
                near_start_location,
                reverse_print_direction,
                mesh_config,
                skin_support_line_distance,
                infill_below_skin,
                skin_support_angle,
                enable_travel_optimization,
                flow_ratio);
            break;
        }

        case InfillPartType::Polygons:
        {
            OpenLinesSet remaining_lines;
            constexpr bool extra_inwards_move = false;
            layer_plan.addInfillPolygonsByOptimizer(*part.paths.polygons, remaining_lines, mesh_config.skin_support_config, settings, extra_inwards_move, near_start_location);
            if (! remaining_lines.empty())
            {
                addSkinSupportLinesToLayer(
                    remaining_lines,
                    layer_plan,
                    settings,
                    near_start_location,
                    reverse_print_direction,
                    mesh_config,
                    skin_support_line_distance,
                    infill_below_skin,
                    skin_support_angle,
                    enable_travel_optimization,
                    flow_ratio);
            }

            break;
        }
        }

        break;
    }
    }
}

void InfillOrderOptimizer::addInfillLinesToLayer(
    const OpenLinesSet& lines,
    LayerPlan& layer_plan,
    const Settings& settings,
    const std::optional<Point2LL>& near_start_location,
    const bool reverse_print_direction,
    const EFillMethod infill_pattern,
    const MeshPathConfigs& mesh_config,
    const coord_t start_move_inwards_length,
    const coord_t end_move_inwards_length,
    const Shape& infill_inner_contour,
    const bool enable_travel_optimization,
    const Ratio& flow_ratio,
    const double fan_speed,
    const std::unordered_multimap<const Polyline*, const Polyline*>& order_requirements) const
{
    SpaceFillType space_fill_type;
    coord_t wipe_dist;
    if (infill_pattern == EFillMethod::GRID || infill_pattern == EFillMethod::LINES || infill_pattern == EFillMethod::TRIANGLES || infill_pattern == EFillMethod::CUBIC
        || infill_pattern == EFillMethod::TETRAHEDRAL || infill_pattern == EFillMethod::QUARTER_CUBIC || infill_pattern == EFillMethod::CUBICSUBDIV
        || infill_pattern == EFillMethod::LIGHTNING)
    {
        space_fill_type = SpaceFillType::Lines;
        wipe_dist = settings.get<coord_t>("infill_wipe_dist");
    }
    else
    {
        space_fill_type = (infill_pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines;
        wipe_dist = 0;
    }

    layer_plan.addLinesByOptimizer(
        lines,
        mesh_config.infill_config[0],
        space_fill_type,
        enable_travel_optimization,
        wipe_dist,
        flow_ratio,
        near_start_location,
        fan_speed,
        reverse_print_direction,
        order_requirements,
        start_move_inwards_length,
        end_move_inwards_length,
        MendedShape(&settings, SectionType::INFILL, &infill_inner_contour));
}

void InfillOrderOptimizer::addSkinSupportLinesToLayer(
    const OpenLinesSet& lines,
    LayerPlan& layer_plan,
    const Settings& settings,
    const std::optional<Point2LL>& near_start_location,
    const bool reverse_print_direction,
    const MeshPathConfigs& mesh_config,
    const coord_t skin_support_line_distance,
    const Shape& infill_below_skin,
    const AngleDegrees& skin_support_angle,
    const bool enable_travel_optimization,
    const Ratio& flow_ratio) const
{
    const auto skin_support_fan_speed = settings.get<double>("skin_support_fan_speed");
    constexpr SpaceFillType skin_support_space_fill_type = SpaceFillType::Lines;
    constexpr coord_t skin_support_wipe_dist = 0;
    const auto skin_support_interlace_lines = settings.get<bool>("skin_support_interlace_lines");
    if (skin_support_interlace_lines)
    {
        const coord_t max_adjacent_distance = skin_support_line_distance * 1.1;
        constexpr coord_t exclude_distance = 0;
        constexpr bool skin_support_interlaced = true;
        layer_plan.addLinesMonotonic(
            infill_below_skin,
            lines,
            mesh_config.skin_support_config,
            skin_support_space_fill_type,
            skin_support_angle,
            max_adjacent_distance,
            exclude_distance,
            skin_support_wipe_dist,
            flow_ratio,
            skin_support_fan_speed,
            skin_support_interlaced);
    }
    else
    {
        layer_plan.addLinesByOptimizer(
            lines,
            mesh_config.skin_support_config,
            skin_support_space_fill_type,
            enable_travel_optimization,
            skin_support_wipe_dist,
            flow_ratio,
            near_start_location,
            skin_support_fan_speed,
            reverse_print_direction);
    }
}

bool InfillOrderOptimizer::shouldPrintBefore(const InfillPart& part1, const InfillPart& part2)
{
    if (part1.area == part2.area)
    {
        return part1.type < part2.type;
    }
    else
    {
        return part1.area < part2.area;
    }
}

template<class LinesSetType>
bool InfillOrderOptimizer::isCloserTo(
    const LinesSetType& line_set,
    const Point2LL& location,
    std::optional<std::pair<size_t, size_t>>& closest_point,
    coord_t& closest_distance_squared)
{
    bool closer_found = false;
    for (const auto& [line_index, line] : line_set | ranges::views::enumerate)
    {
        for (const auto& [point_index, point] : line | ranges::views::enumerate)
        {
            const coord_t distance_squared = vSize2(point - location);
            if (! closest_point.has_value() || distance_squared < closest_distance_squared)
            {
                closer_found = true;
                closest_point = std::make_optional(std::make_pair(line_index, point_index));
                closest_distance_squared = distance_squared;
            }
        }
    }

    return closer_found;
}

} // namespace cura
