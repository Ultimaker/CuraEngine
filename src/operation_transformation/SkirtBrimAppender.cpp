// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/SkirtBrimAppender.h"

#include <Application.h>
#include <Cura.pb.h>
#include <Slice.h>
#include <geometry/Shape.h>
#include <settings/EnumSettings.h>
#include <settings/PathConfigStorage.h>
#include <settings/Settings.h>
#include <utils/MixedPolylineStitcher.h>
#include <utils/Simplify.h>

#include <range/v3/algorithm/all_of.hpp>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/view/map.hpp>

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/LayerPlanPtr.h"
#include "sliceDataStorage.h"
#include "utils/ExtrudersSet.h"

namespace cura
{

SkirtBrimAppender::SkirtBrimAppender()
    : PrintOperationTransformer()
{
}

void SkirtBrimAppender::process(PrintPlan* print_plan)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;
    const auto adhesion_type = settings.get<EPlatformAdhesion>("adhesion_type");
    const auto support_brim_enable = settings.get<bool>("support_brim_enable") && settings.get<bool>("support_enable");

    if (adhesion_type != EPlatformAdhesion::SKIRT && adhesion_type != EPlatformAdhesion::BRIM && ! support_brim_enable)
    {
        return;
    }

    const auto skirt_brim_extruder_nr_setting = settings.get<int>("skirt_brim_extruder_nr");
    const std::optional<ExtruderNumber> skirt_brim_extruder_nr = skirt_brim_extruder_nr_setting >= 0 ? std::make_optional(skirt_brim_extruder_nr_setting) : std::nullopt;

    // Get the first extruder plan for each extruder
    std::vector<ExtruderNumber> used_extruders = print_plan->calculateUsedExtruders();

    const std::map<ExtruderNumber, ExtruderConfig> extruders_configs = generateExtrudersConfigs(used_extruders, adhesion_type);
    const size_t height = calculateMaxHeight(extruders_configs, adhesion_type);

    std::map<ExtruderNumber, Shape> starting_outlines;
    std::map<ExtruderNumber, Shape> allowed_areas;
    generateBaseAreas(print_plan, skirt_brim_extruder_nr, height, adhesion_type, extruders_configs, used_extruders, starting_outlines, allowed_areas);

    // Specific outlines that are used for upper layers of skirt and generated at first layer, first offset
    std::map<ExtruderNumber, Shape> specific_starting_outlines;

    for (LayerIndex actual_height = 0; actual_height < height; ++actual_height)
    {
        LayerPlanPtr layer_plan = print_plan->findLayerPlan(actual_height);
        if (layer_plan)
        {
            auto first_extruder_outline_action = FirstExtruderOutlineAction::None;
            if (adhesion_type == EPlatformAdhesion::SKIRT)
            {
                first_extruder_outline_action = actual_height == 0 ? FirstExtruderOutlineAction::Save : FirstExtruderOutlineAction::Use;
            }

            generateSkirtBrim(
                adhesion_type,
                skirt_brim_extruder_nr,
                used_extruders,
                starting_outlines,
                specific_starting_outlines,
                allowed_areas,
                extruders_configs,
                layer_plan,
                first_extruder_outline_action);
        }
    }
}

coord_t SkirtBrimAppender::getLineWidth(const ExtruderNumber extruder_nr, const ConstLayerPlanPtr& layer_plan)
{
    return Application::getInstance().current_slice_->scene.getExtruder(extruder_nr).settings_.getLineWidth(layer_plan->getLayerIndex(), "skirt_brim_line_width");
}

size_t SkirtBrimAppender::calculateMaxHeight(const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs, const EPlatformAdhesion adhesion_type)
{
    size_t height = 1;

    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        height = std::max(height, ranges::max(extruders_configs | ranges::views::values | ranges::views::transform(&ExtruderConfig::skirt_height_)));
    }

    return height;
}

std::map<ExtruderNumber, SkirtBrimAppender::ExtruderConfig>
    SkirtBrimAppender::generateExtrudersConfigs(std::vector<ExtruderNumber>& used_extruders, const EPlatformAdhesion adhesion_type)
{
    std::map<ExtruderNumber, ExtruderConfig> extruders_configs;

    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        ExtruderConfig& extruder_config = extruders_configs[extruder_nr];
        const Settings& settings = Application::getInstance().current_slice_->scene.getExtruder(extruder_nr).settings_;

        const auto& location = settings.get<BrimLocation>("brim_location");
        // FIXME: Handle the initial-layer speed and flow
        extruder_config.line_width_X_ = settings.get<coord_t>("skirt_brim_line_width");
        extruder_config.line_width_0_ = extruder_config.line_width_X_ * settings.get<Ratio>("initial_layer_line_width_factor");
        extruder_config.skirt_brim_minimal_length_ = settings.get<coord_t>("skirt_brim_minimal_length");
        extruder_config.outside_polys_ = adhesion_type == EPlatformAdhesion::SKIRT || (location & BrimLocation::OUTSIDE);
        extruder_config.inside_polys_ = adhesion_type == EPlatformAdhesion::BRIM && (location & BrimLocation::INSIDE);
        extruder_config.line_count_ = settings.get<size_t>(adhesion_type == EPlatformAdhesion::BRIM ? "brim_line_count" : "skirt_line_count");
        extruder_config.gap_ = settings.get<coord_t>(adhesion_type == EPlatformAdhesion::BRIM ? "brim_gap" : "skirt_gap");
        extruder_config.brim_inside_margin_ = settings.get<coord_t>("brim_inside_margin");
        extruder_config.skirt_height_ = settings.get<size_t>("skirt_height");
    }

    return extruders_configs;
}

void SkirtBrimAppender::generateBaseAreas(
    const PrintPlan* print_plan,
    const std::optional<ExtruderNumber> brim_extruder_nr,
    const size_t height,
    const EPlatformAdhesion adhesion_type,
    const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
    const std::vector<ExtruderNumber>& used_extruders,
    std::map<ExtruderNumber, Shape>& starting_outlines,
    std::map<ExtruderNumber, Shape>& allowed_areas) const
{
    // Geather footprints of printed features for each extruder
    const std::map<ExtruderNumber, FeatureFootprint> features_footprints = print_plan->calculateFootprint(std::nullopt, height + 1);

    // Now generate the starting outlines for each extruder based on the footprints
    // Target extruders are the ones for which we want to generate starting outlines
    const std::vector<ExtruderNumber> brim_extruders = brim_extruder_nr.has_value() ? std::vector({ brim_extruder_nr.value() }) : used_extruders;
    const std::vector<ExtruderNumber> target_extruders = adhesion_type == EPlatformAdhesion::SKIRT ? std::vector<ExtruderNumber>({ 0 }) : brim_extruders;
    for (const ExtruderNumber target_extruder : target_extruders)
    {
        const ExtruderConfig& target_extruder_config = extruders_configs.at(target_extruder);
        std::vector<Shape> expanded_features_footprints;

        // Source extruders are the ones for which we want to take the footprints into account
        const bool merge_all_extruders = (adhesion_type == EPlatformAdhesion::SKIRT) || brim_extruder_nr.has_value();
        const std::vector<ExtruderNumber> source_extruders = merge_all_extruders ? used_extruders : std::vector({ target_extruder });

        for (const ExtruderNumber source_extruder : source_extruders)
        {
            const auto iterator = features_footprints.find(source_extruder);
            if (iterator != features_footprints.end())
            {
                for (const auto& [feature_type, footprint] : iterator->second)
                {
                    Shape expanded_footprint;
                    if (PrintFeatureTypes::isModel(feature_type))
                    {
                        expanded_footprint = footprint.offset(extruders_configs.at(source_extruder).gap_, ClipperLib::jtRound);
                    }
                    else
                    {
                        expanded_footprint = footprint;
                    }

                    expanded_features_footprints.push_back(std::move(expanded_footprint));
                }
            }
        }

        Shape& extruder_starting_outlines = starting_outlines[target_extruder];
        const Shape united_footprints = Shape::unionShapes(expanded_features_footprints);
        const coord_t max_line_width = std::max(target_extruder_config.line_width_0_, target_extruder_config.line_width_X_);

        for (const Polygon& polygon : united_footprints)
        {
            const bool is_hole = polygon.isHole();
            const bool add_polygon = is_hole ? target_extruder_config.inside_polys_ : target_extruder_config.outside_polys_;
            if (add_polygon)
            {
                // Make an outline of the polygon as if it was a previous offset
                if (is_hole)
                {
                    extruder_starting_outlines.push_back(Shape(polygon).unionPolygons(polygon.offset(max_line_width)));
                }
                else
                {
                    extruder_starting_outlines.push_back(Shape(polygon).difference(polygon.offset(-max_line_width)));
                }
            }
        }

        if (adhesion_type == EPlatformAdhesion::SKIRT)
        {
            extruder_starting_outlines = extruder_starting_outlines.approxConvexHull();
        }
    }

    // Initialize each extruder allowed area with the whole build plate
    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        allowed_areas[extruder_nr] = SliceDataStorage::getCurrent()->getMachineBorder(extruder_nr);
    }

    // Don't allow offsets to grow over themselves
    for (const auto& [extruder_nr, extruder_starting_outlines] : starting_outlines)
    {
        Shape& extruder_allowed_areas = allowed_areas[extruder_nr];
        extruder_allowed_areas = extruder_allowed_areas.difference(extruder_starting_outlines);
    }

    // Don't allow brim offsets to grow too close to already printed areas
    for (const auto& [source_extruder, extruder_features_footprints] : features_footprints)
    {
        const ExtruderConfig& source_extruder_config = extruders_configs.at(source_extruder);

        for (const auto& [feature_type, footprint] : extruder_features_footprints)
        {
            Shape expanded_footprint_self;
            Shape expanded_footprint_other;
            if (PrintFeatureTypes::isModel(feature_type))
            {
                expanded_footprint_self = footprint.offset(source_extruder_config.gap_);
                if (adhesion_type == EPlatformAdhesion::BRIM && ! brim_extruder_nr.has_value() && used_extruders.size() > 1)
                {
                    expanded_footprint_other = footprint.offset(source_extruder_config.brim_inside_margin_);
                }
                else
                {
                    expanded_footprint_other = expanded_footprint_self;
                }
            }
            else
            {
                expanded_footprint_self = expanded_footprint_other = footprint;
            }

            for (auto& [target_extruder, extruder_allowed_areas] : allowed_areas)
            {
                if (target_extruder == source_extruder)
                {
                    extruder_allowed_areas = extruder_allowed_areas.difference(expanded_footprint_self);
                }
                else
                {
                    extruder_allowed_areas = extruder_allowed_areas.difference(expanded_footprint_other);
                }
            }
        }
    }
}

std::vector<ContinuousExtruderMoveSequencePtr> SkirtBrimAppender::generateOffset(
    const ExtruderNumber extruder_nr,
    const coord_t total_offset,
    const Shape& outline,
    Shape& covered_area,
    std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
    const LayerPlanPtr& layer_plan,
    const bool update_allowed_areas)
{
    const coord_t line_width = getLineWidth(extruder_nr, layer_plan);

    Shape brim = outline.offset(total_offset, ClipperLib::jtRound);

    // limit brim lines to allowed areas, stitch them and store them in the result
    brim = Simplify(Application::getInstance().current_slice_->scene.extruders_[extruder_nr].settings_).polygon(brim);

    const Shape allowed_area = allowed_areas_per_extruder[extruder_nr];

    const OpenLinesSet brim_lines = allowed_area.intersection(brim, false);

    const coord_t max_stitch_distance = line_width;
    MixedLinesSet result;
    MixedPolylineStitcher::stitch(brim_lines, result, max_stitch_distance);

    const GCodePathConfig& config = layer_plan->getConfigsStorage()->skirt_brim_config_per_extruder.at(extruder_nr);

    std::vector<ContinuousExtruderMoveSequencePtr> extrusions;
    for (PolylinePtr& extrusion_line : result)
    {
        if (extrusion_line->length() >= min_brim_line_length_)
        {
            extrusions.push_back(ContinuousExtruderMoveSequence::makeFrom(*extrusion_line, line_width, config.getSpeed()));
        }
    }

    if (update_allowed_areas)
    {
        const Shape newly_covered = brim_lines.offset((line_width / 2) + 10, ClipperLib::jtRound);
        covered_area = covered_area.unionPolygons(newly_covered.unionPolygons());
        for (Shape& allowed_area : ranges::views::values(allowed_areas_per_extruder))
        {
            allowed_area = allowed_area.difference(covered_area);
        }
    }

    return extrusions;

    // FIXME: This should be done globally by a "filter" transformer
    // simplify paths to prevent buffer unnerruns in firmware
    // const Settings& global_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    // const coord_t maximum_resolution = global_settings.get<coord_t>("meshfix_maximum_resolution");
    // const coord_t maximum_deviation = global_settings.get<coord_t>("meshfix_maximum_deviation");
    // constexpr coord_t max_area_dev = 0u; // No area deviation applied
    // for (int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    // {
    //     for (MixedLinesSet& lines : storage_.skirt_brim[extruder_nr])
    //     {
    //         lines = Simplify(maximum_resolution, maximum_deviation, max_area_dev).polyline(lines);
    //     }
    // }
}

void SkirtBrimAppender::generateSkirtBrim(
    const EPlatformAdhesion adhesion_type,
    const std::optional<ExtruderNumber> brim_extruder_nr,
    std::vector<ExtruderNumber>& used_extruders,
    const std::map<ExtruderNumber, Shape>& starting_outlines,
    std::map<ExtruderNumber, Shape>& specific_starting_outlines,
    std::map<ExtruderNumber, Shape> allowed_areas_per_extruder,
    const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
    const LayerPlanPtr& layer_plan,
    const FirstExtruderOutlineAction first_extruder_outline_action)
{
    struct ExtruderOffsetData
    {
        ExtruderNumber extruder_nr;
        FeatureExtrusionPtr extrusion;
        coord_t total_offset;
        coord_t extruded_length{ 0 };
        bool done{ false };
        size_t processed_offsets{ 0 };
    };

    // Create a vector containing the processing data for each extruder to be processed,
    // ordered in actual processing order
    std::vector<ExtruderOffsetData> extruder_offset_datas;
    const std::vector<ExtruderNumber> offset_extruders = brim_extruder_nr.has_value() ? std::vector({ brim_extruder_nr.value() }) : used_extruders;
    for (const ExtruderNumber extruder_nr : offset_extruders)
    {
        const ExtruderConfig& extruder_config = extruders_configs.at(extruder_nr);
        if (extruder_config.skirt_height_ > layer_plan->getLayerIndex())
        {
            extruder_offset_datas.push_back(ExtruderOffsetData{ .extruder_nr = extruder_nr,
                                                                .extrusion = std::make_shared<FeatureExtrusion>(PrintFeatureType::SkirtBrim, getLineWidth(extruder_nr, layer_plan)),
                                                                .total_offset = extruder_config.gap_ });
        }
    }

    // Create a cache containing the extruders processing ordering, we are going to need it extensively
    const std::map<ExtruderNumber, size_t> extruder_ordering = [&used_extruders]()
    {
        std::map<ExtruderNumber, size_t> extruder_ordering_tmp;

        for (const auto& [index, extruder_number] : used_extruders | ranges::views::enumerate)
        {
            extruder_ordering_tmp[extruder_number] = index;
        }

        return extruder_ordering_tmp;
    }();

    std::map<ExtruderNumber, Shape> covered_areas = starting_outlines;

    while (! ranges::all_of(
        extruder_offset_datas,
        [](const ExtruderOffsetData& data)
        {
            return data.done;
        }))
    {
        auto iterator = ranges::min_element(
            extruder_offset_datas,
            [&extruder_ordering, &adhesion_type](const auto& offset1, const auto& offset2)
            {
                if (! offset1.done && ! offset2.done)
                {
                    if (adhesion_type == EPlatformAdhesion::SKIRT || offset1.total_offset == offset2.total_offset)
                    {
                        return extruder_ordering.at(offset1.extruder_nr) < extruder_ordering.at(offset2.extruder_nr);
                    }
                    return offset1.total_offset < offset2.total_offset;
                }
                return ! offset1.done;
            });

        ExtruderOffsetData& extruder_offset_data = *iterator;
        const ExtruderNumber extruder_nr = extruder_offset_data.extruder_nr;
        const ExtruderConfig& extruder_config = extruders_configs.at(extruder_nr);
        const ExtruderNumber outline_extruder_number = adhesion_type == EPlatformAdhesion::SKIRT ? 0 : extruder_nr;

        Shape* starting_outline;
        if (first_extruder_outline_action == FirstExtruderOutlineAction::Use)
        {
            starting_outline = &specific_starting_outlines[extruder_nr];
        }
        else
        {
            starting_outline = &covered_areas.at(outline_extruder_number);
        }

        if (extruder_offset_data.processed_offsets == 0 && first_extruder_outline_action == FirstExtruderOutlineAction::Save)
        {
            specific_starting_outlines[extruder_nr] = *starting_outline;
        }

        const coord_t offset = getLineWidth(extruder_nr, layer_plan) / 2;
        extruder_offset_data.processed_offsets++;
        extruder_offset_data.total_offset += offset;

        const bool update_allowed_areas = first_extruder_outline_action != FirstExtruderOutlineAction::Use;
        const std::vector<ContinuousExtruderMoveSequencePtr> offset_extrusions
            = generateOffset(extruder_nr, offset, *starting_outline, covered_areas[outline_extruder_number], allowed_areas_per_extruder, layer_plan, update_allowed_areas);

        for (const ContinuousExtruderMoveSequencePtr& offset_extrusion : offset_extrusions)
        {
            extruder_offset_data.extrusion->appendExtruderMoveSequence(offset_extrusion);
            extruder_offset_data.extruded_length += offset_extrusion->calculateLength();
        }

        if (offset_extrusions.empty() || layer_plan->getLayerIndex() > 0
            || (extruder_offset_data.processed_offsets >= extruder_config.line_count_ && extruder_offset_data.extruded_length >= extruder_config.skirt_brim_minimal_length_))
        {
            extruder_offset_data.done = true;
        }
    }

    // Now add the generated feature to the proper extruder plans
    for (auto& extruder_offset_data : extruder_offset_datas)
    {
        const auto& feature_extrusion = extruder_offset_data.extrusion;
        const ExtruderNumber extruder_nr = extruder_offset_data.extruder_nr;
        ExtruderPlanPtr extruder_plan = layer_plan->findFirstExtruderPlan(extruder_nr);
        if (! extruder_plan)
        {
            extruder_plan = std::make_shared<ExtruderPlan>(extruder_nr);
        }

        extruder_plan->appendFeatureExtrusion(feature_extrusion);
        layer_plan->appendExtruderPlan(extruder_plan);
    }
}

} // namespace cura