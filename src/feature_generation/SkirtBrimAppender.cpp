// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/SkirtBrimAppender.h"

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
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/map.hpp>

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/LayerPlanPtr.h"
#include "sliceDataStorage.h"
#include "utils/ExtrudersSet.h"
#include "utils/SVG.h"

namespace cura
{

coord_t SkirtBrimAppender::ExtruderConfig::getLineWidth(const ConstLayerPlanPtr& layer_plan) const
{
    return getLineWidth(layer_plan->getLayerIndex());
}

SkirtBrimAppender::SkirtBrimAppender(const SliceDataStorage& storage)
    : PrintOperationTransformer()
    , storage_(storage)
{
}

void SkirtBrimAppender::process(PrintPlan* print_plan)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;
    const auto adhesion_type = settings.get<EPlatformAdhesion>("adhesion_type");
    const auto support_brim_enable = settings.get<bool>("support_brim_enable");

    if (adhesion_type != EPlatformAdhesion::SKIRT && adhesion_type != EPlatformAdhesion::BRIM && ! support_brim_enable)
    {
        return;
    }

    const auto skirt_brim_extruder_nr_setting = adhesion_type == EPlatformAdhesion::SKIRT ? -1 : settings.get<int>("skirt_brim_extruder_nr");
    const std::optional<ExtruderNumber> skirt_brim_extruder_nr = skirt_brim_extruder_nr_setting >= 0 ? std::make_optional(skirt_brim_extruder_nr_setting) : std::nullopt;

    // Get the first extruder plan for each extruder
    std::vector<ExtruderNumber> used_extruders = generateUsedExtruders(print_plan);

    const std::map<ExtruderNumber, ExtruderConfig> extruders_configs = generateExtrudersConfigs(used_extruders, adhesion_type);
    const size_t height = calculateMaxHeight(extruders_configs, adhesion_type);

    const std::map<ExtruderNumber, Shape> starting_outlines = generateStartingOutlines(print_plan, skirt_brim_extruder_nr, height, adhesion_type, used_extruders);
    const std::map<ExtruderNumber, Shape> allowed_areas_per_extruder
        = generateAllowedAreas(starting_outlines, adhesion_type, used_extruders, skirt_brim_extruder_nr, extruders_configs);

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
                used_extruders,
                starting_outlines,
                specific_starting_outlines,
                allowed_areas_per_extruder,
                extruders_configs,
                layer_plan,
                first_extruder_outline_action);
        }
    }
}

std::vector<ExtruderNumber> SkirtBrimAppender::generateUsedExtruders(const PrintPlan* print_plan)
{
    std::vector<ExtruderNumber> used_extruders;

    print_plan->applyOnOperationsByType<ExtruderPlan>(
        [&used_extruders](const ConstExtruderPlanPtr& extruder_plan)
        {
            const ExtruderNumber extruder_nr = extruder_plan->getExtruderNr();
            if (! ranges::contains(used_extruders, extruder_nr))
            {
                used_extruders.push_back(extruder_nr);
            }
        },
        PrintOperationSequence::SearchOrder::Forward,
        2);

    return used_extruders;
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

std::map<size_t, Shape> SkirtBrimAppender::generateStartingOutlines(
    const PrintPlan* print_plan,
    const std::optional<ExtruderNumber> brim_extruder_nr,
    const size_t height,
    const EPlatformAdhesion adhesion_type,
    std::vector<ExtruderNumber>& used_extruders)
{
    // Calculate the united footprints of all the extrusion features on the first layers
    std::map<ExtruderNumber, std::vector<Shape>> features_footprints;
    for (const LayerPlanPtr& layer_plan : print_plan->getOperationsAs<LayerPlan>())
    {
        if (layer_plan->getLayerIndex() < height)
        {
            for (const ConstExtruderPlanPtr& extruder_plan : layer_plan->getOperationsAs<ExtruderPlan>())
            {
                const ExtruderNumber target_extruder_nr = adhesion_type == EPlatformAdhesion::SKIRT ? 0 : brim_extruder_nr.value_or(extruder_plan->getExtruderNr());
                std::vector<Shape>& target_footprints = features_footprints[target_extruder_nr];

                for (const ConstFeatureExtrusionPtr& feature_extrusion : extruder_plan->getOperationsAs<FeatureExtrusion>())
                {
                    target_footprints.push_back(feature_extrusion->calculateFootprint());
                }
            }
        }
    }

    std::map<ExtruderNumber, Shape> starting_outlines;
    for (auto& [extruder_nr, extruder_starting_outlines] : features_footprints)
    {
        starting_outlines[extruder_nr] = Shape::unionShapes(extruder_starting_outlines).getOutsidePolygons();
    }

    // Make sure we have an entry for all used extruders
    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        if (! starting_outlines.contains(extruder_nr))
        {
            starting_outlines[extruder_nr] = Shape();
        }
    }

    return starting_outlines;
}

std::map<ExtruderNumber, Shape> SkirtBrimAppender::generateAllowedAreas(
    const std::map<ExtruderNumber, Shape>& starting_outlines,
    const EPlatformAdhesion adhesion_type,
    std::vector<ExtruderNumber>& used_extruders,
    const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
    const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs) const
{
    constexpr LayerIndex layer_nr = 0;

    // For each extruder, pre-compute the areas covered by models/supports/prime tower
    struct ExtruderOutlines
    {
        Shape models_outlines;
        Shape supports_outlines;
    };

    std::map<ExtruderNumber, ExtruderOutlines> covered_area_by_extruder;
    if (adhesion_type == EPlatformAdhesion::BRIM)
    {
        for (ExtruderNumber extruder_nr : used_extruders)
        {
            // Gather models/support/prime tower areas separately to apply different margins
            ExtruderOutlines& extruder_outlines = covered_area_by_extruder[extruder_nr];
            constexpr bool external_polys_only = false;
            {
                constexpr bool include_support = false;
                constexpr bool include_prime_tower = false;
                constexpr bool include_model = true;
                extruder_outlines.models_outlines = storage_.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_polys_only, extruder_nr, include_model);
            }
            {
                constexpr bool include_support = true;
                constexpr bool include_prime_tower = true;
                constexpr bool include_model = false;
                extruder_outlines.supports_outlines = storage_.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_polys_only, extruder_nr, include_model);
            }
        }
    }

    std::map<ExtruderNumber, Shape> allowed_areas_per_extruder;
    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        // Initialize allowed area to full build plate, then remove disallowed areas
        Shape& allowed_areas = allowed_areas_per_extruder[extruder_nr];
        allowed_areas = storage_.getMachineBorder(extruder_nr);

        if (adhesion_type == EPlatformAdhesion::BRIM)
        {
            const ExtruderConfig& extruder_config = extruders_configs.at(extruder_nr);
            const coord_t hole_brim_distance = extruder_config.brim_inside_margin_;

            for (size_t other_extruder_nr = 0; other_extruder_nr < covered_area_by_extruder.size(); ++other_extruder_nr)
            {
                const ExtruderOutlines& extruder_outlines = covered_area_by_extruder[other_extruder_nr];
                const coord_t base_offset = extruder_config.line_width_0_ / 2;

                // Remove areas covered by models
                for (const Polygon& covered_surface : extruder_outlines.models_outlines)
                {
                    coord_t offset = base_offset;
                    const double covered_area = covered_surface.area();

                    if ((other_extruder_nr == extruder_nr || extruder_nr == skirt_brim_extruder_nr)
                        && ((covered_area > 0 && extruder_config.outside_polys_) || (covered_area < 0 && extruder_config.inside_polys_)))
                    {
                        // This is an area we are gonna intentionnally print brim in, use the actual gap
                        offset += extruder_config.gap_ - 50; // Lower margin a bit to avoid discarding legitimate lines
                    }
                    else
                    {
                        // This is an area we do not expect brim to be printed in, use a larger gap to keep the printed surface clean
                        offset += hole_brim_distance;
                    }

                    if (covered_area < 0)
                    {
                        // Invert offset to make holes grow inside
                        allowed_areas.push_back(covered_surface.offset(-offset, ClipperLib::jtRound));
                    }
                    else
                    {
                        allowed_areas = allowed_areas.difference(covered_surface.offset(offset, ClipperLib::jtRound));
                    }
                }

                // Remove areas covered by support, with a low margin because we don't care if the brim touches it
                allowed_areas = allowed_areas.difference(extruder_outlines.supports_outlines.offset(base_offset - 50));
            }
        }

        // Anyway, don't allow a brim/skirt to grow inside itself, which may happen e.g. with ooze shield+skirt
        const Shape starting_outline = starting_outlines.at(adhesion_type == EPlatformAdhesion::SKIRT ? 0 : extruder_nr);

        // When using a skirt, the used gap will be the one of the first processed extruder
        const ExtruderNumber gap_extruder_nr = adhesion_type == EPlatformAdhesion::SKIRT ? used_extruders.front() : extruder_nr;
        const coord_t gap = extruders_configs.at(gap_extruder_nr).gap_;
        allowed_areas = allowed_areas.difference(starting_outline.offset(gap - 50, ClipperLib::jtRound));
    }

    return allowed_areas_per_extruder;
}

std::vector<ContinuousExtruderMoveSequencePtr> SkirtBrimAppender::generateOffset(
    const ExtruderNumber extruder_nr,
    const coord_t total_offset,
    Shape& covered_area,
    std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
    const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
    const LayerPlanPtr& layer_plan,
    const bool update_allowed_areas)
{
    const ExtruderConfig& extruder_config = extruders_configs.at(extruder_nr);
    const coord_t line_width = extruder_config.getLineWidth(layer_plan);

    Shape brim = covered_area.offset(total_offset, ClipperLib::jtRound);

    // limit brim lines to allowed areas, stitch them and store them in the result
    brim = Simplify(Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_).polygon(brim);

    OpenLinesSet brim_lines = allowed_areas_per_extruder[extruder_nr].intersection(brim, false);

    Shape newly_covered = brim_lines.offset((line_width / 2) + 10, ClipperLib::jtRound);

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
        coord_t total_offset{ 0 };
        coord_t extruded_length{ 0 };
        bool done{ false };
        size_t processed_offsets{ 0 };
    };

    // Create a vector containing the processing data for each extruder to be processed,
    // ordered in actual processing order
    std::vector<ExtruderOffsetData> extruder_offset_datas;
    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        const ExtruderConfig& extruder_config = extruders_configs.at(extruder_nr);
        if (extruder_config.skirt_height_ > layer_plan->getLayerIndex())
        {
            extruder_offset_datas.push_back(
                ExtruderOffsetData{ .extruder_nr = extruder_nr,
                                    .extrusion = std::make_shared<FeatureExtrusion>(PrintFeatureType::SkirtBrim, extruder_config.getLineWidth(layer_plan)) });
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
    bool first_offset = true;

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

        Shape* starting_outline;
        if (first_extruder_outline_action == FirstExtruderOutlineAction::Use)
        {
            starting_outline = &specific_starting_outlines[extruder_nr];
        }
        else
        {
            starting_outline = &covered_areas.at(adhesion_type == EPlatformAdhesion::SKIRT ? 0 : extruder_nr);
        }

        if (extruder_offset_data.processed_offsets == 0 && first_extruder_outline_action == FirstExtruderOutlineAction::Save)
        {
            specific_starting_outlines[extruder_nr] = *starting_outline;
        }

        const bool add_gap = (adhesion_type == EPlatformAdhesion::SKIRT) ? first_offset : (extruder_offset_data.processed_offsets == 0);
        const coord_t offset = (extruder_config.getLineWidth(layer_plan) / 2) + (add_gap ? extruder_config.gap_ : 0);
        extruder_offset_data.processed_offsets++;
        extruder_offset_data.total_offset += offset;

        const bool update_allowed_areas = first_extruder_outline_action != FirstExtruderOutlineAction::Use;
        const std::vector<ContinuousExtruderMoveSequencePtr> offset_extrusions
            = generateOffset(extruder_nr, offset, *starting_outline, allowed_areas_per_extruder, extruders_configs, layer_plan, update_allowed_areas);

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

        first_offset = false;
    }

    // Now add the generated feature to the proper extruder plans
    for (auto& extruder_offset_data : extruder_offset_datas)
    {
        const auto& feature_extrusion = extruder_offset_data.extrusion;
        const ExtruderNumber extruder_nr = extruder_offset_data.extruder_nr;
        ExtruderPlanPtr extruder_plan = layer_plan->findFirstExtruderPlan(extruder_nr);
        if (! extruder_plan)
        {
            // FIXME: Find a way to easily create an extruder plan (maybe it should not contain the travel speeds)
            const SpeedDerivatives& travel_speed = layer_plan->getConfigsStorage()->travel_config_per_extruder[extruder_nr].speed_derivatives;
            extruder_plan = std::make_shared<ExtruderPlan>(extruder_nr, travel_speed);
        }

        extruder_plan->appendFeatureExtrusion(feature_extrusion);
        layer_plan->appendExtruderPlan(extruder_plan);
    }
}

} // namespace cura