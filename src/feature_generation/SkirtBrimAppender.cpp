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

SkirtBrimAppender::SkirtBrimAppender(const SliceDataStorage& storage) :
    PrintOperationTransformer<PrintPlan>()
    ,storage_(storage)
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
    std::vector<ConstExtruderPlanPtr> first_extruder_plans;
    ExtrudersSet used_extruders;
    print_plan->applyOnOperationsByType<ExtruderPlan>(
        [&first_extruder_plans, &used_extruders](const ConstExtruderPlanPtr& extruder_plan)
        {
            const ExtruderNumber extruder_nr = extruder_plan->getExtruderNr();
            if (! used_extruders.contains(extruder_nr))
            {
                first_extruder_plans.push_back(extruder_plan);
                used_extruders.set(extruder_plan->getExtruderNr());
            }
        },
        PrintOperationSequence::SearchOrder::Forward,
        2);

    // Gather the skirt/brim-related settings for all used extruders
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice_->scene.extruders)
    {
        if (used_extruders.contains(extruder.extruder_nr_))
        {
            ExtruderConfig& extruder_config = extruders_configs_[extruder.extruder_nr_];
            const Settings& settings = extruder.settings_;

            const auto& location = settings.get<BrimLocation>("brim_location");
            extruder_config.line_width_ = settings.get<coord_t>("skirt_brim_line_width") * extruder.settings_.get<Ratio>("initial_layer_line_width_factor");
            extruder_config.skirt_brim_minimal_length_ = settings.get<coord_t>("skirt_brim_minimal_length");
            extruder_config.outside_polys_ = adhesion_type == EPlatformAdhesion::SKIRT || (location & BrimLocation::OUTSIDE);
            extruder_config.inside_polys_ = adhesion_type == EPlatformAdhesion::BRIM && (location & BrimLocation::INSIDE);
            extruder_config.line_count_ = settings.get<int>(adhesion_type == EPlatformAdhesion::BRIM ? "brim_line_count" : "skirt_line_count");
            extruder_config.gap_ = settings.get<coord_t>(adhesion_type == EPlatformAdhesion::BRIM ? "brim_gap" : "skirt_gap");
            extruder_config.brim_inside_margin_ = settings.get<coord_t>("brim_inside_margin");
        }
    }

    // Find the maximum height we are going to print the skirt/brim to
    size_t height = 1;
    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        for (const ExtruderTrain& extruder : Application::getInstance().current_slice_->scene.extruders)
        {
            if (used_extruders.contains(extruder.extruder_nr_))
            {
                height = std::max(height, extruder.settings_.get<size_t>("skirt_height"));
            }
        }
    }

    std::map<ExtruderNumber, Shape> starting_outlines = generateStartingOutlines(print_plan, skirt_brim_extruder_nr, height, adhesion_type, used_extruders);
    std::vector<Offset> offset_plan = generateBrimOffsetPlan(skirt_brim_extruder_nr, adhesion_type, starting_outlines, used_extruders);
    std::map<ExtruderNumber, Shape> allowed_areas_per_extruder = generateAllowedAreas(starting_outlines, adhesion_type, used_extruders, skirt_brim_extruder_nr, storage_);

    //generateSkirtBrim(storage_, adhesion_type, starting_outlines, offset_plan, allowed_areas_per_extruder, print_plan);
    generateSkirtBrimV2(storage_, adhesion_type, used_extruders, first_extruder_plans, starting_outlines, allowed_areas_per_extruder, print_plan);
}

std::map<size_t, Shape> SkirtBrimAppender::generateStartingOutlines(
    const PrintPlan* print_plan,
    const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
    const size_t height,
    const EPlatformAdhesion adhesion_type,
    const ExtrudersSet& used_extruders)
{
    // Calculate the united footprints of all the extrusion features on the first layers
    std::map<ExtruderNumber, std::vector<Shape>> features_footprints;
    for (const LayerPlanPtr& layer_plan : print_plan->getOperationsAs<LayerPlan>())
    {
        if (layer_plan->getLayerIndex() < height)
        {
            for (const ConstExtruderPlanPtr &extruder_plan :layer_plan->getOperationsAs<ExtruderPlan>())
            {
                const ExtruderNumber target_extruder_nr = adhesion_type == EPlatformAdhesion::SKIRT ? 0 : skirt_brim_extruder_nr.value_or(extruder_plan->getExtruderNr());
                std::vector<Shape> &target_footprints = features_footprints[target_extruder_nr];

                for (const ConstFeatureExtrusionPtr& feature_extrusion : extruder_plan->getOperationsAs<FeatureExtrusion>())
                {
                    target_footprints.push_back(feature_extrusion->calculateFootprint());
                }
            }
        }
    }

    std::map<ExtruderNumber, Shape> starting_outlines;
    for (auto & [extruder_nr, extruder_starting_outlines] : features_footprints)
    {
        starting_outlines[extruder_nr] = Shape::unionShapes(extruder_starting_outlines).getOutsidePolygons();
    }

    // Make sure we have an entry for all used extruders
    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        if (!starting_outlines.contains(extruder_nr))
        {
            starting_outlines[extruder_nr] = Shape();
        }
    }

    return starting_outlines;
}

std::vector<SkirtBrimAppender::Offset> SkirtBrimAppender::generateBrimOffsetPlan(const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
    const EPlatformAdhesion adhesion_type,
    const std::map<ExtruderNumber, Shape>& starting_outlines,
    const ExtrudersSet &used_extruders) const
{
    std::vector<Offset> all_brim_offsets;

    for (ExtruderNumber extruder_nr : used_extruders)
    {
        const ExtruderConfig& extruder_config = extruders_configs_.at(extruder_nr);
        const coord_t semi_line_width = extruder_config.line_width_ / 2;

        if (adhesion_type == EPlatformAdhesion::BRIM && skirt_brim_extruder_nr.has_value() && skirt_brim_extruder_nr.value() != extruder_nr)
        {
            continue; // only include offsets for brim extruder
        }

        const Shape & starting_outline = starting_outlines.at(extruder_nr);
        if (starting_outline.empty())
        {
            continue; // Extruder has empty starting outline
        }

        for (int line_idx = 0; line_idx < extruder_config.line_count_; line_idx++)
        {
            const bool is_last = line_idx == extruder_config.line_count_ - 1;
            coord_t offset = extruder_config.gap_ + semi_line_width + extruder_config.line_width_ * line_idx;
            if (line_idx == 0)
            {
                all_brim_offsets
                    .emplace_back(&starting_outline, extruder_config.outside_polys_, extruder_config.inside_polys_, offset, offset, line_idx, extruder_nr, is_last);
            }
            else
            {
                all_brim_offsets
                    .emplace_back(line_idx - 1, extruder_config.outside_polys_, extruder_config.inside_polys_, extruder_config.line_width_, offset, line_idx, extruder_nr, is_last);
            }
        }
    }

    ranges::sort(all_brim_offsets, sortOffsets);

    return all_brim_offsets;
}

std::map<ExtruderNumber, Shape> SkirtBrimAppender::generateAllowedAreas(
    const std::map<ExtruderNumber, Shape>& starting_outlines,
    const EPlatformAdhesion adhesion_type,
    const ExtrudersSet &used_extruders,
    const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
    const SliceDataStorage& storage) const
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
                extruder_outlines.models_outlines = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_polys_only, extruder_nr, include_model);
            }
            {
                constexpr bool include_support = true;
                constexpr bool include_prime_tower = true;
                constexpr bool include_model = false;
                extruder_outlines.supports_outlines = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_polys_only, extruder_nr, include_model);
            }
        }
    }

    std::map<ExtruderNumber, Shape> allowed_areas_per_extruder;
    for (ExtruderNumber extruder_nr : used_extruders)
    {
        const ExtruderConfig& extruder_config = extruders_configs_.at(extruder_nr);

        // Initialize allowed area to full build plate, then remove disallowed areas
        Shape& allowed_areas = allowed_areas_per_extruder[extruder_nr];
        allowed_areas = storage.getMachineBorder(extruder_nr);

        if (adhesion_type == EPlatformAdhesion::BRIM)
        {
            const coord_t hole_brim_distance = extruder_config.brim_inside_margin_;

            for (size_t other_extruder_nr = 0; other_extruder_nr < covered_area_by_extruder.size(); ++other_extruder_nr)
            {
                const ExtruderOutlines& extruder_outlines = covered_area_by_extruder[other_extruder_nr];
                const coord_t base_offset = extruder_config.line_width_ / 2;

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
        const Shape starting_outline = starting_outlines.at(extruder_nr);
        allowed_areas = allowed_areas.difference(starting_outline.offset(extruder_config.gap_ - 50, ClipperLib::jtRound));
    }

    return allowed_areas_per_extruder;
}

void SkirtBrimAppender::generateSkirtBrim(
    const SliceDataStorage& storage,
    const EPlatformAdhesion adhesion_type,
    const std::map<ExtruderNumber, Shape>& starting_outlines,
    std::vector<Offset>& offset_plan,
    std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
    PrintPlan* print_plan) const
{
    // Apply 'approximate convex hull' if the adhesion is skirt _after_ any skirt but also prime-tower-brim adhesion.
    // Otherwise, the now expanded convex hull covered areas will mess with that brim. Fortunately this does not mess
    // with the other area calculation above, since they are either itself a simple/convex shape or relevant for brim.
    constexpr bool include_support = true;
    const bool include_prime_tower = (adhesion_type == EPlatformAdhesion::SKIRT);
    Shape covered_area = storage.getLayerOutlines(0, include_support, include_prime_tower);
    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        covered_area = covered_area.approxConvexHull();
    }

    std::map<ExtruderNumber, coord_t> total_length;

    for (size_t offset_idx = 0; offset_idx < offset_plan.size(); offset_idx++)
    {
        Offset& offset = offset_plan[offset_idx];
        /*if (storage.skirt_brim[offset.extruder_nr_].size() <= offset.inset_idx_)
        {
            storage.skirt_brim[offset.extruder_nr_].resize(offset.inset_idx_ + 1);
        }
        MixedLinesSet& output_location = storage.skirt_brim[offset.extruder_nr_][offset.inset_idx_];*/

        FeatureExtrusionPtr offset_extrusion
            = generateOffset(offset, starting_outlines, covered_area, allowed_areas_per_extruder, print_plan->getOperationsAs<LayerPlan>().front());

        if (offset_extrusion->empty())
        { // no more place for more brim. Trying to satisfy minimum length constraint with generateSecondarySkirtBrim
            continue;
        }
        total_length[offset.extruder_nr_] += offset_extrusion->calculateLength();

        const ExtruderConfig& extruder_config = extruders_configs_.at(offset.extruder_nr_);


        if (offset.is_last_ && total_length[offset.extruder_nr_] < extruder_config.skirt_brim_minimal_length_
            && // This was the last offset of this extruder, but the brim lines don't meet minimal length yet
            total_length[offset.extruder_nr_] > 0u // No lines got added; we have no extrusion lines to build on
        )
        {
            offset.is_last_ = false;
            constexpr bool is_last = true;
            offset_plan.emplace_back(
                offset.inset_idx_,
                extruder_config.outside_polys_,
                extruder_config.inside_polys_,
                extruder_config.line_width_,
                offset.total_offset_ + extruder_config.line_width_,
                offset.inset_idx_ + 1,
                offset.extruder_nr_,
                is_last);
            std::sort(offset_plan.begin() + offset_idx + 1, offset_plan.end(), sortOffsets); // reorder remaining offsets
        }
    }

    // ooze/draft shield brim
    // generateShieldBrim(covered_area, allowed_areas_per_extruder);

    // { // only allow secondary skirt/brim to appear on the very outside
    //     covered_area = covered_area.getOutsidePolygons();
    //     for (int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    //     {
    //         allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
    //     }
    // }

    // Secondary brim of all other materials which don't meet minimum length constraint yet
    // generateSecondarySkirtBrim(covered_area, allowed_areas_per_extruder, total_length);

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

FeatureExtrusionPtr SkirtBrimAppender::generateOffsetV2(
        const ExtruderNumber extruder_nr,
        const coord_t total_offset,
        Shape& covered_area,
        std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
        const LayerPlanPtr &layer_plan) const
{
    const ExtruderConfig &extruder_config = extruders_configs_.at(extruder_nr);

    Shape brim = covered_area.offset(total_offset, ClipperLib::jtRound);

    // limit brim lines to allowed areas, stitch them and store them in the result
    brim = Simplify(Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_).polygon(brim);

    OpenLinesSet brim_lines = allowed_areas_per_extruder[extruder_nr].intersection(brim, false);

    Shape newly_covered = brim_lines.offset(extruder_config.line_width_ / 2 + 10, ClipperLib::jtRound);

    const coord_t max_stitch_distance = extruder_config.line_width_;
    MixedLinesSet result;
    MixedPolylineStitcher::stitch(brim_lines, result, max_stitch_distance);

    const GCodePathConfig& config = layer_plan->getConfigsStorage()->skirt_brim_config_per_extruder.at(extruder_nr);

    auto feature_extrusion = std::make_shared<FeatureExtrusion>(PrintFeatureType::SkirtBrim, extruder_config.line_width_);
    for (PolylinePtr &extrusion_line: result)
    {
        if (extrusion_line->length() >= min_brim_line_length_)
        {
            feature_extrusion->appendExtruderMoveSequence(ContinuousExtruderMoveSequence::makeFrom(*extrusion_line, extruder_config.line_width_, config.getSpeed()));
        }
    }

    // update allowed_areas_per_extruder
    covered_area = covered_area.unionPolygons(newly_covered.unionPolygons());
    for (Shape& allowed_area : ranges::views::values(allowed_areas_per_extruder))
    {
        allowed_area = allowed_area.difference(covered_area);
    }

    return feature_extrusion;
}

void SkirtBrimAppender::generateSkirtBrimV2(
    const SliceDataStorage& storage,
    const EPlatformAdhesion adhesion_type,
    const ExtrudersSet &used_extruders,
    const std::vector<ConstExtruderPlanPtr> first_extruder_plans,
    const std::map<ExtruderNumber, Shape>& starting_outlines,
    std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
    PrintPlan* print_plan) const
{
    struct ExtruderOffsetData
    {
        coord_t next_offset;
        coord_t extruded_length{0};
    };

    // Create a map containing the processing data for each extruder to be processed. When an extruder is finished,
    // it is removed from the map, so when the map is empty, we are done.
    std::map<ExtruderNumber, ExtruderOffsetData> next_offset;
    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        const ExtruderConfig &extruder_config = extruders_configs_.at(extruder_nr);
        const coord_t first_offset = extruder_config.gap_ + extruder_config.line_width_ / 2;
        next_offset[extruder_nr] = ExtruderOffsetData{ .next_offset = first_offset };
    }

    // Create a cache containing the extruders processing ordering, we are going to need it extensively
    const std::map<ExtruderNumber, size_t> extruder_ordering = [&first_extruder_plans]()
    {
        size_t ordering = 0;
        std::map<ExtruderNumber, size_t> extruder_ordering;
        for (const ConstExtruderPlanPtr &extruder_plan : first_extruder_plans)
        {
            extruder_ordering[extruder_plan->getExtruderNr()] = ordering++;
        }
        return extruder_ordering;
    }();

    std::map<ExtruderNumber, Shape> covered_areas = starting_outlines;

    std::map<ExtruderNumber, std::vector<FeatureExtrusionPtr>> extrusions;
    while (!next_offset.empty())
    {
        auto iterator = ranges::min_element(next_offset, [&extruder_ordering](const auto &offset1, const auto& offset2)
        {
            if (offset1.second.next_offset == offset2.second.next_offset)
            {
                return extruder_ordering.at(offset1.first) < extruder_ordering.at(offset2.first);
            }
            return offset1.second.next_offset < offset2.second.next_offset;
        });

        ExtruderOffsetData &extruder_offset_data = iterator->second;
        const ExtruderNumber extruder_nr = iterator->first;
        const ExtruderConfig &extruder_config = extruders_configs_.at(extruder_nr);
        FeatureExtrusionPtr offset_extrusion = generateOffsetV2(
            extruder_nr,
            extruder_offset_data.next_offset,
            covered_areas.at(extruder_nr),
            allowed_areas_per_extruder,
            print_plan->getOperationsAs<LayerPlan>().front());

        extrusions[extruder_nr].push_back(offset_extrusion);

        extruder_offset_data.extruded_length += offset_extrusion->calculateLength();
        extruder_offset_data.next_offset = extruder_config.line_width_ / 2;

        if (offset_extrusion->empty() || extruder_offset_data.extruded_length >= extruder_config.skirt_brim_minimal_length_)
        {
            next_offset.erase(iterator);
        }
    }

    for (auto & extrusion : extrusions)
    {
        for (auto &feature : extrusion.second)
        {
            print_plan->getOperationsAs<LayerPlan>().front()->getOperationsAs<ExtruderPlan>().front()->appendFeatureExtrusion(feature);
        }
    }
}

FeatureExtrusionPtr SkirtBrimAppender::generateOffset(const Offset& offset,
    const std::map<ExtruderNumber, Shape>& starting_outlines,
    Shape& covered_area,
    std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
    const LayerPlanPtr &layer_plan) const
{
    const ExtruderConfig &extruder_config = extruders_configs_.at(offset.extruder_nr_);


    Shape brim;
    coord_t distance;
    if (offset.inset_idx_ == 0)
    {
        brim = starting_outlines.at(offset.extruder_nr_);
        distance = extruder_config.gap_ + extruder_config.line_width_ * 0.5;
    }
    else
    {
        brim = covered_area;
        distance = extruder_config.line_width_ / 2;
    }
    brim = brim.offset(distance, ClipperLib::jtRound);

    // if (std::holds_alternative<Shape*>(offset.reference_outline_or_index_))
    // {
    //     Shape* reference_outline = std::get<Shape*>(offset.reference_outline_or_index_);
    //     const coord_t offset_value = offset.offset_value_;
    //     for (const Polygon& polygon : *reference_outline)
    //     {
    //         const double area = polygon.area();
    //         if (area > 0 && offset.outside_)
    //         {
    //             brim.push_back(polygon.offset(offset_value, ClipperLib::jtRound));
    //         }
    //         else if (area < 0 && offset.inside_)
    //         {
    //             brim.push_back(polygon.offset(-offset_value, ClipperLib::jtRound));
    //         }
    //     }
    // }
    // else
    // {
    //     const int reference_idx = std::get<int>(offset.reference_outline_or_index_);
    //     const coord_t offset_dist = extruder_config.line_width_;
    //
    //     brim.push_back(storage.skirt_brim[offset.extruder_nr_][reference_idx].offset(offset_dist, ClipperLib::jtRound));
    // }

    // limit brim lines to allowed areas, stitch them and store them in the result
    brim = Simplify(Application::getInstance().current_slice_->scene.extruders[offset.extruder_nr_].settings_).polygon(brim);

    OpenLinesSet brim_lines = allowed_areas_per_extruder[offset.extruder_nr_].intersection(brim, false);

    Shape newly_covered = brim_lines.offset(extruder_config.line_width_ / 2 + 10, ClipperLib::jtRound);

    const coord_t max_stitch_distance = extruder_config.line_width_;
    MixedLinesSet result;
    MixedPolylineStitcher::stitch(brim_lines, result, max_stitch_distance);

    const GCodePathConfig& config = layer_plan->getConfigsStorage()->skirt_brim_config_per_extruder.at(offset.extruder_nr_);

    auto feature_extrusion = std::make_shared<FeatureExtrusion>(PrintFeatureType::SkirtBrim, extruder_config.line_width_);
    for (PolylinePtr &extrusion_line: result)
    {
        feature_extrusion->appendExtruderMoveSequence(ContinuousExtruderMoveSequence::makeFrom(*extrusion_line, extruder_config.line_width_, config.getSpeed()));
    }

    layer_plan->getOperationsAs<ExtruderPlan>().front()->appendFeatureExtrusion(feature_extrusion);

    // clean up too small lines (only open ones, which was done historically but may be a mistake)
    // result.erase(
    //     std::remove_if(
    //         result.begin(),
    //         result.end(),
    //         [](const PolylinePtr& line)
    //         {
    //             if (const std::shared_ptr<const OpenPolyline> open_line = dynamic_pointer_cast<const OpenPolyline>(line))
    //             {
    //                 return open_line->shorterThan(min_brim_line_length_);
    //             }
    //             return false;
    //         }),
    //     result.end());

    // update allowed_areas_per_extruder
    covered_area = covered_area.unionPolygons(newly_covered.unionPolygons());
    for (Shape& allowed_area : ranges::views::values(allowed_areas_per_extruder))
    {
        allowed_area = allowed_area.difference(covered_area);
    }

    return feature_extrusion;
}

} // namespace cura