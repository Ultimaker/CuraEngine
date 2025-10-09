// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SkirtBrim.h"

#include <spdlog/spdlog.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Shape.h"
#include "settings/EnumSettings.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "support.h"
#include "utils/MixedPolylineStitcher.h"
#include "utils/Simplify.h"

namespace cura
{

SkirtBrim::SkirtBrim(SliceDataStorage& storage)
    : storage_(storage)
    , adhesion_type_(Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type"))
    , has_ooze_shield_(storage.ooze_shield.size() > 0 && storage.ooze_shield[0].size() > 0)
    , has_draft_shield_(storage.draft_protection_shield.size() > 0)
    , extruders_(Application::getInstance().current_slice_->scene.extruders)
    , extruder_count_(extruders_.size())
    , extruders_configs_(extruder_count_)
{
    const std::vector<bool> used_extruders = storage.getExtrudersUsed();

    std::optional<size_t> first_used_extruder_nr;
    for (size_t extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        const bool extruder_is_used = used_extruders[extruder_nr];
        extruders_configs_[extruder_nr].extruder_is_used_ = extruder_is_used;
        if (extruder_is_used && ! first_used_extruder_nr.has_value())
        {
            first_used_extruder_nr = extruder_nr;
        }
    }
    first_used_extruder_nr_ = first_used_extruder_nr.value_or(0);


    skirt_brim_extruder_nr_ = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<int>("skirt_brim_extruder_nr");
    if (skirt_brim_extruder_nr_ == -1 && adhesion_type_ == EPlatformAdhesion::SKIRT)
    { // Skirt is always printed with all extruders in order to satisfy minimum legnth constraint
        // NOTE: the line count will only be satisfied for the first extruder used.
        skirt_brim_extruder_nr_ = first_used_extruder_nr_;
    }

    for (size_t extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        ExtruderConfig& extruder_config = extruders_configs_[extruder_nr];
        if (! extruder_config.extruder_is_used_)
        {
            continue;
        }

        const ExtruderTrain& extruder = extruders_[extruder_nr];
        const BrimLocation location = extruder.settings_.get<BrimLocation>("brim_location");

        extruder_config.line_width_ = extruder.settings_.get<coord_t>("skirt_brim_line_width") * extruder.settings_.get<Ratio>("initial_layer_line_width_factor");
        extruder_config.skirt_brim_minimal_length_ = extruder.settings_.get<coord_t>("skirt_brim_minimal_length");
        extruder_config.outside_polys_ = adhesion_type_ == EPlatformAdhesion::SKIRT || (location & BrimLocation::OUTSIDE);
        extruder_config.inside_polys_ = adhesion_type_ == EPlatformAdhesion::BRIM && (location & BrimLocation::INSIDE);
        extruder_config.line_count_ = extruder.settings_.get<int>(adhesion_type_ == EPlatformAdhesion::BRIM ? "brim_line_count" : "skirt_line_count");
        extruder_config.gap_ = extruder.settings_.get<coord_t>(adhesion_type_ == EPlatformAdhesion::BRIM ? "brim_gap" : "skirt_gap");
    }
}

std::vector<SkirtBrim::Offset> SkirtBrim::generateBrimOffsetPlan(std::vector<Outline>& starting_outlines)
{
    std::vector<Offset> all_brim_offsets;

    if (skirt_brim_extruder_nr_ >= 0)
    {
        starting_outlines[skirt_brim_extruder_nr_] = getFirstLayerOutline();
    }
    else
    {
        for (int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
        {
            if (! extruders_configs_[extruder_nr].extruder_is_used_)
            {
                continue;
            }
            starting_outlines[extruder_nr] = getFirstLayerOutline(extruder_nr);
        }
    }

    for (int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        const ExtruderConfig& extruder_config = extruders_configs_[extruder_nr];
        const coord_t semi_line_width = extruder_config.line_width_ / 2;

        if (! extruder_config.extruder_is_used_ || (skirt_brim_extruder_nr_ >= 0 && extruder_nr != skirt_brim_extruder_nr_)
            || (starting_outlines[extruder_nr].gapped.empty() && starting_outlines[extruder_nr].touching.empty()))
        {
            continue; // only include offsets for brim extruder
        }

        for (int line_idx = 0; line_idx < extruder_config.line_count_; line_idx++)
        {
            const bool is_last = line_idx == extruder_config.line_count_ - 1;
            const coord_t offset_touching = semi_line_width + extruder_config.line_width_ * line_idx;
            const coord_t offset_gapped = offset_touching + extruder_config.gap_;
            if (line_idx == 0)
            {
                all_brim_offsets.emplace_back(
                    &starting_outlines[extruder_nr],
                    extruder_config.outside_polys_,
                    extruder_config.inside_polys_,
                    offset_gapped,
                    offset_touching,
                    offset_gapped,
                    line_idx,
                    extruder_nr,
                    is_last);
            }
            else
            {
                all_brim_offsets.emplace_back(
                    line_idx - 1,
                    extruder_config.outside_polys_,
                    extruder_config.inside_polys_,
                    extruder_config.line_width_,
                    extruder_config.line_width_,
                    offset_gapped,
                    line_idx,
                    extruder_nr,
                    is_last);
            }
        }
    }

    std::stable_sort(all_brim_offsets.begin(), all_brim_offsets.end(), OffsetSorter);
    return all_brim_offsets;
}

void SkirtBrim::generate()
{
    std::vector<Outline> starting_outlines(extruder_count_);
    std::vector<Offset> all_brim_offsets = generateBrimOffsetPlan(starting_outlines);
    std::vector<Shape> allowed_areas_per_extruder = generateAllowedAreas(starting_outlines);

    // Apply 'approximate convex hull' if the adhesion is skirt _after_ any skirt but also prime-tower-brim adhesion.
    // Otherwise, the now expanded convex hull covered areas will mess with that brim. Fortunately this does not mess
    // with the other area calculation above, since they are either itself a simple/convex shape or relevant for brim.
    Shape covered_area = storage_.getLayerOutlines(
        0,
        /*include_support*/ true,
        /*include_prime_tower*/ adhesion_type_ == EPlatformAdhesion::SKIRT);
    if (adhesion_type_ == EPlatformAdhesion::SKIRT)
    {
        covered_area = covered_area.approxConvexHull();
    }

    std::vector<coord_t> total_length = generatePrimaryBrim(all_brim_offsets, covered_area, allowed_areas_per_extruder);

    // ooze/draft shield brim
    generateShieldBrim(covered_area, allowed_areas_per_extruder);

    { // only allow secondary skirt/brim to appear on the very outside
        covered_area = covered_area.getOutsidePolygons();
        for (int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
        {
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
    }

    // Secondary brim of all other materials which don't meet minimum length constraint yet
    generateSecondarySkirtBrim(covered_area, allowed_areas_per_extruder, total_length);

    // simplify paths to prevent buffer unnerruns in firmware
    const Settings& global_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const coord_t maximum_resolution = global_settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t maximum_deviation = global_settings.get<coord_t>("meshfix_maximum_deviation");
    constexpr coord_t max_area_dev = 0u; // No area deviation applied
    for (int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        for (MixedLinesSet& lines : storage_.skirt_brim[extruder_nr])
        {
            lines = Simplify(maximum_resolution, maximum_deviation, max_area_dev).polyline(lines);
        }
    }
}

std::vector<coord_t> SkirtBrim::generatePrimaryBrim(std::vector<Offset>& all_brim_offsets, Shape& covered_area, std::vector<Shape>& allowed_areas_per_extruder)
{
    std::vector<coord_t> total_length(extruder_count_, 0U);

    for (size_t offset_idx = 0; offset_idx < all_brim_offsets.size(); offset_idx++)
    {
        Offset& offset = all_brim_offsets[offset_idx];
        if (storage_.skirt_brim[offset.extruder_nr_].size() <= offset.inset_idx_)
        {
            storage_.skirt_brim[offset.extruder_nr_].resize(offset.inset_idx_ + 1);
        }
        MixedLinesSet& output_location = storage_.skirt_brim[offset.extruder_nr_][offset.inset_idx_];
        const coord_t added_length = generateOffset(offset, covered_area, allowed_areas_per_extruder, output_location);

        if (added_length == 0)
        { // no more place for more brim. Trying to satisfy minimum length constraint with generateSecondarySkirtBrim
            continue;
        }
        total_length[offset.extruder_nr_] += added_length;

        const ExtruderConfig& extruder_config = extruders_configs_[offset.extruder_nr_];

        if (offset.is_last_ && total_length[offset.extruder_nr_] < extruder_config.skirt_brim_minimal_length_
            && // This was the last offset of this extruder, but the brim lines don't meet minimal length yet
            total_length[offset.extruder_nr_] > 0u // No lines got added; we have no extrusion lines to build on
        )
        {
            offset.is_last_ = false;
            constexpr bool is_last = true;
            all_brim_offsets.emplace_back(
                offset.inset_idx_,
                extruder_config.outside_polys_,
                extruder_config.inside_polys_,
                extruder_config.line_width_,
                offset.total_offset_ + extruder_config.line_width_,
                offset.total_offset_ + extruder_config.line_width_,
                offset.inset_idx_ + 1,
                offset.extruder_nr_,
                is_last);
            std::stable_sort(all_brim_offsets.begin() + offset_idx + 1, all_brim_offsets.end(), OffsetSorter); // reorder remaining offsets
        }
    }
    return total_length;
}

coord_t SkirtBrim::generateOffset(const Offset& offset, Shape& covered_area, std::vector<Shape>& allowed_areas_per_extruder, MixedLinesSet& result)
{
    coord_t length_added;
    Shape brim;
    const ExtruderConfig& extruder_config = extruders_configs_[offset.extruder_nr_];

    if (std::holds_alternative<Outline*>(offset.reference_outline_or_index_))
    {
        Outline* reference_outline = std::get<Outline*>(offset.reference_outline_or_index_);
        for (const auto& [shape, offset_value] :
             { std::make_tuple(reference_outline->gapped, offset.offset_value_gapped_), std::make_tuple(reference_outline->touching, offset.offset_value_touching_) })
        {
            for (const Polygon& polygon : shape)
            {
                const double area = polygon.area();
                if (area > 0 && offset.outside_)
                {
                    brim.push_back(polygon.offset(offset_value, ClipperLib::jtRound));
                }
                else if (area < 0 && offset.inside_)
                {
                    brim.push_back(polygon.offset(-offset_value, ClipperLib::jtRound));
                }
            }
        }
    }
    else
    {
        const int reference_idx = std::get<int>(offset.reference_outline_or_index_);
        const coord_t offset_dist = extruder_config.line_width_;

        brim.push_back(storage_.skirt_brim[offset.extruder_nr_][reference_idx].offset(offset_dist, ClipperLib::jtRound));
    }

    // limit brim lines to allowed areas, stitch them and store them in the result
    brim = Simplify(Application::getInstance().current_slice_->scene.extruders[offset.extruder_nr_].settings_).polygon(brim);

    OpenLinesSet brim_lines = allowed_areas_per_extruder[offset.extruder_nr_].intersection(brim, false);
    length_added = brim_lines.length();

    Shape newly_covered = brim_lines.offset(extruder_config.line_width_ / 2 + 10, ClipperLib::jtRound);

    const coord_t max_stitch_distance = extruder_config.line_width_;
    MixedPolylineStitcher::stitch(brim_lines, result, max_stitch_distance);

    // clean up too small lines (only open ones, which was done historically but may be a mistake)
    result.erase(
        std::remove_if(
            result.begin(),
            result.end(),
            [](const PolylinePtr& line)
            {
                if (const std::shared_ptr<const OpenPolyline> open_line = dynamic_pointer_cast<const OpenPolyline>(line))
                {
                    return open_line->shorterThan(min_brim_line_length);
                }
                return false;
            }),
        result.end());

    // update allowed_areas_per_extruder
    covered_area = covered_area.unionPolygons(newly_covered.unionPolygons());
    for (size_t extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        if (extruders_configs_[extruder_nr].extruder_is_used_)
        {
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
    }

    return length_added;
}

SkirtBrim::Outline SkirtBrim::getFirstLayerOutline(const int extruder_nr /* = -1 */)
{
    Outline first_layer_outline;
    Settings& global_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    int reference_extruder_nr = skirt_brim_extruder_nr_;
    assert(! (reference_extruder_nr == -1 && extruder_nr == -1) && "We should only request the outlines of all layers when the brim is being generated for only one material");
    if (reference_extruder_nr == -1)
    {
        reference_extruder_nr = extruder_nr;
    }
    const ExtruderConfig& reference_extruder_config = extruders_configs_[reference_extruder_nr];
    const int primary_line_count = reference_extruder_config.line_count_;
    const LayerIndex layer_nr = 0;
    if (adhesion_type_ == EPlatformAdhesion::SKIRT)
    {
        int skirt_height = 0;
        for (const auto& extruder : Application::getInstance().current_slice_->scene.extruders)
        {
            if (extruder_nr == -1 || extruder_nr == extruder.extruder_nr_)
            {
                skirt_height = std::max(skirt_height, extruder.settings_.get<int>("skirt_height"));
            }
        }
        skirt_height = std::min(skirt_height, static_cast<int>(storage_.print_layer_count));

        for (int i_layer = layer_nr; i_layer < skirt_height; ++i_layer)
        {
            constexpr bool include_support = true;
            constexpr bool include_prime_tower = true;
            first_layer_outline.gapped = first_layer_outline.gapped.unionPolygons(storage_.getLayerOutlines(i_layer, include_support, include_prime_tower, true));
        }

        Shape shields;
        if (has_ooze_shield_)
        {
            shields = storage_.ooze_shield[0];
        }
        if (has_draft_shield_)
        {
            shields = shields.unionPolygons(storage_.draft_protection_shield);
        }
        first_layer_outline.gapped = first_layer_outline.gapped.unionPolygons(shields.offset(
            reference_extruder_config.line_width_ / 2 // because the shield is printed *on* the stored polygons; not inside hteir area
            - reference_extruder_config.gap_)); // so that when we apply the gap we will end up right next to the shield
        // NOTE: offsetting by -gap here and by +gap in the main brim algorithm effectively performs a morphological close,
        // so in some cases with a large skirt gap and small models and small shield distance
        // the skirt lines can cross the shield lines.
        // This shouldn't be a big problem, since the skirt lines are far away from the model.
        first_layer_outline.gapped = first_layer_outline.gapped.approxConvexHull();
    }
    else
    { // add brim underneath support by removing support where there's brim around the model
        {
            constexpr bool include_support = false; // Don't include the supports yet because we need to reduce them before
            constexpr bool include_prime_tower = false; // Not included, has its own brim
            constexpr bool external_polys_only = false; // Gather all polygons and treat them separately.
            first_layer_outline.gapped = storage_.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_polys_only, extruder_nr);
            first_layer_outline.gapped
                = first_layer_outline.gapped.unionPolygons(); // To guard against overlapping outlines, which would produce holes according to the even-odd rule.
        }

        if (storage_.support.generated && primary_line_count > 0 && ! storage_.support.supportLayers.empty()
            && (extruder_nr == -1 || extruder_nr == global_settings.get<int>("support_infill_extruder_nr")))
        { // remove model-brim from support
            SupportLayer& support_layer = storage_.support.supportLayers[0];
            const ExtruderTrain& support_infill_extruder = global_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
            if (support_infill_extruder.settings_.get<bool>("brim_replaces_support"))
            {
                // avoid gap in the middle
                //    V
                //  +---+     +----+
                //  |+-+|     |+--+|
                //  || ||     ||[]|| > expand to fit an extra brim line
                //  |+-+|     |+--+|
                //  +---+     +----+
                const coord_t primary_extruder_skirt_brim_line_width = reference_extruder_config.line_width_;
                Shape model_brim_covered_area;

                // always leave a gap of an even number of brim lines, so that it fits if it's generating brim from both sides
                const coord_t offset = primary_extruder_skirt_brim_line_width * (primary_line_count + primary_line_count % 2);

                for (const Polygon& polygon : first_layer_outline.gapped)
                {
                    // Compute the fringe that the brim is going to cover around the model
                    Shape outset;
                    Shape inset;

                    double area = polygon.area();
                    if (area > 0 && reference_extruder_config.outside_polys_)
                    {
                        outset = polygon.offset(offset, ClipperLib::jtRound);
                        inset.push_back(polygon);
                    }
                    else if (area < 0 && reference_extruder_config.inside_polys_)
                    {
                        outset.push_back(polygon);
                        inset = polygon.offset(-offset, ClipperLib::jtRound);
                    }

                    outset = outset.difference(inset);
                    model_brim_covered_area = model_brim_covered_area.unionPolygons(outset);
                }

                AABB model_brim_covered_area_boundary_box(model_brim_covered_area);
                support_layer.excludeAreasFromSupportInfillAreas(model_brim_covered_area, model_brim_covered_area_boundary_box);

                // If the gap between the model and the BP is small enough, support starts with the interface instead, so remove it there as well:
                support_layer.support_roof = support_layer.support_roof.difference(model_brim_covered_area);
            }

            for (const SupportInfillPart& support_infill_part : support_layer.support_infill_parts)
            {
                first_layer_outline.touching.push_back(support_infill_part.outline_);
            }
            first_layer_outline.touching.push_back(support_layer.support_bottom);
            first_layer_outline.touching.push_back(support_layer.support_roof);
        }
    }

    for (Shape& outline : std::array{ first_layer_outline.gapped, first_layer_outline.touching })
    {
        constexpr coord_t join_distance = 20;
        outline = outline.offset(join_distance).offset(-join_distance); // merge adjacent models into single polygon
        constexpr coord_t smallest_line_length = 200;
        constexpr coord_t largest_error_of_removed_point = 50;
        outline = Simplify(smallest_line_length, largest_error_of_removed_point, 0).polygon(outline);
    }
    return first_layer_outline;
}

void SkirtBrim::generateShieldBrim(Shape& brim_covered_area, std::vector<Shape>& allowed_areas_per_extruder)
{
    int extruder_nr = skirt_brim_extruder_nr_;
    if (extruder_nr < 0)
    { // the shields are always printed with all extruders, so it doesn't really matter with which extruder we print the brim on the first layer
        extruder_nr = first_used_extruder_nr_;
    }

    const ExtruderConfig& extruder_config = extruders_configs_[extruder_nr];

    // generate brim for ooze shield and draft shield
    if (adhesion_type_ == EPlatformAdhesion::BRIM && (has_ooze_shield_ || has_draft_shield_))
    {
        const coord_t primary_extruder_skirt_brim_line_width = extruder_config.line_width_;
        int primary_line_count = extruder_config.line_count_;

        // generate areas where to make extra brim for the shields
        // avoid gap in the middle
        //    V
        //  +---+     +----+
        //  |+-+|     |+--+|
        //  || ||     ||[]|| > expand to fit an extra brim line
        //  |+-+|     |+--+|
        //  +---+     +----+
        const coord_t primary_skirt_brim_width
            = (primary_line_count + primary_line_count % 2) * primary_extruder_skirt_brim_line_width; // always use an even number, because we will fil the area from both sides

        Shape shield_brim;
        if (has_ooze_shield_)
        {
            shield_brim = storage_.ooze_shield[0].difference(storage_.ooze_shield[0].offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width));
        }
        if (has_draft_shield_)
        {
            shield_brim = shield_brim.unionPolygons(
                storage_.draft_protection_shield.difference(storage_.draft_protection_shield.offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width)));
        }
        shield_brim = shield_brim.intersection(allowed_areas_per_extruder[extruder_nr].offset(primary_extruder_skirt_brim_line_width / 2));
        const Shape layer_outlines = storage_.getLayerOutlines(/*layer_nr*/ 0, /*include_support*/ false, /*include_prime_tower*/ true);
        shield_brim = shield_brim.difference(layer_outlines.getOutsidePolygons()); // don't generate any shield brim inside holes

        const Shape covered_area = shield_brim.offset(primary_extruder_skirt_brim_line_width / 2);
        brim_covered_area = brim_covered_area.unionPolygons(covered_area);
        allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);

        // generate brim within shield_brim
        storage_.skirt_brim[extruder_nr].emplace_back();
        storage_.skirt_brim[extruder_nr].back().push_back(shield_brim);
        while (shield_brim.size() > 0)
        {
            shield_brim = shield_brim.offset(-primary_extruder_skirt_brim_line_width);
            storage_.skirt_brim[extruder_nr].back().push_back(shield_brim); // throw all polygons for the shileds onto one heap; because the brim lines are
                                                                            // generated from both sides the order will not be important
        }
    }

    if (adhesion_type_ == EPlatformAdhesion::SKIRT)
    {
        if (has_ooze_shield_)
        {
            const Shape covered_area = storage_.ooze_shield[0].offset(extruder_config.line_width_ / 2);
            brim_covered_area = brim_covered_area.unionPolygons(covered_area);
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
        if (has_draft_shield_)
        {
            const Shape covered_area = storage_.draft_protection_shield.offset(extruder_config.line_width_ / 2);
            brim_covered_area = brim_covered_area.unionPolygons(covered_area);
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
    }
}

void SkirtBrim::generateSecondarySkirtBrim(Shape& covered_area, std::vector<Shape>& allowed_areas_per_extruder, std::vector<coord_t>& total_length)
{
    constexpr coord_t bogus_total_offset = 0u; // Doesn't matter. The offsets won't be sorted here.
    constexpr bool is_last = false; // Doesn't matter. Isn't used in the algorithm below.
    for (int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        bool first = true;
        Outline reference_outline{ .touching = covered_area };
        const ExtruderConfig& extruder_config = extruders_configs_[extruder_nr];
        while (total_length[extruder_nr] < extruder_config.skirt_brim_minimal_length_)
        {
            decltype(Offset::reference_outline_or_index_) ref_polys_or_idx = nullptr;
            coord_t offset_from_reference;
            if (first)
            {
                ref_polys_or_idx = &reference_outline;
                offset_from_reference = extruder_config.line_width_ / 2;
            }
            else
            {
                ref_polys_or_idx = static_cast<int>(storage_.skirt_brim[extruder_nr].size() - 1);
                offset_from_reference = extruder_config.line_width_;
            }
            const bool outside_polys = extruder_config.outside_polys_;
            const bool inside_polys = extruder_config.inside_polys_;
            Offset extra_offset(
                ref_polys_or_idx,
                outside_polys,
                inside_polys,
                offset_from_reference,
                offset_from_reference,
                bogus_total_offset,
                storage_.skirt_brim[extruder_nr].size(),
                extruder_nr,
                is_last);

            storage_.skirt_brim[extruder_nr].emplace_back();
            MixedLinesSet& output_location = storage_.skirt_brim[extruder_nr].back();
            coord_t added_length = generateOffset(extra_offset, covered_area, allowed_areas_per_extruder, output_location);

            if (! added_length)
            {
                spdlog::warn("Couldn't satisfy minimum length constraint of extruder {}!\n", extruder_nr);
                break;
            }

            total_length[extra_offset.extruder_nr_] += added_length;

            first = false;
        }
    }
}

std::vector<Shape> SkirtBrim::generateAllowedAreas(const std::vector<Outline>& starting_outlines) const
{
    constexpr LayerIndex layer_nr = 0;

    // For each extruder, pre-compute the areas covered by models/supports/prime tower
    struct ExtruderOutlines
    {
        Shape models_outlines;
        Shape supports_outlines;
    };

    std::vector<ExtruderOutlines> covered_area_by_extruder;
    if (adhesion_type_ == EPlatformAdhesion::BRIM)
    {
        covered_area_by_extruder.resize(extruder_count_);
        for (size_t extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
        {
            if (extruders_configs_[extruder_nr].extruder_is_used_)
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
                    extruder_outlines.supports_outlines
                        = storage_.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_polys_only, extruder_nr, include_model);
                }
            }
        }
    }

    std::vector<Shape> allowed_areas_per_extruder(extruder_count_);
    for (size_t extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        const ExtruderConfig& extruder_config = extruders_configs_[extruder_nr];

        if (! extruder_config.extruder_is_used_)
        {
            continue;
        }

        // Initialize allowed area to full build plate, then remove disallowed areas
        Shape& allowed_areas = allowed_areas_per_extruder[extruder_nr];
        allowed_areas = storage_.getMachineBorder(extruder_nr);

        if (adhesion_type_ == EPlatformAdhesion::BRIM)
        {
            const Settings& settings = Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_;
            const coord_t hole_brim_distance = settings.get<coord_t>("brim_inside_margin");

            for (size_t other_extruder_nr = 0; other_extruder_nr < covered_area_by_extruder.size(); ++other_extruder_nr)
            {
                const ExtruderOutlines& extruder_outlines = covered_area_by_extruder[other_extruder_nr];
                const coord_t base_offset = extruder_config.line_width_ / 2;

                // Remove areas covered by models
                for (const Polygon& covered_surface : extruder_outlines.models_outlines)
                {
                    coord_t offset = base_offset;
                    const double covered_area = covered_surface.area();

                    if ((other_extruder_nr == extruder_nr || extruder_nr == skirt_brim_extruder_nr_)
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
        allowed_areas = allowed_areas.difference(starting_outlines[extruder_nr].gapped.offset(extruder_config.gap_ - 50, ClipperLib::jtRound));
        allowed_areas = allowed_areas.difference(starting_outlines[extruder_nr].touching.offset(-50, ClipperLib::jtRound));
    }

    return allowed_areas_per_extruder;
}

void SkirtBrim::generateSupportBrim()
{
    constexpr coord_t brim_area_minimum_hole_size_multiplier = 100;

    Scene& scene = Application::getInstance().current_slice_->scene;
    const ExtruderTrain& support_infill_extruder = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const coord_t brim_line_width
        = support_infill_extruder.settings_.get<coord_t>("skirt_brim_line_width") * support_infill_extruder.settings_.get<Ratio>("initial_layer_line_width_factor");
    size_t line_count = support_infill_extruder.settings_.get<size_t>("support_brim_line_count");
    const coord_t minimal_length = support_infill_extruder.settings_.get<coord_t>("skirt_brim_minimal_length");
    if (! storage_.support.generated || line_count <= 0 || storage_.support.supportLayers.empty())
    {
        return;
    }

    const coord_t brim_width = brim_line_width * line_count;
    coord_t skirt_brim_length = 0;

    if (storage_.skirt_brim[support_infill_extruder.extruder_nr_].empty())
    {
        storage_.skirt_brim[support_infill_extruder.extruder_nr_].emplace_back();
    }

    for (const MixedLinesSet& brim_line : storage_.skirt_brim[support_infill_extruder.extruder_nr_])
    {
        skirt_brim_length += brim_line.length();
    }

    SupportLayer& support_layer = storage_.support.supportLayers[0];

    Shape support_outline;
    for (SupportInfillPart& part : support_layer.support_infill_parts)
    {
        support_outline.push_back(part.outline_);
    }
    const Shape brim_area = support_outline.difference(support_outline.offset(-brim_width));
    support_layer.excludeAreasFromSupportInfillAreas(brim_area, AABB(brim_area));

    coord_t offset_distance = brim_line_width / 2;
    for (size_t skirt_brim_number = 0; skirt_brim_number < line_count; skirt_brim_number++)
    {
        offset_distance -= brim_line_width;

        Shape brim_line = support_outline.offset(offset_distance, ClipperLib::jtRound);

        // Remove small inner skirt and brim holes. Holes have a negative area, remove anything smaller then multiplier x extrusion "area"
        for (size_t n = 0; n < brim_line.size(); n++)
        {
            const double area = brim_line[n].area();
            if (area < 0 && area > -brim_line_width * brim_line_width * brim_area_minimum_hole_size_multiplier)
            {
                brim_line.removeAt(n--);
            }
        }

        const bool brim_line_empty = brim_line.empty(); // Store before moving
        storage_.support_brim.push_back(std::move(brim_line));
        // In case of adhesion::NONE length of support brim is only the length of the brims formed for the support
        const coord_t length = (adhesion_type_ == EPlatformAdhesion::NONE) ? skirt_brim_length : skirt_brim_length + storage_.support_brim.length();
        if (skirt_brim_number + 1 >= line_count && length > 0 && length < minimal_length) // Make brim or skirt have more lines when total length is too small.
        {
            line_count++;
        }
        if (brim_line_empty)
        { // the fist layer of support is fully filled with brim
            break;
        }
    }
}

} // namespace cura
