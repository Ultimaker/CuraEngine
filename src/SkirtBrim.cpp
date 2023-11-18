// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SkirtBrim.h"

#include <spdlog/spdlog.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "settings/EnumSettings.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "support.h"
#include "utils/PolylineStitcher.h"
#include "utils/Simplify.h" //Simplifying the brim/skirt at every inset.

namespace cura
{

SkirtBrim::SkirtBrim(SliceDataStorage& storage)
    : storage(storage)
    , adhesion_type(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type"))
    , has_ooze_shield(storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0)
    , has_draft_shield(storage.draft_protection_shield.size() > 0)
    , extruders(Application::getInstance().current_slice->scene.extruders)
    , extruder_count(extruders.size())
    , extruder_is_used(storage.getExtrudersUsed())
{
    first_used_extruder_nr = 0;
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (extruder_is_used[extruder_nr])
        {
            first_used_extruder_nr = extruder_nr;
            break;
        }
    }
    skirt_brim_extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<int>("skirt_brim_extruder_nr");
    if (skirt_brim_extruder_nr == -1 && adhesion_type == EPlatformAdhesion::SKIRT)
    { // Skirt is always printed with all extruders in order to satisfy minimum legnth constraint
        // NOTE: the line count will only be satisfied for the first extruder used.
        skirt_brim_extruder_nr = first_used_extruder_nr;
    }

    line_widths.resize(extruder_count);
    skirt_brim_minimal_length.resize(extruder_count);
    external_polys_only.resize(extruder_count);
    line_count.resize(extruder_count);
    gap.resize(extruder_count);
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (! extruder_is_used[extruder_nr])
        {
            continue;
        }
        const ExtruderTrain& extruder = extruders[extruder_nr];

        line_widths[extruder_nr] = extruder.settings.get<coord_t>("skirt_brim_line_width") * extruder.settings.get<Ratio>("initial_layer_line_width_factor");
        skirt_brim_minimal_length[extruder_nr] = extruder.settings.get<coord_t>("skirt_brim_minimal_length");
        external_polys_only[extruder_nr] = adhesion_type == EPlatformAdhesion::SKIRT || extruder.settings.get<bool>("brim_outside_only");
        line_count[extruder_nr] = extruder.settings.get<int>(adhesion_type == EPlatformAdhesion::BRIM ? "brim_line_count" : "skirt_line_count");
        gap[extruder_nr] = extruder.settings.get<coord_t>(adhesion_type == EPlatformAdhesion::BRIM ? "brim_gap" : "skirt_gap");
    }
}

std::vector<SkirtBrim::Offset> SkirtBrim::generateBrimOffsetPlan(std::vector<Polygons>& starting_outlines)
{
    std::vector<Offset> all_brim_offsets;

    if (skirt_brim_extruder_nr >= 0)
    {
        starting_outlines[skirt_brim_extruder_nr] = getFirstLayerOutline();
    }
    else
    {
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            if (! extruder_is_used[extruder_nr])
            {
                continue;
            }
            starting_outlines[extruder_nr] = getFirstLayerOutline(extruder_nr);
        }
    }

    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (! extruder_is_used[extruder_nr] || (skirt_brim_extruder_nr >= 0 && extruder_nr != skirt_brim_extruder_nr) || starting_outlines[extruder_nr].empty())
        {
            continue; // only include offsets for brim extruder
        }

        for (int line_idx = 0; line_idx < line_count[extruder_nr]; line_idx++)
        {
            const bool is_last = line_idx == line_count[extruder_nr] - 1;
            coord_t offset = gap[extruder_nr] + line_widths[extruder_nr] / 2 + line_widths[extruder_nr] * line_idx;
            if (line_idx == 0)
            {
                all_brim_offsets.emplace_back(&starting_outlines[extruder_nr], external_polys_only[extruder_nr], offset, offset, line_idx, extruder_nr, is_last);
            }
            else
            {
                all_brim_offsets.emplace_back(line_idx - 1, external_polys_only[extruder_nr], line_widths[extruder_nr], offset, line_idx, extruder_nr, is_last);
            }
        }
    }

    std::sort(all_brim_offsets.begin(), all_brim_offsets.end(), OffsetSorter);
    return all_brim_offsets;
}

void SkirtBrim::generate()
{
    std::vector<Polygons> starting_outlines(extruder_count);
    std::vector<Offset> all_brim_offsets = generateBrimOffsetPlan(starting_outlines);

    constexpr LayerIndex layer_nr = 0;
    constexpr bool include_support = true;
    const bool include_prime_tower = adhesion_type == EPlatformAdhesion::SKIRT;
    const bool has_prime_tower = storage.primeTower.enabled;
    Polygons covered_area = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, /*external_polys_only*/ false);

    std::vector<Polygons> allowed_areas_per_extruder(extruder_count);
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (! extruder_is_used[extruder_nr])
        {
            continue;
        }
        Polygons machine_area = storage.getMachineBorder(extruder_nr);
        allowed_areas_per_extruder[extruder_nr] = machine_area.difference(covered_area);
        if (external_polys_only[extruder_nr])
        {
            // Expand covered area on inside of holes when external_only is enabled for any extruder,
            // so that the brim lines don't overlap with the holes by half the line width
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(getInternalHoleExclusionArea(covered_area, extruder_nr));
        }

        if (has_prime_tower)
        {
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(storage.primeTower.getGroundPoly());
        }
    }

    // Apply 'approximate convex hull' if the adhesion is skirt _after_ any skirt but also prime-tower-brim adhesion.
    // Otherwise, the now expanded convex hull covered areas will mess with that brim. Fortunately this does not mess
    // with the other area calculation above, since they are either itself a simple/convex shape or relevant for brim.
    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        covered_area = covered_area.approxConvexHull();
    }

    std::vector<coord_t> total_length = generatePrimaryBrim(all_brim_offsets, covered_area, allowed_areas_per_extruder);

    // ooze/draft shield brim
    generateShieldBrim(covered_area, allowed_areas_per_extruder);

    { // only allow secondary skirt/brim to appear on the very outside
        covered_area = covered_area.getOutsidePolygons();
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
    }

    // Secondary brim of all other materials which don;t meet minimum length constriant yet
    generateSecondarySkirtBrim(covered_area, allowed_areas_per_extruder, total_length);

    // simplify paths to prevent buffer unnerruns in firmware
    const Settings& global_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t maximum_resolution = global_settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t maximum_deviation = global_settings.get<coord_t>("meshfix_maximum_deviation");
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        for (SkirtBrimLine& line : storage.skirt_brim[extruder_nr])
        {
            constexpr coord_t max_area_dev = 0u; // No area deviation applied
            line.open_polylines = Simplify(maximum_resolution, maximum_deviation, max_area_dev).polyline(line.open_polylines);
            line.closed_polygons = Simplify(maximum_resolution, maximum_deviation, max_area_dev).polygon(line.closed_polygons);
        }
    }
}

std::vector<coord_t> SkirtBrim::generatePrimaryBrim(std::vector<Offset>& all_brim_offsets, Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder)
{
    std::vector<coord_t> total_length(extruder_count, 0U);

    for (size_t offset_idx = 0; offset_idx < all_brim_offsets.size(); offset_idx++)
    {
        Offset& offset = all_brim_offsets[offset_idx];
        if (storage.skirt_brim[offset.extruder_nr].size() <= offset.inset_idx)
        {
            storage.skirt_brim[offset.extruder_nr].resize(offset.inset_idx + 1);
        }
        SkirtBrimLine& output_location = storage.skirt_brim[offset.extruder_nr][offset.inset_idx];
        const coord_t added_length = generateOffset(offset, covered_area, allowed_areas_per_extruder, output_location);

        if (added_length == 0)
        { // no more place for more brim. Trying to satisfy minimum length constraint with generateSecondarySkirtBrim
            continue;
        }
        total_length[offset.extruder_nr] += added_length;

        if (offset.is_last && total_length[offset.extruder_nr] < skirt_brim_minimal_length[offset.extruder_nr]
            && // This was the last offset of this extruder, but the brim lines don't meet minimal length yet
            total_length[offset.extruder_nr] > 0u // No lines got added; we have no extrusion lines to build on
        )
        {
            offset.is_last = false;
            constexpr bool is_last = true;
            all_brim_offsets.emplace_back(
                offset.inset_idx,
                external_polys_only[offset.extruder_nr],
                line_widths[offset.extruder_nr],
                offset.total_offset + line_widths[offset.extruder_nr],
                offset.inset_idx + 1,
                offset.extruder_nr,
                is_last);
            std::sort(all_brim_offsets.begin() + offset_idx + 1, all_brim_offsets.end(), OffsetSorter); // reorder remaining offsets
        }
    }
    return total_length;
}

Polygons SkirtBrim::getInternalHoleExclusionArea(const Polygons& outline, const int extruder_nr)
{
    assert(extruder_nr >= 0);
    const Settings& settings = Application::getInstance().current_slice->scene.extruders[extruder_nr].settings;
    // If brim is external_only, the distance between the external brim of a part inside a hole and the inside hole of the outer part.
    const coord_t hole_brim_distance = settings.get<coord_t>("brim_inside_margin");

    Polygons ret;
    std::vector<PolygonsPart> parts = outline.splitIntoParts();
    for (const PolygonsPart& part : parts)
    {
        for (size_t hole_idx = 1; hole_idx < part.size(); hole_idx++)
        {
            Polygon hole_poly = part[hole_idx];
            hole_poly.reverse();
            Polygons disallowed_region = hole_poly.offset(10u).difference(hole_poly.offset(-line_widths[extruder_nr] / 2 - hole_brim_distance));
            ret = ret.unionPolygons(disallowed_region);
        }
    }
    return ret;
}

coord_t SkirtBrim::generateOffset(const Offset& offset, Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder, SkirtBrimLine& result)
{
    coord_t length_added;
    Polygons brim;
    Polygons newly_covered;
    {
        if (std::holds_alternative<Polygons*>(offset.reference_outline_or_index))
        {
            Polygons* reference_outline = std::get<Polygons*>(offset.reference_outline_or_index);
            if (offset.external_only)
            { // prevent unioning of external polys enclosed by other parts, e.g. a small part inside a hollow cylinder.
                for (Polygons& polys : reference_outline->sortByNesting())
                { // offset external polygons of islands contained within another part in each batch
                    for (PolygonRef poly : polys)
                    {
                        if (poly.area() < 0)
                        {
                            poly.reverse();
                        }
                    }
                    brim.add(polys.offset(offset.offset_value, ClipperLib::jtRound));
                    newly_covered.add(polys.offset(offset.offset_value + line_widths[offset.extruder_nr] / 2, ClipperLib::jtRound));
                    for (PolygonRef poly : polys)
                    {
                        poly.reverse();
                    }
                    newly_covered.add(polys); // don't remove area inside external polygon
                }
            }
            else
            {
                brim = reference_outline->offset(offset.offset_value, ClipperLib::jtRound);
                newly_covered = reference_outline->offset(offset.offset_value + line_widths[offset.extruder_nr] / 2, ClipperLib::jtRound);
            }
        }
        else
        {
            const int reference_idx = std::get<int>(offset.reference_outline_or_index);
            auto offset_dist = line_widths[offset.extruder_nr];

            Polygons local_brim;
            auto closed_polygons_brim = storage.skirt_brim[offset.extruder_nr][reference_idx].closed_polygons.offsetPolyLine(offset_dist, ClipperLib::jtRound, true);
            local_brim.add(closed_polygons_brim);

            auto open_polylines_brim = storage.skirt_brim[offset.extruder_nr][reference_idx].open_polylines.offsetPolyLine(offset_dist, ClipperLib::jtRound);
            local_brim.add(open_polylines_brim);
            local_brim.unionPolygons();

            brim.add(local_brim);

            newly_covered.add(local_brim.offset(offset_dist / 2, ClipperLib::jtRound));
        }
    }

    { // limit brim lines to allowed areas, stitch them and store them in the result
        brim = Simplify(Application::getInstance().current_slice->scene.extruders[offset.extruder_nr].settings).polygon(brim);
        brim.toPolylines();
        Polygons brim_lines = allowed_areas_per_extruder[offset.extruder_nr].intersectionPolyLines(brim, false);
        length_added = brim_lines.polyLineLength();

        const coord_t max_stitch_distance = line_widths[offset.extruder_nr];
        PolylineStitcher<Polygons, Polygon, Point>::stitch(brim_lines, result.open_polylines, result.closed_polygons, max_stitch_distance);

        // clean up too small lines
        for (size_t line_idx = 0; line_idx < result.open_polylines.size();)
        {
            PolygonRef line = result.open_polylines[line_idx];
            if (line.shorterThan(min_brim_line_length))
            {
                result.open_polylines.remove(line_idx);
            }
            else
            {
                line_idx++;
            }
        }
    }

    { // update allowed_areas_per_extruder
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            if (! extruder_is_used[extruder_nr])
            {
                continue;
            }
            covered_area = covered_area.unionPolygons(newly_covered.unionPolygons());
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
    }
    return length_added;
}

Polygons SkirtBrim::getFirstLayerOutline(const int extruder_nr /* = -1 */)
{
    Polygons first_layer_outline;
    Settings& global_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    int reference_extruder_nr = skirt_brim_extruder_nr;
    assert(! (reference_extruder_nr == -1 && extruder_nr == -1) && "We should only request the outlines of all layers when the brim is being generated for only one material");
    if (reference_extruder_nr == -1)
    {
        reference_extruder_nr = extruder_nr;
    }
    const int primary_line_count = line_count[reference_extruder_nr];
    const bool external_only
        = adhesion_type == EPlatformAdhesion::SKIRT || external_polys_only[reference_extruder_nr]; // Whether to include holes or not. Skirt doesn't have any holes.
    const bool has_prime_tower = storage.primeTower.enabled;
    const LayerIndex layer_nr = 0;
    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        constexpr bool include_support = true;
        const bool include_prime_tower = ! has_prime_tower; // include manually otherwise

        first_layer_outline = Polygons();
        int skirt_height = 0;
        for (const auto& extruder : Application::getInstance().current_slice->scene.extruders)
        {
            if (extruder_nr == -1 || extruder_nr == extruder.extruder_nr)
            {
                skirt_height = std::max(skirt_height, extruder.settings.get<int>("skirt_height"));
            }
        }
        skirt_height = std::min(skirt_height, static_cast<int>(storage.print_layer_count));

        for (int i_layer = layer_nr; i_layer < skirt_height; ++i_layer)
        {
            for (const auto& extruder : Application::getInstance().current_slice->scene.extruders)
            {
                first_layer_outline
                    = first_layer_outline.unionPolygons(storage.getLayerOutlines(i_layer, include_support, include_prime_tower, external_only, extruder.extruder_nr));
            }
        }

        if (has_prime_tower)
        {
            first_layer_outline = first_layer_outline.unionPolygons(storage.primeTower.getGroundPoly());
        }

        Polygons shields;
        if (has_ooze_shield)
        {
            shields = storage.oozeShield[0];
        }
        if (has_draft_shield)
        {
            shields = shields.unionPolygons(storage.draft_protection_shield);
        }
        first_layer_outline = first_layer_outline.unionPolygons(shields.offset(
            line_widths[reference_extruder_nr] / 2 // because the shield is printed *on* the stored polygons; not inside hteir area
            - gap[reference_extruder_nr])); // so that when we apply the gap we will end up right next to the shield
        // NOTE: offsetting by -gap here and by +gap in the main brim algorithm effectively performs a morphological close,
        // so in some cases with a large skirt gap and small models and small shield distance
        // the skirt lines can cross the shield lines.
        // This shouldn't be a big problem, since the skirt lines are far away from the model.
        first_layer_outline = first_layer_outline.approxConvexHull();
    }
    else
    { // add brim underneath support by removing support where there's brim around the model
        constexpr bool include_support = false; // Include manually below.
        constexpr bool include_prime_tower = false; // Not included.
        constexpr bool external_outlines_only = false; // Remove manually below.
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_outlines_only, extruder_nr);
        first_layer_outline = first_layer_outline.unionPolygons(); // To guard against overlapping outlines, which would produce holes according to the even-odd rule.
        Polygons first_layer_empty_holes;
        if (external_only)
        {
            first_layer_empty_holes = first_layer_outline.getEmptyHoles();
            first_layer_outline = first_layer_outline.removeEmptyHoles();
        }
        if (storage.support.generated && primary_line_count > 0 && ! storage.support.supportLayers.empty()
            && (extruder_nr == -1 || extruder_nr == global_settings.get<int>("support_infill_extruder_nr")))
        { // remove model-brim from support
            SupportLayer& support_layer = storage.support.supportLayers[0];
            const ExtruderTrain& support_infill_extruder = global_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
            if (support_infill_extruder.settings.get<bool>("brim_replaces_support"))
            {
                // avoid gap in the middle
                //    V
                //  +---+     +----+
                //  |+-+|     |+--+|
                //  || ||     ||[]|| > expand to fit an extra brim line
                //  |+-+|     |+--+|
                //  +---+     +----+
                const coord_t primary_extruder_skirt_brim_line_width = line_widths[reference_extruder_nr];
                Polygons model_brim_covered_area = first_layer_outline.offset(
                    primary_extruder_skirt_brim_line_width * (primary_line_count + primary_line_count % 2),
                    ClipperLib::jtRound); // always leave a gap of an even number of brim lines, so that it fits if it's generating brim from both sides
                if (external_only)
                { // don't remove support within empty holes where no brim is generated.
                    model_brim_covered_area.add(first_layer_empty_holes);
                }
                AABB model_brim_covered_area_boundary_box(model_brim_covered_area);
                support_layer.excludeAreasFromSupportInfillAreas(model_brim_covered_area, model_brim_covered_area_boundary_box);

                // If the gap between the model and the BP is small enough, support starts with the interface instead, so remove it there as well:
                support_layer.support_roof = support_layer.support_roof.difference(model_brim_covered_area);
            }
            for (const SupportInfillPart& support_infill_part : support_layer.support_infill_parts)
            {
                first_layer_outline.add(support_infill_part.outline);
            }
            first_layer_outline.add(support_layer.support_bottom);
            first_layer_outline.add(support_layer.support_roof);
        }
    }
    constexpr coord_t join_distance = 20;
    first_layer_outline = first_layer_outline.offset(join_distance).offset(-join_distance); // merge adjacent models into single polygon
    constexpr coord_t smallest_line_length = 200;
    constexpr coord_t largest_error_of_removed_point = 50;
    first_layer_outline = Simplify(smallest_line_length, largest_error_of_removed_point, 0).polygon(first_layer_outline);
    if (first_layer_outline.empty())
    {
        spdlog::error("Couldn't generate skirt / brim! No polygons on first layer.");
    }
    return first_layer_outline;
}

void SkirtBrim::generateShieldBrim(Polygons& brim_covered_area, std::vector<Polygons>& allowed_areas_per_extruder)
{
    int extruder_nr = skirt_brim_extruder_nr;
    if (extruder_nr < 0)
    { // the shields are always printed with all extruders, so it doesn't really matter with which extruder we print the brim on the first layer
        extruder_nr = first_used_extruder_nr;
    }

    // generate brim for ooze shield and draft shield
    if (adhesion_type == EPlatformAdhesion::BRIM && (has_ooze_shield || has_draft_shield))
    {
        const coord_t primary_extruder_skirt_brim_line_width = line_widths[extruder_nr];
        int primary_line_count = line_count[extruder_nr];

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

        Polygons shield_brim;
        if (has_ooze_shield)
        {
            shield_brim = storage.oozeShield[0].difference(storage.oozeShield[0].offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width));
        }
        if (has_draft_shield)
        {
            shield_brim = shield_brim.unionPolygons(
                storage.draft_protection_shield.difference(storage.draft_protection_shield.offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width)));
        }
        shield_brim = shield_brim.intersection(allowed_areas_per_extruder[extruder_nr].offset(primary_extruder_skirt_brim_line_width / 2));
        const Polygons layer_outlines = storage.getLayerOutlines(/*layer_nr*/ 0, /*include_support*/ false, /*include_prime_tower*/ true, /*external_polys_only*/ false);
        shield_brim = shield_brim.difference(layer_outlines.getOutsidePolygons()); // don't generate any shield brim inside holes

        const Polygons covered_area = shield_brim.offset(primary_extruder_skirt_brim_line_width / 2);
        brim_covered_area = brim_covered_area.unionPolygons(covered_area);
        allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);

        // generate brim within shield_brim
        storage.skirt_brim[extruder_nr].emplace_back();
        storage.skirt_brim[extruder_nr].back().closed_polygons.add(shield_brim);
        while (shield_brim.size() > 0)
        {
            shield_brim = shield_brim.offset(-primary_extruder_skirt_brim_line_width);
            storage.skirt_brim[extruder_nr].back().closed_polygons.add(
                shield_brim); // throw all polygons for the shileds onto one heap; because the brim lines are generated from both sides the order will not be important
        }
    }

    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        if (has_ooze_shield)
        {
            const Polygons covered_area = storage.oozeShield[0].offset(line_widths[extruder_nr] / 2);
            brim_covered_area = brim_covered_area.unionPolygons(covered_area);
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
        if (has_draft_shield)
        {
            const Polygons covered_area = storage.draft_protection_shield.offset(line_widths[extruder_nr] / 2);
            brim_covered_area = brim_covered_area.unionPolygons(covered_area);
            allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
        }
    }
}

void SkirtBrim::generateSecondarySkirtBrim(Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder, std::vector<coord_t>& total_length)
{
    constexpr coord_t bogus_total_offset = 0u; // Doesn't matter. The offsets won't be sorted here.
    constexpr bool is_last = false; // Doesn't matter. Isn't used in the algorithm below.
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        bool first = true;
        Polygons reference_outline = covered_area;
        while (total_length[extruder_nr] < skirt_brim_minimal_length[extruder_nr])
        {
            decltype(Offset::reference_outline_or_index) ref_polys_or_idx = nullptr;
            coord_t offset_from_reference;
            if (first)
            {
                ref_polys_or_idx = &reference_outline;
                offset_from_reference = line_widths[extruder_nr] / 2;
            }
            else
            {
                ref_polys_or_idx = static_cast<int>(storage.skirt_brim[extruder_nr].size() - 1);
                offset_from_reference = line_widths[extruder_nr];
            }
            constexpr bool external_only = false; // The reference outline may contain both outlines and hole polygons.
            Offset extra_offset(ref_polys_or_idx, external_only, offset_from_reference, bogus_total_offset, storage.skirt_brim[extruder_nr].size(), extruder_nr, is_last);

            storage.skirt_brim[extruder_nr].emplace_back();
            SkirtBrimLine& output_location = storage.skirt_brim[extruder_nr].back();
            coord_t added_length = generateOffset(extra_offset, covered_area, allowed_areas_per_extruder, output_location);

            if (! added_length)
            {
                spdlog::warn("Couldn't satisfy minimum length constraint of extruder {}!\n", extruder_nr);
                break;
            }

            total_length[extra_offset.extruder_nr] += added_length;

            first = false;
        }
    }
}

void SkirtBrim::generateSupportBrim()
{
    constexpr coord_t brim_area_minimum_hole_size_multiplier = 100;

    Scene& scene = Application::getInstance().current_slice->scene;
    const ExtruderTrain& support_infill_extruder = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const coord_t brim_line_width
        = support_infill_extruder.settings.get<coord_t>("skirt_brim_line_width") * support_infill_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    size_t line_count = support_infill_extruder.settings.get<size_t>("support_brim_line_count");
    const coord_t minimal_length = support_infill_extruder.settings.get<coord_t>("skirt_brim_minimal_length");
    if (! storage.support.generated || line_count <= 0 || storage.support.supportLayers.empty())
    {
        return;
    }

    const coord_t brim_width = brim_line_width * line_count;
    coord_t skirt_brim_length = 0;

    if (storage.skirt_brim[support_infill_extruder.extruder_nr].empty())
    {
        storage.skirt_brim[support_infill_extruder.extruder_nr].emplace_back();
    }

    for (const SkirtBrimLine& brim_line : storage.skirt_brim[support_infill_extruder.extruder_nr])
    {
        skirt_brim_length += brim_line.closed_polygons.polygonLength();
        skirt_brim_length += brim_line.open_polylines.polyLineLength();
    }

    SupportLayer& support_layer = storage.support.supportLayers[0];

    Polygons support_outline;
    for (SupportInfillPart& part : support_layer.support_infill_parts)
    {
        support_outline.add(part.outline);
    }
    const Polygons brim_area = support_outline.difference(support_outline.offset(-brim_width));
    support_layer.excludeAreasFromSupportInfillAreas(brim_area, AABB(brim_area));

    coord_t offset_distance = brim_line_width / 2;
    for (size_t skirt_brim_number = 0; skirt_brim_number < line_count; skirt_brim_number++)
    {
        offset_distance -= brim_line_width;

        Polygons brim_line = support_outline.offset(offset_distance, ClipperLib::jtRound);

        // Remove small inner skirt and brim holes. Holes have a negative area, remove anything smaller then multiplier x extrusion "area"
        for (size_t n = 0; n < brim_line.size(); n++)
        {
            const double area = brim_line[n].area();
            if (area < 0 && area > -brim_line_width * brim_line_width * brim_area_minimum_hole_size_multiplier)
            {
                brim_line.remove(n--);
            }
        }

        storage.support_brim.add(brim_line);
        // In case of adhesion::NONE length of support brim is only the length of the brims formed for the support
        const coord_t length = (adhesion_type == EPlatformAdhesion::NONE) ? skirt_brim_length : skirt_brim_length + storage.support_brim.polygonLength();
        if (skirt_brim_number + 1 >= line_count && length > 0 && length < minimal_length) // Make brim or skirt have more lines when total length is too small.
        {
            line_count++;
        }
        if (brim_line.empty())
        { // the fist layer of support is fully filled with brim
            break;
        }
    }
}

} // namespace cura
