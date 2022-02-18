//Copyright (C) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SkirtBrim.h"

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "support.h"
#include "settings/types/Ratio.h"
#include "settings/EnumSettings.h"
#include "utils/logoutput.h"
#include "utils/PolylineStitcher.h"

namespace cura 
{


// TODO
/*
- fix min volume constraint
- fix connection between normal brim and prime tower brim?
- fix draft shield etc
- fix print order?!



*/

SkirtBrim::SkirtBrim(SliceDataStorage& storage)
: storage(storage)
, is_brim(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type")== EPlatformAdhesion::BRIM)
, is_skirt( ! is_brim)
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
    if (skirt_brim_extruder_nr == -1 && is_skirt)
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
        if ( ! extruder_is_used[extruder_nr]) continue;
    
        const ExtruderTrain& extruder = extruders[extruder_nr];

        line_widths[extruder_nr] = extruder.settings.get<coord_t>("skirt_brim_line_width") * extruder.settings.get<Ratio>("initial_layer_line_width_factor");
        skirt_brim_minimal_length[extruder_nr] = extruder.settings.get<coord_t>("skirt_brim_minimal_length");
        external_polys_only[extruder_nr] = is_skirt || extruder.settings.get<bool>("brim_outside_only");
        line_count[extruder_nr] = extruder.settings.get<int>(is_brim? "brim_line_count" : "skirt_line_count");
        gap[extruder_nr] = extruder.settings.get<coord_t>(is_brim? "brim_gap" : "skirt_gap");
    }
}

void SkirtBrim::generate()
{
    std::vector<Offset> all_brim_offsets;

    constexpr LayerIndex layer_nr = 0;
    std::vector<Polygons> starting_outlines(extruder_count);
    if (skirt_brim_extruder_nr >= 0)
    {
        starting_outlines[skirt_brim_extruder_nr] = getFirstLayerOutline(-1);
    }
    else
    {
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            if ( ! extruder_is_used[extruder_nr]) continue;
            starting_outlines[extruder_nr] = getFirstLayerOutline(extruder_nr);
        }
    }

    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if ( ! extruder_is_used[extruder_nr]) continue;
        if (skirt_brim_extruder_nr >= 0 && extruder_nr != skirt_brim_extruder_nr)
        {
            continue; // only include offsets for brim extruder
        }

        for (int line_idx = 0; line_idx < line_count[extruder_nr]; line_idx++)
        {
            const bool is_last = line_idx == line_count[extruder_nr] - 1;
            coord_t offset = gap[extruder_nr] + line_widths[extruder_nr] / 2 + line_widths[extruder_nr] * line_idx;
            if (line_idx == 0)
            {
                constexpr int reference_idx = -1; // not used, we use the outliens as reference instead
                all_brim_offsets.emplace_back(&starting_outlines[extruder_nr], reference_idx, external_polys_only[extruder_nr], offset, offset, line_idx, extruder_nr, is_last);
            }
            else
            {
                constexpr Polygons* reference_outline = nullptr; // not used, we use previous brimlines as reference instead
                all_brim_offsets.emplace_back(reference_outline, line_idx - 1, external_polys_only[extruder_nr], line_widths[extruder_nr], offset, line_idx, extruder_nr, is_last);
            }
        }
    }

    
    std::sort(all_brim_offsets.begin(), all_brim_offsets.end(), OffsetSorter{});
    
    coord_t max_offset = 0;
    for (const Offset& offset : all_brim_offsets)
    {
        max_offset = std::max(max_offset, offset.offset_value);
    }
    
    std::vector<coord_t> total_length(extruder_count, 0u);
    
    const bool include_support = true;
    Polygons covered_area = storage.getLayerOutlines(layer_nr, include_support, /*include_prime_tower*/ true, /*external_polys_only*/ false);
    
    std::vector<Polygons> allowed_areas_per_extruder(extruder_count);
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if ( ! extruder_is_used[extruder_nr]) continue;
        Polygons machine_area = storage.getMachineBorder(extruder_nr);
        allowed_areas_per_extruder[extruder_nr] = machine_area.difference(covered_area);
    }
    
    for (size_t offset_idx = 0; offset_idx < all_brim_offsets.size(); offset_idx++)
    {
        const Offset& offset = all_brim_offsets[offset_idx];
        if (storage.skirt_brim[offset.extruder_nr].size() <= offset.inset_idx)
        {
            storage.skirt_brim[offset.extruder_nr].resize(offset.inset_idx + 1);
        }
        SkirtBrimLine& output_location = storage.skirt_brim[offset.extruder_nr][offset.inset_idx];
        coord_t added_length = generateOffset(offset, covered_area, allowed_areas_per_extruder, output_location);
        if ( ! added_length)
        { // no more place for more brim. Trying to satisfy minimum length constraint with generateSecondarySkirtBrim
            break;
        }
        total_length[offset.extruder_nr] += added_length;
        
        if (offset.is_last
            // v This was the last offset of this extruder, but the brim lines don't meet minimal length yet
            && total_length[offset.extruder_nr] < skirt_brim_minimal_length[offset.extruder_nr]
            && total_length[offset.extruder_nr] > 0u // No lines got added; we have no extrusion lines to build on
        )
        {
            offset.is_last = false;
            constexpr bool is_last = true;
            constexpr Polygons* reference_outline = nullptr; // not used
            all_brim_offsets.emplace_back(reference_outline, offset.inset_idx, external_polys_only[offset.extruder_nr], line_widths[offset.extruder_nr], offset.total_offset + line_widths[offset.extruder_nr], offset.inset_idx + 1, offset.extruder_nr, is_last);
            std::sort(all_brim_offsets.begin() + offset_idx + 1, all_brim_offsets.end(), OffsetSorter{}); // reorder remaining offsets
        }
    }
    
    if (is_skirt)
    { // prevent areas inside the convex hull from being covered with skirt

        covered_area = covered_area.unionPolygons(getFirstLayerOutline(first_used_extruder_nr));
    }
    
    // ooze/draft shield brim
    generateShieldBrim(covered_area);
    
    // Secondary brim of all other materials which don;t meet minimum length constriant yet
    generateSecondarySkirtBrim(covered_area, allowed_areas_per_extruder, total_length);
    
    
    // TODO list:
    
    // remove small open lines

    // TODO: make allowed areas a bit smaller so that internal external-only brims don't overlap with model by half the line width
    
    // TODO: only put secondary brim on the external outside of the primary brim?
    
    // fix external only offsetting. E.g. with brim distance the separately offsetted brim lines may intersect with each other.
    
    // remove prime blobs from brim
    
    // remove prime tower from shields OR fix disallowed areas in frontend!

    // robustness against when things are empty (brim lines, layer outlines, etc)
    
    // const correctness
    
    // fix printing order
    // See also TODO at the bottom of this file
    
    // documentation
    
    // frontend stuff
    
    // make sure one-at-a-time mode will still consider the brims
    
    
    
    // simplify
    
}

coord_t SkirtBrim::generateOffset(const Offset& offset, Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder, SkirtBrimLine& result)
{
    constexpr bool indent_to_prevent_git_changes = true; // TODO
    if (indent_to_prevent_git_changes)
    {
        coord_t length_added;
        Polygons brim;
        Polygons newly_covered;
        {
            if (offset.reference_outline)
            {
                if (offset.external_only)
                { // prevent unioning of external polys enclosed by other parts, e.g. a small part inside a hollow cylinder.
                    for (ConstPolygonRef poly : *offset.reference_outline)
                    {
                        brim.add(poly.offset(offset.offset_value, ClipperLib::jtRound));
                        {
                            newly_covered.add(poly.offset(offset.offset_value + line_widths[offset.extruder_nr] / 2, ClipperLib::jtRound));
                            Polygon reference = poly;
                            reference.reverse();
                            newly_covered.add(reference); // don't remove area inside external polygon
                        }
                    }
                }
                else
                {
                    brim = offset.reference_outline->offset(offset.offset_value, ClipperLib::jtRound);
                    newly_covered = offset.reference_outline->offset(offset.offset_value + line_widths[offset.extruder_nr] / 2, ClipperLib::jtRound);
                }
            }
            else
            {
                
                Polygons polylines = storage.skirt_brim[offset.extruder_nr][offset.reference_idx].closed_polygons;
                polylines.toPolylines();
                polylines.add(storage.skirt_brim[offset.extruder_nr][offset.reference_idx].open_polylines);
                brim.add(polylines.offsetPolyLine(line_widths[offset.extruder_nr], ClipperLib::jtRound));
                newly_covered.add(polylines.offsetPolyLine(line_widths[offset.extruder_nr] * 3 / 2, ClipperLib::jtRound));
            }
        }

        { // limit brim lines to allowed areas, stitch them and store them in the result
            brim.simplify();
            brim.toPolylines();
            Polygons brim_lines = allowed_areas_per_extruder[offset.extruder_nr].intersectionPolyLines(brim, false);
            length_added = brim_lines.polyLineLength();

            const coord_t max_stitch_distance = line_widths[offset.extruder_nr];
            PolylineStitcher<Polygons, Polygon, Point>::stitch(brim_lines, result.open_polylines, result.closed_polygons, max_stitch_distance);
        }
        
        { // update allowed_areas_per_extruder
            for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
            {
                if ( ! extruder_is_used[extruder_nr]) continue;
                covered_area = covered_area.unionPolygons(newly_covered.unionPolygons());
                allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
            }
        }
        return length_added;
    }
}

Polygons SkirtBrim::getFirstLayerOutline(const int extruder_nr)
{
    Polygons first_layer_outline;
    Settings& global_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    int reference_extruder_nr = skirt_brim_extruder_nr;
    assert( ! (reference_extruder_nr == -1 && extruder_nr == -1) && "We should only request the outlines of all layers when the brim is being generated for only one material");
    if (reference_extruder_nr == -1)
    {
        reference_extruder_nr = extruder_nr;
    }
    const int primary_line_count = line_count[reference_extruder_nr];
    const bool external_only = is_skirt || external_polys_only[reference_extruder_nr]; //Whether to include holes or not. Skirt doesn't have any holes.
    const LayerIndex layer_nr = 0;
    if (is_skirt)
    {
        constexpr bool include_support = true;
        constexpr bool include_prime_tower = true;
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_only, extruder_nr);
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
        constexpr bool include_support = false; //Include manually below.
        constexpr bool include_prime_tower = false; //Include manually below.
        constexpr bool external_outlines_only = false; //Remove manually below.
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_outlines_only, extruder_nr);
        first_layer_outline = first_layer_outline.unionPolygons(); //To guard against overlapping outlines, which would produce holes according to the even-odd rule.
        Polygons first_layer_empty_holes;
        if (external_only)
        {
            first_layer_empty_holes = first_layer_outline.getEmptyHoles();
            first_layer_outline = first_layer_outline.removeEmptyHoles();
        }
        if (storage.support.generated && primary_line_count > 0 && !storage.support.supportLayers.empty()
            && (extruder_nr == -1 || extruder_nr == global_settings.get<int>("support_infill_extruder_nr"))
        )
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
                Polygons model_brim_covered_area = first_layer_outline.offset(primary_extruder_skirt_brim_line_width * (primary_line_count + primary_line_count % 2), ClipperLib::jtRound); // always leave a gap of an even number of brim lines, so that it fits if it's generating brim from both sides
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
        if (storage.primeTower.enabled
            && global_settings.get<bool>("prime_tower_brim_enable")
            && (extruder_nr == -1 || int(storage.primeTower.extruder_order[0]) == extruder_nr))
        {
            first_layer_outline.add(storage.primeTower.outer_poly); // don't remove parts of the prime tower, but make a brim for it
        }
    }
    constexpr coord_t join_distance = 20;
    first_layer_outline = first_layer_outline.offset(join_distance).offset(-join_distance); // merge adjacent models into single polygon
    constexpr coord_t smallest_line_length = 200;
    constexpr coord_t largest_error_of_removed_point = 50;
    first_layer_outline.simplify(smallest_line_length, largest_error_of_removed_point); // simplify for faster processing of the brim lines
    if (first_layer_outline.size() == 0)
    {
        logError("Couldn't generate skirt / brim! No polygons on first layer.\n");
    }
    return first_layer_outline;
}

void SkirtBrim::generateShieldBrim(Polygons& brim_covered_area)
{
    int extruder_nr = skirt_brim_extruder_nr;
    if (extruder_nr < 0)
    { // the shields are always printed with all extruders, so it doesn't really matter with which extruder we print the brim on the first layer
        extruder_nr = first_used_extruder_nr;
    }

    // generate brim for ooze shield and draft shield
    if (!is_skirt && (has_ooze_shield || has_draft_shield))
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
        const coord_t primary_skirt_brim_width = (primary_line_count + primary_line_count % 2) * primary_extruder_skirt_brim_line_width; // always use an even number, because we will fil the area from both sides

        Polygons shield_brim;
        if (has_ooze_shield)
        {
            shield_brim = storage.oozeShield[0].difference(storage.oozeShield[0].offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width));
        }
        if (has_draft_shield)
        {
            shield_brim = shield_brim.unionPolygons(storage.draft_protection_shield.difference(storage.draft_protection_shield.offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width)));
        }
        shield_brim = shield_brim.difference(brim_covered_area.offset(primary_extruder_skirt_brim_line_width / 2));

        brim_covered_area = brim_covered_area.unionPolygons(shield_brim.offset(primary_extruder_skirt_brim_line_width / 2));

        // generate brim within shield_brim
        storage.skirt_brim[extruder_nr].emplace_back();
        storage.skirt_brim[extruder_nr].back().closed_polygons.add(shield_brim);
        while (shield_brim.size() > 0)
        {
            shield_brim = shield_brim.offset(-primary_extruder_skirt_brim_line_width);
            storage.skirt_brim[extruder_nr].back().closed_polygons.add(shield_brim); // throw all polygons for the shileds onto one heap; because the brim lines are generated from both sides the order will not be important
        }
    }

    if (is_skirt)
    {
        if (has_ooze_shield)
        {
            brim_covered_area = brim_covered_area.unionPolygons(storage.oozeShield[0].offset(line_widths[extruder_nr] / 2));
        }
        if (has_draft_shield)
        {
            brim_covered_area = brim_covered_area.unionPolygons(storage.draft_protection_shield.offset(line_widths[extruder_nr] / 2));
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
            int reference_index = -1;
            Polygons* reference_polygons = nullptr;
            coord_t offset_from_reference;
            if (first)
            {
                reference_polygons = &reference_outline;
                offset_from_reference = line_widths[extruder_nr] / 2;
            }
            else
            {
                reference_index = storage.skirt_brim[extruder_nr].size() - 1;
                offset_from_reference = line_widths[extruder_nr];
            }
            constexpr bool external_only = false; // TODO is this correct?
            Offset extra_offset(reference_polygons, reference_index, external_only, offset_from_reference, bogus_total_offset, storage.skirt_brim[extruder_nr].size(), extruder_nr, is_last);
            
            storage.skirt_brim[extruder_nr].emplace_back();
            SkirtBrimLine& output_location = storage.skirt_brim[extruder_nr].back();
            coord_t added_length = generateOffset(extra_offset, covered_area, allowed_areas_per_extruder, output_location);

            if ( ! added_length)
            {
                logWarning("Couldn't satisfy minimum length constraint of extruder %i!\n", extruder_nr);
                break;
            }

            total_length[extra_offset.extruder_nr] += added_length;
            
            first = false;
        }
    }
}

void SkirtBrim::generateSupportBrim(const bool merge_with_model_skirtbrim)
{
    constexpr coord_t brim_area_minimum_hole_size_multiplier = 100;

    Scene& scene = Application::getInstance().current_slice->scene;
    const ExtruderTrain& support_infill_extruder = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const coord_t brim_line_width = support_infill_extruder.settings.get<coord_t>("skirt_brim_line_width") * support_infill_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    size_t line_count = support_infill_extruder.settings.get<size_t>("support_brim_line_count");
    const coord_t minimal_length = support_infill_extruder.settings.get<coord_t>("skirt_brim_minimal_length");
    if (!storage.support.generated || line_count <= 0 || storage.support.supportLayers.empty())
    {
        return;
    }

    const coord_t brim_width = brim_line_width * line_count;
    coord_t skirt_brim_length = 0;
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

    Polygons support_brim;

    coord_t offset_distance = brim_line_width / 2;
    for (size_t skirt_brim_number = 0; skirt_brim_number < line_count; skirt_brim_number++)
    {
        offset_distance -= brim_line_width;

        Polygons brim_line = support_outline.offset(offset_distance, ClipperLib::jtRound);

        //Remove small inner skirt and brim holes. Holes have a negative area, remove anything smaller then multiplier x extrusion "area"
        for (size_t n = 0; n < brim_line.size(); n++)
        {
            const double area = brim_line[n].area();
            if (area < 0 && area > -brim_line_width * brim_line_width * brim_area_minimum_hole_size_multiplier)
            {
                brim_line.remove(n--);
            }
        }

        support_brim.add(brim_line);

        const coord_t length = skirt_brim_length + support_brim.polygonLength();
        if (skirt_brim_number + 1 >= line_count && length > 0 && length < minimal_length) //Make brim or skirt have more lines when total length is too small.
        {
            line_count++;
        }
        if (brim_line.empty())
        { // the fist layer of support is fully filled with brim
            break;
        }
    }

    /* TODO
    if (support_brim.size())
    {
        if (merge_with_model_skirtbrim)
        {
            // to ensure that the skirt brim is printed from outside to inside, the support brim lines must
            // come before the skirt brim lines in the Polygon object so that the outermost skirt brim line
            // is at the back of the list
            support_brim.add(skirt_brim);
            skirt_brim = support_brim;
        }
        else
        {
            // OTOH, if we use a skirt instead of a brim for the polygon, the skirt line(s) should _always_ come first.
            skirt_brim.add(support_brim);
        }
    }
    */
}


}//namespace cura
