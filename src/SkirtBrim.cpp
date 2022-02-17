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
, extruders(Application::getInstance().current_slice->scene.extruders)
, extruder_count(extruders.size())
, extruder_is_used(storage.getExtrudersUsed())
{
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
        external_polys_only[extruder_nr] = extruder.settings.get<bool>("brim_outside_only");
        line_count[extruder_nr] = extruder.settings.get<int>(is_brim? "brim_line_count" : "skirt_line_count");
        gap[extruder_nr] = extruder.settings.get<coord_t>(is_brim? "brim_gap" : "skirt_gap");
    }
}

void SkirtBrim::generate()
{
    
    const bool has_ooze_shield = storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0;
    const bool has_draft_shield = storage.draft_protection_shield.size() > 0;

    
    
    std::vector<Offset> all_brim_offsets;

    const int skirt_brim_extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<int>("skirt_brim_extruder_nr");
    
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

    bool first = true;
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if ( ! extruder_is_used[extruder_nr]) continue;
        if (skirt_brim_extruder_nr >= 0 && extruder_nr != skirt_brim_extruder_nr)
        {
            continue; // only include offsets for brim extruder
        }
        if ( ! is_brim && ! first)
        { // Only generate primary skirt for first extruder, rest is handled via minimum length enforcement
            break;
        }
        first = false;

        storage.skirt_brim[extruder_nr].resize(line_count[extruder_nr] + 20); // 20 should be enough for the extra lines required for minimum length


        for (int line_idx = 0; line_idx < line_count[extruder_nr]; line_idx++)
        {
            coord_t offset = gap[extruder_nr] + line_widths[extruder_nr] / 2 + line_widths[extruder_nr] * line_idx;
            const bool is_last = line_idx == line_count[extruder_nr] - 1;
            all_brim_offsets.emplace_back(offset, line_widths[extruder_nr], gap[extruder_nr], line_idx, extruder_nr, is_last);
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
    // TODO: make allowed areas a bit smaller so that internal external-only brims don't overlap with model by half the line width

    bool brim_lines_can_be_cut = skirt_brim_extruder_nr < 0; // brims can interfere with each other.
    for (bool external_only : external_polys_only)
    {
        brim_lines_can_be_cut |= external_only;
    }

    bool covered_area_needs_update = brim_lines_can_be_cut || has_ooze_shield || has_draft_shield;
    
    for (size_t offset_idx = 0; offset_idx < all_brim_offsets.size(); offset_idx++)
    {
        const Offset& offset = all_brim_offsets[offset_idx];
        generateOffset(offset, starting_outlines, brim_lines_can_be_cut, covered_area_needs_update,
                       covered_area, allowed_areas_per_extruder, total_length);
        
        if (offset.is_last
            // v This was the last offset of this extruder, but the brim lines don't meet minimal length yet
            && total_length[offset.extruder_nr] < skirt_brim_minimal_length[offset.extruder_nr]
            && total_length[offset.extruder_nr] > 0u // No lines got added; we have no extrusion lines to build on
        )
        {
            offset.is_last = false;
            constexpr bool is_last = true;
            all_brim_offsets.emplace_back(offset.offset_value + offset.line_width, offset.line_width, offset.gap, offset.line_idx + 1, offset.extruder_nr, is_last);
            std::sort(all_brim_offsets.begin() + offset_idx + 1, all_brim_offsets.end(), OffsetSorter{}); // reorder remaining offsets
        }
    }
    
    // TODO list:
    
    // remove small open lines
    
    // ooze/draft shield brim
    generateShieldBrim(covered_area);
    
//     generate extra skirt for non-primary extruders
    // make extra skirt around outer brim of other material if this material has no room for more brim!
    
    // make skirt with minimal length algorithm instead of the core brim algorithms
    
    // robustness against when things are empty (brim lines, layer outlines, etc)
    
    // const correctness
    
    // documentation
    
    // frontend stuff
    
    // remove prime blobs from brim
    
    // simplify
    
}

void SkirtBrim::generateOffset(const Offset& offset, const std::vector<Polygons>& starting_outlines, const bool brim_lines_can_be_cut, const bool covered_area_needs_update, Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder, std::vector<coord_t>& total_length)
{
    constexpr bool indent_to_prevent_git_changes = true;
    if (indent_to_prevent_git_changes)
    {
        Polygons brim;
        Polygons newly_covered;
        if (external_polys_only[offset.extruder_nr])
        { // prevent unioning of external polys enclosed by other parts, e.g. a small part inside a hollow cylinder.
            if (offset.line_idx == 0)
            {
                for (ConstPolygonRef poly : starting_outlines[offset.extruder_nr])
                {
                    brim.add(poly.offset(offset.offset_value, ClipperLib::jtRound));
                    if (covered_area_needs_update)
                    {
                        newly_covered.add(poly.offset(offset.offset_value + offset.line_width / 2, ClipperLib::jtRound));
                        Polygon reference = poly;
                        reference.reverse();
                        newly_covered.add(reference); // don't remove area inside external polygon
                    }
                }
            }
            else
            {
                
                Polygons polylines = storage.skirt_brim[offset.extruder_nr][offset.line_idx - 1].closed_polygons;
                polylines.toPolylines();
                polylines.add(storage.skirt_brim[offset.extruder_nr][offset.line_idx - 1].open_polylines);
                brim.add(polylines.offsetPolyLine(offset.line_width, ClipperLib::jtRound));
                if (covered_area_needs_update) newly_covered.add(polylines.offsetPolyLine(offset.line_width * 3 / 2, ClipperLib::jtRound));
            }
        }
        else
        {
            brim = starting_outlines[offset.extruder_nr].offset(offset.offset_value, ClipperLib::jtRound);
            if (covered_area_needs_update) newly_covered = starting_outlines[offset.extruder_nr].offset(offset.offset_value + offset.line_width / 2, ClipperLib::jtRound);
        }

        if (brim_lines_can_be_cut)
        {
            brim.simplify();
            brim.toPolylines();
            Polygons brim_lines = allowed_areas_per_extruder[offset.extruder_nr].intersectionPolyLines(brim, false);
            total_length[offset.extruder_nr] += brim_lines.polyLineLength();

            const coord_t max_stitch_distance = line_widths[offset.extruder_nr];
            PolylineStitcher<Polygons, Polygon, Point>::stitch(brim_lines, storage.skirt_brim[offset.extruder_nr][offset.line_idx].open_polylines, storage.skirt_brim[offset.extruder_nr][offset.line_idx].closed_polygons, max_stitch_distance);
        }
        else
        {
            storage.skirt_brim[offset.extruder_nr][offset.line_idx].closed_polygons = brim;
            total_length[offset.extruder_nr] += brim.polygonLength();
        }
        
        if (covered_area_needs_update)
        {
            for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
            {
                if ( ! extruder_is_used[extruder_nr]) continue;
                covered_area = covered_area.unionPolygons(newly_covered.unionPolygons());
                allowed_areas_per_extruder[extruder_nr] = allowed_areas_per_extruder[extruder_nr].difference(covered_area);
            }
        }
    }
}

Polygons SkirtBrim::getFirstLayerOutline(const int extruder_nr)
{
    Polygons first_layer_outline;
    Settings& global_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    int skirt_brim_extruder_nr = global_settings.get<int>("skirt_brim_extruder_nr");
    assert( ! (skirt_brim_extruder_nr == -1 && extruder_nr == -1) && "We should only request the outlines of all layers when the brim is being generated for only one material");
    if (skirt_brim_extruder_nr == -1)
    {
        skirt_brim_extruder_nr = extruder_nr;
    }
    const int primary_line_count = line_count[skirt_brim_extruder_nr];
    const bool external_only = is_skirt || external_polys_only[skirt_brim_extruder_nr]; //Whether to include holes or not. Skirt doesn't have any holes.
    const LayerIndex layer_nr = 0;
    if (is_skirt)
    {
        constexpr bool include_support = true;
        constexpr bool include_prime_tower = true;
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_only, extruder_nr);
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
                const coord_t primary_extruder_skirt_brim_line_width = line_widths[skirt_brim_extruder_nr];
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
/*
coord_t SkirtBrim::generatePrimarySkirtBrimLines(const coord_t start_distance, size_t& primary_line_count, const coord_t primary_extruder_minimal_length, const Polygons& first_layer_outline, Polygons& skirt_brim_primary_extruder)
{
    const Settings& adhesion_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("skirt_brim_extruder_nr").settings;
    const coord_t primary_extruder_skirt_brim_line_width = adhesion_settings.get<coord_t>("skirt_brim_line_width") * adhesion_settings.get<Ratio>("initial_layer_line_width_factor");
    coord_t offset_distance = start_distance - primary_extruder_skirt_brim_line_width / 2;
    for (unsigned int skirt_brim_number = 0; skirt_brim_number < primary_line_count; skirt_brim_number++)
    {
        offset_distance += primary_extruder_skirt_brim_line_width;

        Polygons outer_skirt_brim_line = first_layer_outline.offset(offset_distance, ClipperLib::jtRound);

        //Remove small inner skirt and brim holes. Holes have a negative area, remove anything smaller then 100x extrusion "area"
        for (unsigned int n = 0; n < outer_skirt_brim_line.size(); n++)
        {
            double area = outer_skirt_brim_line[n].area();
            if (area < 0 && area > -primary_extruder_skirt_brim_line_width * primary_extruder_skirt_brim_line_width * 100)
            {
                outer_skirt_brim_line.remove(n--);
            }
        }

        skirt_brim_primary_extruder.add(outer_skirt_brim_line);

        const coord_t length = skirt_brim_primary_extruder.polygonLength();
        if (skirt_brim_number + 1 >= primary_line_count && length > 0 && length < primary_extruder_minimal_length) //Make brim or skirt have more lines when total length is too small.
        {
            primary_line_count++;
        }
    }
    return offset_distance;
}

void SkirtBrim::generate(Polygons first_layer_outline, const coord_t start_distance, size_t primary_line_count, const bool allow_helpers)
{
    const bool is_skirt = start_distance > 0;
    Scene& scene = Application::getInstance().current_slice->scene;
    const size_t skirt_brim_extruder_nr = scene.current_mesh_group->settings.get<ExtruderTrain&>("skirt_brim_extruder_nr").extruder_nr;
    const Settings& adhesion_settings = scene.extruders[skirt_brim_extruder_nr].settings;
    const coord_t primary_extruder_skirt_brim_line_width = adhesion_settings.get<coord_t>("skirt_brim_line_width") * adhesion_settings.get<Ratio>("initial_layer_line_width_factor");
    const coord_t primary_extruder_minimal_length = adhesion_settings.get<coord_t>("skirt_brim_minimal_length");

    Polygons& skirt_brim_primary_extruder = storage.skirt_brim[skirt_brim_extruder_nr];

    const bool has_ooze_shield = allow_helpers && storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0;
    const bool has_draft_shield = allow_helpers && storage.draft_protection_shield.size() > 0;

    coord_t gap;
    if (is_skirt && (has_ooze_shield || has_draft_shield))
    { // make sure we don't generate skirt through draft / ooze shield
        first_layer_outline = first_layer_outline.offset(start_distance - primary_extruder_skirt_brim_line_width / 2, ClipperLib::jtRound).unionPolygons(storage.draft_protection_shield);
        if (has_ooze_shield)
        {
            first_layer_outline = first_layer_outline.unionPolygons(storage.oozeShield[0]);
        }
        first_layer_outline = first_layer_outline.approxConvexHull();
        gap = primary_extruder_skirt_brim_line_width / 2;
    }
    else
    {
        gap = start_distance;
    }

    coord_t offset_distance = generatePrimarySkirtBrimLines(gap, primary_line_count, primary_extruder_minimal_length, first_layer_outline, skirt_brim_primary_extruder);

    // Skirt needs to be 'locked' first, otherwise the optimizer can change to order, which can cause undesirable outcomes w.r.t combo w. support-brim or prime-tower brim.
    // If this method is called multiple times, the max order shouldn't reset to 0, so the maximum is taken.
    storage.skirt_brim_max_locked_part_order[skirt_brim_extruder_nr] = std::max(is_skirt ? primary_line_count : 0, storage.skirt_brim_max_locked_part_order[skirt_brim_extruder_nr]);

    // handle support-brim
    const ExtruderTrain& support_infill_extruder = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    if (allow_helpers && support_infill_extruder.settings.get<bool>("support_brim_enable"))
    {
        const bool merge_with_model_skirtbrim = !is_skirt;
        generateSupportBrim(merge_with_model_skirtbrim);
    }
*/


void SkirtBrim::generateShieldBrim(Polygons& brim_covered_area)
{
    const bool has_ooze_shield = storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0;
    const bool has_draft_shield = storage.draft_protection_shield.size() > 0;

    // generate brim for ooze shield and draft shield
    if (!is_skirt && (has_ooze_shield || has_draft_shield))
    {
        int extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<int>("skirt_brim_extruder_nr");
        if (extruder_nr < 0)
        { // the shields are always printed with all extruders, so it doesn't really matter with which extruder we print the brim on the first layer
            extruder_nr = storage.getExtrudersUsed()[0];
        }
        const Settings& adhesion_settings = Application::getInstance().current_slice->scene.extruders[extruder_nr].settings;

        const coord_t primary_extruder_skirt_brim_line_width = adhesion_settings.get<coord_t>("skirt_brim_line_width") * adhesion_settings.get<Ratio>("initial_layer_line_width_factor");

        int primary_line_count = adhesion_settings.get<int>("brim_line_count");

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

        // generate brim within shield_brim
        storage.skirt_brim[extruder_nr].emplace_back();
        storage.skirt_brim[extruder_nr].back().closed_polygons.add(shield_brim);
        while (shield_brim.size() > 0)
        {
            shield_brim = shield_brim.offset(-primary_extruder_skirt_brim_line_width);
            storage.skirt_brim[extruder_nr].back().closed_polygons.add(shield_brim); // throw all polygons for the shileds onto one heap; because the brim lines are generated from both sides the order will not be important
        }
    }
}

/*
        // update parameters to generate secondary skirt around
        first_layer_outline = outer_primary_brim;
        if (has_draft_shield)
        {
            first_layer_outline = first_layer_outline.unionPolygons(storage.draft_protection_shield);
        }
        if (has_ooze_shield)
        {
            first_layer_outline = first_layer_outline.unionPolygons(storage.oozeShield[0]);
        }

        offset_distance = 0;
    }

    if (first_layer_outline.polygonLength() > 0)
    { // process other extruders' brim/skirt (as one brim line around the old brim)
        int last_width = primary_extruder_skirt_brim_line_width;
        std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
        for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
        {
            if (extruder_nr == skirt_brim_extruder_nr || !extruder_is_used[extruder_nr])
            {
                continue;
            }
            const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
            const coord_t width = train.settings.get<coord_t>("skirt_brim_line_width") * train.settings.get<Ratio>("initial_layer_line_width_factor");
            const coord_t minimal_length = train.settings.get<coord_t>("skirt_brim_minimal_length");
            offset_distance += last_width / 2 + width/2;
            last_width = width;
            while (storage.skirt_brim[extruder_nr].polygonLength() < minimal_length)
            {
                storage.skirt_brim[extruder_nr].add(first_layer_outline.offset(offset_distance, ClipperLib::jtRound));
                offset_distance += width;
            }
        }
    }
}
*/

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
