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

void SkirtBrim::generate(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    EPlatformAdhesion adhesion_type = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type");

    const int prime_tower_brim_extruder = storage.primeTower.extruder_order[0]; // The extruder with which the outer ring of the prime tower is printed

    const bool is_brim = adhesion_type == EPlatformAdhesion::BRIM;
    
    struct Offset
    {
        Offset(const Polygons& reference_polygons, const coord_t offset_value, coord_t line_width, coord_t gap, const coord_t line_idx, const int extruder_nr)
        : reference_polygons(&reference_polygons)
        , offset_value(offset_value)
        , line_width(line_width)
        , gap(gap)
        , line_idx(line_idx)
        , extruder_nr(extruder_nr)
        {}
        const Polygons* reference_polygons;
        coord_t offset_value;
        coord_t line_width;
        coord_t gap;
        coord_t line_idx;
        int extruder_nr;
    };
    
    const std::vector<ExtruderTrain>& extruders = Application::getInstance().current_slice->scene.extruders;
    const int extruder_count = extruders.size();
    
    std::vector<coord_t> line_widths(extruder_count);
    std::vector<bool> external_polys_only(extruder_count);
    
    std::vector<Offset> all_brim_offsets;

    const int skirt_brim_extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<int>("skirt_brim_extruder_nr");
    
    
    std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    std::vector<Polygons> starting_outlines(extruder_count);
    constexpr LayerIndex layer_nr = 0;
    const bool include_support = false; // TODO: extruder_nr == mesh_group_settings.get<int>("support_infill_extruder_nr");
    const bool include_prime_tower = mesh_group_settings.get<bool>("prime_tower_brim_enable");
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if ( ! extruder_is_used[extruder_nr]) continue;
        if (skirt_brim_extruder_nr >= 0 && extruder_nr != skirt_brim_extruder_nr)
        {
            continue; // only include offsets for brim extruder
        }


        
        const ExtruderTrain& extruder = extruders[extruder_nr];
        coord_t line_width = extruder.settings.get<coord_t>("skirt_brim_line_width") * extruder.settings.get<Ratio>("initial_layer_line_width_factor");
        int line_count = extruder.settings.get<int>(is_brim? "brim_line_count" : "skirt_line_count");
        coord_t gap = extruder.settings.get<coord_t>(is_brim? "brim_gap" : "skirt_gap");
        
        external_polys_only[extruder_nr] = ! is_brim || extruder.settings.get<bool>("brim_outside_only");

        line_widths[extruder_nr] = line_width;

        storage.skirt_brim[extruder_nr].resize(line_count + 20); // 20 should be enough for the extra lines required for minimum length

        const bool prime_tower_here = include_prime_tower && (extruder_nr == prime_tower_brim_extruder);
        const int include_polys_from_extruder = (skirt_brim_extruder_nr < 0)? extruder_nr : -1; // include polys from all extruders if we print one brim for all materials
        starting_outlines[extruder_nr] = storage.getLayerOutlines(layer_nr, include_support, prime_tower_here, external_polys_only[extruder_nr], include_polys_from_extruder);

        for (int line_idx = 0; line_idx < line_count; line_idx++)
        {
            coord_t offset = gap + line_width / 2 + line_width * line_idx;
            all_brim_offsets.emplace_back(starting_outlines[extruder_nr], offset, line_width, gap, line_idx, extruder_nr);
        }
    }

    std::sort(all_brim_offsets.begin(), all_brim_offsets.end(),
              [](const Offset& a, const Offset& b)
              {
                  return a.offset_value + a.extruder_nr
                    < b.offset_value + b.extruder_nr; // add extruder_nr so that it's more stable when both extruders have the same offset settings
            } );
    
    coord_t max_offset = 0;
    for (const Offset& offset : all_brim_offsets)
    {
        max_offset = std::max(max_offset, offset.offset_value);
    }
    
    Polygons covered_area = storage.getLayerOutlines(layer_nr, include_support, /*include_prime_tower*/ true, /*external_polys_only*/ false);
    
    Polygons machine_area = storage.getMachineBorder(/*adhesion_offset*/ false);
    Polygons allowed_areas = machine_area.difference(covered_area);
    // TODO: make allowed areas a bit smaller so that internal external-only brims don't overlap with model by half the line width

    bool brim_lines_can_be_cut = skirt_brim_extruder_nr < 0; // brims can interfere with each other.
    for (bool external_only : external_polys_only)
    {
        brim_lines_can_be_cut |= external_only;
    }

    for (const Offset& offset : all_brim_offsets)
    {
        Polygons brim;
        Polygons newly_covered;
        if (external_polys_only[offset.extruder_nr])
        { // prevent unioning of external polys enclosed by other parts, e.g. a small part inside a hollow cylinder.
            if (offset.line_idx == 0)
            {
                for (ConstPolygonRef poly : *offset.reference_polygons)
                {
                    brim.add(poly.offset(offset.offset_value, ClipperLib::jtRound));
                    if (brim_lines_can_be_cut)
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
                if (brim_lines_can_be_cut) newly_covered.add(polylines.offsetPolyLine(offset.line_width * 3 / 2, ClipperLib::jtRound));
            }
        }
        else
        {
            brim = offset.reference_polygons->offset(offset.offset_value, ClipperLib::jtRound);
            if (brim_lines_can_be_cut) newly_covered = offset.reference_polygons->offset(offset.offset_value + offset.line_width / 2, ClipperLib::jtRound);
        }

        if (brim_lines_can_be_cut)
        {
            brim.simplify();
            brim.toPolylines();
            Polygons brim_lines = allowed_areas.intersectionPolyLines(brim, false);
            const coord_t max_stitch_distance = line_widths[offset.extruder_nr];
            PolylineStitcher<Polygons, Polygon, Point>::stitch(brim_lines, storage.skirt_brim[offset.extruder_nr][offset.line_idx].open_polylines, storage.skirt_brim[offset.extruder_nr][offset.line_idx].closed_polygons, max_stitch_distance);
        }
        else
        {
            storage.skirt_brim[offset.extruder_nr][offset.line_idx].closed_polygons = brim;
        }

        if (brim_lines_can_be_cut)
        {
            allowed_areas = allowed_areas.difference(newly_covered.unionPolygons());
        }
    }
    
    // support brim
    
    // ooze/draft shield brim
    
//     ensureMinimalLength();

//     generate extra skirt for non-primary extruders
    // make extra skirt around outer brim of other material if this material has no room for more brim!

    // simplify
    
}
/*
void SkirtBrim::getFirstLayerOutline(SliceDataStorage& storage, const size_t primary_line_count, const bool is_skirt, Polygons& first_layer_outline)
{
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("skirt_brim_extruder_nr");
    const ExtruderTrain& support_infill_extruder = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const bool external_only = is_skirt || train.settings.get<bool>("brim_outside_only"); //Whether to include holes or not. Skirt doesn't have any holes.
    const LayerIndex layer_nr = 0;
    if (is_skirt)
    {
        constexpr bool include_support = true;
        constexpr bool include_prime_tower = true;
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_only);
        first_layer_outline = first_layer_outline.approxConvexHull();
    }
    else
    { // add brim underneath support by removing support where there's brim around the model
        constexpr bool include_support = false; //Include manually below.
        constexpr bool include_prime_tower = false; //Include manually below.
        constexpr bool external_outlines_only = false; //Remove manually below.
        constexpr bool for_brim = true;
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_outlines_only, for_brim);
        first_layer_outline = first_layer_outline.unionPolygons(); //To guard against overlapping outlines, which would produce holes according to the even-odd rule.
        Polygons first_layer_empty_holes;
        if (external_only)
        {
            first_layer_empty_holes = first_layer_outline.getEmptyHoles();
            first_layer_outline = first_layer_outline.removeEmptyHoles();
        }
        if (storage.support.generated && primary_line_count > 0 && !storage.support.supportLayers.empty())
        { // remove model-brim from support
            SupportLayer& support_layer = storage.support.supportLayers[0];
            if (support_infill_extruder.settings.get<bool>("brim_replaces_support"))
            {
                // avoid gap in the middle
                //    V
                //  +---+     +----+
                //  |+-+|     |+--+|
                //  || ||     ||[]|| > expand to fit an extra brim line
                //  |+-+|     |+--+|
                //  +---+     +----+
                const coord_t primary_extruder_skirt_brim_line_width = train.settings.get<coord_t>("skirt_brim_line_width") * train.settings.get<Ratio>("initial_layer_line_width_factor");
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
        if (storage.primeTower.enabled && !train.settings.get<bool>("prime_tower_brim_enable"))
        {
            first_layer_outline.add(storage.primeTower.outer_poly_first_layer); // don't remove parts of the prime tower, but make a brim for it
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
}

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

void SkirtBrim::generate(SliceDataStorage& storage, Polygons first_layer_outline, const coord_t start_distance, size_t primary_line_count, const bool allow_helpers)
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
        generateSupportBrim(storage, merge_with_model_skirtbrim);
    }

    // generate brim for ooze shield and draft shield
    if (!is_skirt && (has_ooze_shield || has_draft_shield))
    {
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
        const Polygons outer_primary_brim = first_layer_outline.offset(offset_distance, ClipperLib::jtRound);
        shield_brim = shield_brim.difference(outer_primary_brim.offset(primary_extruder_skirt_brim_line_width));

        // generate brim within shield_brim
        skirt_brim_primary_extruder.add(shield_brim);
        while (shield_brim.size() > 0)
        {
            shield_brim = shield_brim.offset(-primary_extruder_skirt_brim_line_width);
            skirt_brim_primary_extruder.add(shield_brim);
        }

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

void SkirtBrim::generateSupportBrim(SliceDataStorage& storage, const bool merge_with_model_skirtbrim)
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
