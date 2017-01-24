//Copyright (C) 2013 David Braam
//Copyright (c) 2016 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SkirtBrim.h"
#include "support.h"

namespace cura 
{

void SkirtBrim::getFirstLayerOutline(SliceDataStorage& storage, const unsigned int primary_line_count, const int primary_extruder_skirt_brim_line_width, const bool is_skirt, const bool outside_only, Polygons& first_layer_outline)
{
    bool external_only = is_skirt; // whether to include holes or not
    const int layer_nr = 0;
    if (is_skirt)
    {
        const bool include_helper_parts = true;
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_helper_parts, external_only);
        first_layer_outline = first_layer_outline.approxConvexHull();
    }
    else
    { // add brim underneath support by removing support where there's brim around the model
        const bool include_helper_parts = false; // include manually below
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_helper_parts, external_only);
        first_layer_outline = first_layer_outline.unionPolygons(); //To guard against overlapping outlines, which would produce holes according to the even-odd rule.
        Polygons first_layer_empty_holes;
        if (outside_only)
        {
            first_layer_empty_holes = first_layer_outline.getEmptyHoles();
            first_layer_outline = first_layer_outline.removeEmptyHoles();
        }
        if (storage.support.generated && primary_line_count > 0)
        { // remove model-brim from support
            // avoid gap in the middle
            //    V
            //  +---+     +----+
            //  |+-+|     |+--+|
            //  || ||     ||[]|| > expand to fit an extra brim line
            //  |+-+|     |+--+|
            //  +---+     +----+ 
            Polygons model_brim_covered_area = first_layer_outline.offset(primary_extruder_skirt_brim_line_width * (primary_line_count + primary_line_count % 2), ClipperLib::jtRound); // always leave a gap of an even number of brim lines, so that it fits if it's generating brim from both sides
            if (outside_only)
            { // don't remove support within empty holes where no brim is generated.
                model_brim_covered_area.add(first_layer_empty_holes);
            }
            SupportLayer& support_layer = storage.support.supportLayers[0];
            support_layer.supportAreas = support_layer.supportAreas.difference(model_brim_covered_area);
            first_layer_outline.add(support_layer.supportAreas);
            first_layer_outline.add(support_layer.skin);
        }
        if (storage.primeTower.enabled)
        {
            first_layer_outline.add(storage.primeTower.ground_poly); // don't remove parts of the prime tower, but make a brim for it
        }
    }
    constexpr int join_distance = 20;
    first_layer_outline = first_layer_outline.offset(join_distance).offset(-join_distance); // merge adjacent models into single polygon
    constexpr int smallest_line_length = 200;
    constexpr int largest_error_of_removed_point = 50;
    first_layer_outline.simplify(smallest_line_length, largest_error_of_removed_point); // simplify for faster processing of the brim lines
    if (first_layer_outline.size() == 0)
    {
        logError("Couldn't generate skirt / brim! No polygons on first layer.");
    }
}

int SkirtBrim::generatePrimarySkirtBrimLines(SliceDataStorage& storage, int start_distance, unsigned int primary_line_count, const int primary_extruder_skirt_brim_line_width, const int64_t primary_extruder_minimal_length, const Polygons& first_layer_outline, Polygons& skirt_brim_primary_extruder)
{

    int offset_distance = start_distance - primary_extruder_skirt_brim_line_width / 2;
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

        int length = skirt_brim_primary_extruder.polygonLength();
        if (skirt_brim_number + 1 >= primary_line_count && length > 0 && length < primary_extruder_minimal_length) //Make brim or skirt have more lines when total length is too small.
        {
            primary_line_count++;
        }
    }
    return offset_distance;
}

void SkirtBrim::generate(SliceDataStorage& storage, int start_distance, unsigned int primary_line_count, bool outside_only)
{
    const bool is_skirt = start_distance > 0;

    const int adhesion_extruder_nr = storage.getSettingAsIndex("adhesion_extruder_nr");
    const ExtruderTrain* adhesion_extruder = storage.meshgroup->getExtruderTrain(adhesion_extruder_nr);
    const int primary_extruder_skirt_brim_line_width = adhesion_extruder->getSettingInMicrons("skirt_brim_line_width");
    const int64_t primary_extruder_minimal_length = adhesion_extruder->getSettingInMicrons("skirt_brim_minimal_length");

    Polygons& skirt_brim_primary_extruder = storage.skirt_brim[adhesion_extruder_nr];

    Polygons first_layer_outline;
    getFirstLayerOutline(storage, primary_line_count, primary_extruder_skirt_brim_line_width, is_skirt, outside_only, first_layer_outline);

    const bool has_ooze_shield = storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0;
    const bool has_draft_shield = storage.draft_protection_shield.size() > 0;

    if (is_skirt && (has_ooze_shield || has_draft_shield))
    { // make sure we don't generate skirt through draft / ooze shield
        first_layer_outline = first_layer_outline.offset(start_distance - primary_extruder_skirt_brim_line_width / 2, ClipperLib::jtRound).unionPolygons(storage.draft_protection_shield);
        if (has_ooze_shield)
        {
            first_layer_outline = first_layer_outline.unionPolygons(storage.oozeShield[0]);
        }
        first_layer_outline = first_layer_outline.approxConvexHull();
        start_distance = primary_extruder_skirt_brim_line_width / 2;
    }

    int offset_distance = generatePrimarySkirtBrimLines(storage, start_distance, primary_line_count, primary_extruder_skirt_brim_line_width, primary_extruder_minimal_length, first_layer_outline, skirt_brim_primary_extruder);


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
        const int64_t primary_skirt_brim_width = (primary_line_count + primary_line_count % 2) * primary_extruder_skirt_brim_line_width; // always use an even number, because we will fil the area from both sides

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

    { // process other extruders' brim/skirt (as one brim line around the old brim)
        int last_width = primary_extruder_skirt_brim_line_width;
        for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
        {
            if (extruder == adhesion_extruder_nr || !storage.meshgroup->getExtruderTrain(extruder)->getIsUsed())
            {
                continue;
            }
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder);
            const int width = train->getSettingInMicrons("skirt_brim_line_width");
            const int64_t minimal_length = train->getSettingInMicrons("skirt_brim_minimal_length");
            offset_distance += last_width / 2 + width/2;
            last_width = width;
            while (storage.skirt_brim[extruder].polygonLength() < minimal_length)
            {
                storage.skirt_brim[extruder].add(first_layer_outline.offset(offset_distance, ClipperLib::jtRound));
                offset_distance += width;
            }
        }
    }
}

}//namespace cura
