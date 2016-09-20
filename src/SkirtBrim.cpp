/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "SkirtBrim.h"
#include "support.h"

namespace cura 
{

void SkirtBrim::generate(SliceDataStorage& storage, int start_distance, unsigned int primary_line_count, bool outside_only)
{
    bool is_skirt = start_distance > 0;
    bool external_only = is_skirt; // whether to include holes or not

    const int adhesion_extruder_nr = storage.getSettingAsIndex("adhesion_extruder_nr");
    const ExtruderTrain* adhesion_extruder = storage.meshgroup->getExtruderTrain(adhesion_extruder_nr);
    const int primary_extruder_skirt_brim_line_width = adhesion_extruder->getSettingInMicrons("skirt_brim_line_width");
    const int64_t primary_extruder_minimal_length = adhesion_extruder->getSettingInMicrons("skirt_brim_minimal_length");

    Polygons& skirt_brim_primary_extruder = storage.skirt_brim[adhesion_extruder_nr];

    Polygons first_layer_outline;
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
        first_layer_outline.add(storage.primeTower.ground_poly); // don't remove parts of the prime tower, but make a brim for it
        if (outside_only)
        {
            first_layer_outline = first_layer_outline.removeEmptyHoles();
        }
        if (storage.support.generated && primary_line_count > 0)
        { // remove model-brim from support
            const Polygons& model_brim_covered_area = first_layer_outline.offset(primary_extruder_skirt_brim_line_width * (primary_line_count + primary_line_count % 2)); // always leave a gap of an even number of brim lines, so that it fits if it's generating brim from both sides
            SupportLayer& support_layer = storage.support.supportLayers[0];
            support_layer.supportAreas = support_layer.supportAreas.difference(model_brim_covered_area);
            first_layer_outline.add(support_layer.supportAreas);
            first_layer_outline.add(support_layer.skin);
        }
    }

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
