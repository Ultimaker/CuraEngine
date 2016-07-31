/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "SkirtBrim.h"
#include "support.h"

namespace cura 
{

void generateSkirtBrim(SliceDataStorage& storage, int start_distance, unsigned int count, int minLength, bool outside_only)
{
    if (count == 0) return;
    
    bool externalOnly = (start_distance > 0); // whether to include holes or not

    const int primary_extruder = storage.getSettingAsIndex("adhesion_extruder_nr");
    const int primary_extruder_skirt_brim_line_width = storage.meshgroup->getExtruderTrain(primary_extruder)->getSettingInMicrons("skirt_brim_line_width");

    Polygons& skirt_brim_primary_extruder = storage.skirt_brim[primary_extruder];
    
    bool is_skirt = count == 1 && start_distance > 0;
    
    Polygons first_layer_outline;
    const int layer_nr = 0;
    if (is_skirt)
    {
        const bool include_helper_parts = true;
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_helper_parts, externalOnly);
        first_layer_outline = first_layer_outline.approxConvexHull();
    }
    else
    { // don't add brim _around_ support, but underneath it by removing support where there's brim
        const bool include_helper_parts = false;
        first_layer_outline = storage.getLayerOutlines(layer_nr, include_helper_parts, externalOnly);
        first_layer_outline.add(storage.primeTower.ground_poly); // don't remove parts of the prime tower, but make a brim for it
        if (outside_only)
        {
            first_layer_outline = first_layer_outline.getOutsidePolygons();
        }
    }

    Polygons outer_skirt_brim_line_primary_extruder;
    for (unsigned int skirt_brim_number = 0; skirt_brim_number < count; skirt_brim_number++)
    {
        const int offsetDistance = start_distance + primary_extruder_skirt_brim_line_width * skirt_brim_number + primary_extruder_skirt_brim_line_width / 2;

        outer_skirt_brim_line_primary_extruder = first_layer_outline.offset(offsetDistance, ClipperLib::jtRound);

        //Remove small inner skirt and brim holes. Holes have a negative area, remove anything smaller then 100x extrusion "area"
        for (unsigned int n = 0; n < outer_skirt_brim_line_primary_extruder.size(); n++)
        {
            double area = outer_skirt_brim_line_primary_extruder[n].area();
            if (area < 0 && area > -primary_extruder_skirt_brim_line_width * primary_extruder_skirt_brim_line_width * 100)
            {
                outer_skirt_brim_line_primary_extruder.remove(n--);
            }
        }

        skirt_brim_primary_extruder.add(outer_skirt_brim_line_primary_extruder);

        int length = skirt_brim_primary_extruder.polygonLength();
        if (skirt_brim_number + 1 >= count && length > 0 && length < minLength) //Make brim or skirt have more lines when total length is too small.
        {
            count++;
        }
    }
    
    { // process other extruders' brim/skirt (as one brim line around the old brim)
        int offset_distance = 0;
        int last_width = primary_extruder_skirt_brim_line_width;
        for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
        {
            if (extruder == primary_extruder || !storage.meshgroup->getExtruderTrain(extruder)->getIsUsed())
            {
                continue;
            }
            int width = storage.meshgroup->getExtruderTrain(extruder)->getSettingInMicrons("skirt_brim_line_width");
            offset_distance += last_width / 2 + width/2;
            last_width = width;
            while (storage.skirt_brim[extruder].polygonLength() < minLength)
            {
                storage.skirt_brim[extruder].add(outer_skirt_brim_line_primary_extruder.offset(offset_distance, ClipperLib::jtRound));
                offset_distance += width;
            }
        }
    }

    if (!is_skirt && storage.support.generated)
    { // remove brim from support
        for (int extruder_nr = storage.meshgroup->getExtruderCount() - 1; extruder_nr >= 0; extruder_nr--)
        {
            Polygons& last_brim = storage.skirt_brim[extruder_nr];
            if (last_brim.size() > 0)
            {
                int brim_line_width = storage.meshgroup->getExtruderTrain(extruder_nr)->getSettingInMicrons("skirt_brim_line_width");
                SupportLayer& support_layer = storage.support.supportLayers[0];
                Polygons area_covered_by_brim = last_brim.offset(brim_line_width / 2);
                support_layer.roofs = support_layer.roofs.difference(area_covered_by_brim);
                support_layer.supportAreas = support_layer.supportAreas.difference(area_covered_by_brim);
                break;
            }
        }
    }
}

}//namespace cura
