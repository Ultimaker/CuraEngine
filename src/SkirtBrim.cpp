/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "SkirtBrim.h"
#include "support.h"

namespace cura 
{

void generateSkirtBrim(SliceDataStorage& storage, int distance, int count, int minLength)
{
    if (count == 0) return;
    
    bool externalOnly = (distance > 0); // whether to include holes or not

    const int primary_extruder = storage.getSettingAsIndex("adhesion_extruder_nr");
    const int primary_extruder_skirt_line_width = storage.meshgroup->getExtruderTrain(primary_extruder)->getSettingInMicrons("skirt_line_width");

    Polygons& skirt_primary_extruder = storage.skirt_brim[primary_extruder];
    
    bool get_convex_hull = count == 1 && distance > 0;
    
    Polygons first_layer_outline = storage.getLayerOutlines(0, true, externalOnly);
    
    std::vector<Polygons> skirts;
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        const int offsetDistance = distance + primary_extruder_skirt_line_width * skirtNr + primary_extruder_skirt_line_width / 2;

        skirts.emplace_back(first_layer_outline.offset(offsetDistance, ClipperLib::jtRound));
        Polygons& skirt_polygons = skirts.back();
        
        //Remove small inner skirt holes. Holes have a negative area, remove anything smaller then 100x extrusion "area"
        for(unsigned int n=0; n<skirt_polygons.size(); n++)
        {
            double area = skirt_polygons[n].area();
            if (area < 0 && area > -primary_extruder_skirt_line_width * primary_extruder_skirt_line_width * 100)
            {
                skirt_polygons.remove(n--);
            }
        }
    
        if (get_convex_hull)
        {
            skirt_polygons = skirt_polygons.approxConvexHull();
        }

        skirt_primary_extruder.add(skirt_polygons);

        int length = skirt_primary_extruder.polygonLength();
        if (skirtNr + 1 >= count && length > 0 && length < minLength) // make brim have more lines when total length is too small
            count++;
    }
    
    { // process other extruders' brim/skirt (as one brim line around the old brim)
        int offset_distance = 0;
        int last_width = primary_extruder_skirt_line_width;
        for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
        {
            if (extruder == primary_extruder) { continue; }
            int width = storage.meshgroup->getExtruderTrain(extruder)->getSettingInMicrons("skirt_line_width");
            offset_distance += last_width / 2 + width/2;
            last_width = width;
            while (storage.skirt_brim[extruder].polygonLength() < minLength)
            {
                storage.skirt_brim[extruder].add(skirts.back().offset(offset_distance, ClipperLib::jtRound));
                offset_distance += width;
            }
        }
    }
}

}//namespace cura
