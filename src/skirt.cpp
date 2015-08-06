/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skirt.h"
#include "support.h"

#include <queue> 

namespace cura 
{

void generateSkirt(SliceDataStorage& storage, int distance, int count, int minLength)
{
    if (count == 0) return;
    
    bool externalOnly = (distance > 0); // whether to include holes or not
    
    int primary_extruder = 0; // TODO allow for other extruder to be primary
    int primary_extrusion_width = storage.meshgroup->getExtruderTrain(primary_extruder)->getSettingInMicrons("skirt_line_width");
    
    Polygons& skirt_primary_extruder = storage.skirt[primary_extruder];
    
    Polygons first_layer_outline;
    {// add support polygons
        if (storage.support.generated) 
            first_layer_outline = storage.support.supportLayers[0].supportAreas
                    .unionPolygons(storage.support.supportLayers[0].roofs);
        { // get support polygons
            for(SliceMeshStorage& mesh : storage.meshes)
            {
                if (mesh.layers.size() < 1) continue;
                SliceLayer* layer = &mesh.layers[0];
                for(unsigned int i=0; i<layer->parts.size(); i++)        
                    first_layer_outline = first_layer_outline.difference(layer->parts[i].outline);
            }
            
            // expand and contract to smooth the final polygon
            if (count == 1 && distance > 0)
            {
                int dist = primary_extrusion_width * 5;
                first_layer_outline = first_layer_outline.offset(dist).offset(-dist);
            }
        }
    }
    { // add prime tower and part outlines
        first_layer_outline = first_layer_outline.unionPolygons(storage.primeTower.ground_poly);
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.layers.size() < 1) continue;
            for(SliceLayerPart& part : mesh.layers[0].parts)
            {
                if (externalOnly)
                {
                    Polygons p;
                    p.add(part.outline.outerPolygon());
                    first_layer_outline = first_layer_outline.unionPolygons(p);
                }
                else
                {
                    first_layer_outline = first_layer_outline.unionPolygons(part.outline);
                }
            }
        }
    }
    
    Polygons skirt_polygons;
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        int offsetDistance = distance + primary_extrusion_width * skirtNr + primary_extrusion_width / 2;

        skirt_polygons = first_layer_outline.offset(offsetDistance, ClipperLib::jtRound);
        
        //Remove small inner skirt holes. Holes have a negative area, remove anything smaller then 100x extrusion "area"
        for(unsigned int n=0; n<skirt_polygons.size(); n++)
        {
            double area = skirt_polygons[n].area();
            if (area < 0 && area > -primary_extrusion_width * primary_extrusion_width * 100)
                skirt_polygons.remove(n--);
        }
    
        if (count == 1 && distance > 0)
        {
            skirt_polygons = skirt_polygons.convexHull(); 
        } 

        skirt_primary_extruder.add(skirt_polygons);

        int length = skirt_primary_extruder.polygonLength();
        if (skirtNr + 1 >= count && length > 0 && length < minLength) // make brim have more lines when total length is too small
            count++;
    }


    
    { //Add a skirt UNDER the prime tower to make it stick better.
        Polygons prime_tower = storage.primeTower.ground_poly.offset(-primary_extrusion_width / 2);
        std::queue<Polygons> prime_tower_insets;
        while(prime_tower.size() > 0)
        {
            prime_tower_insets.emplace(prime_tower);
            prime_tower = prime_tower.offset(-primary_extrusion_width);
        }
        while (!prime_tower_insets.empty())
        {
            Polygons& inset = prime_tower_insets.back();
            skirt_primary_extruder.add(inset);
            prime_tower_insets.pop();
        }
    }
    
    { // process other extruders' brim/skirt (as one brim line around the old brim)
        int offset_distance = 0;
        int last_width = primary_extrusion_width;
        for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
        {
            if (extruder == primary_extruder) { continue; }
            int width = storage.meshgroup->getExtruderTrain(extruder)->getSettingInMicrons("skirt_line_width");
            offset_distance += last_width / 2 + width/2;
            last_width = width;
            while (storage.skirt[extruder].polygonLength() < minLength)
            {
                storage.skirt[extruder].add(skirt_polygons.offset(offset_distance, ClipperLib::jtRound));
                offset_distance += width;
            }
        }
    }
}

}//namespace cura
