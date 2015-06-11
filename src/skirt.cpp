/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skirt.h"
#include "support.h"

namespace cura 
{

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count, int minLength)
{
    if (count == 0) return;
    
    bool externalOnly = (distance > 0);
    
    Polygons support;
    if (storage.support.generated) 
        support = storage.support.supportLayers[0].supportAreas;
    { // get support polygons
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.layers.size() < 1) continue;
            SliceLayer* layer = &mesh.layers[0];
            for(unsigned int i=0; i<layer->parts.size(); i++)        
                support = support.difference(layer->parts[i].outline);
        }
        
        // expand and contract to smooth the final polygon
        if (count == 1 && distance > 0)
        {
            int dist = extrusionWidth * 5;
            support = support.offset(dist).offset(-dist);
        }
    }
    
    int overshoot = 0; // distance by which to expand and contract the skirt to approximate the convex hull of the first layer
    if (count == 1 && distance > 0)
    {
        overshoot = 100000; // 10 cm 
    } 
    
    
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        int offsetDistance = distance + extrusionWidth * skirtNr + extrusionWidth / 2 + overshoot;

        Polygons skirtPolygons(storage.wipeTower.offset(offsetDistance));
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.layers.size() < 1) continue;
            for(SliceLayerPart& part : mesh.layers[0].parts)
            {
                if (externalOnly)
                {
                    Polygons p;
                    p.add(part.outline.outerPolygon());
                    skirtPolygons = skirtPolygons.unionPolygons(p.offset(offsetDistance, ClipperLib::jtRound));
                }
                else
                {
                    skirtPolygons = skirtPolygons.unionPolygons(part.outline.offset(offsetDistance, ClipperLib::jtRound));
                }
            }
        }

        skirtPolygons = skirtPolygons.unionPolygons(support.offset(offsetDistance, ClipperLib::jtRound));
        //Remove small inner skirt holes. Holes have a negative area, remove anything smaller then 100x extrusion "area"
        for(unsigned int n=0; n<skirtPolygons.size(); n++)
        {
            double area = skirtPolygons[n].area();
            if (area < 0 && area > -extrusionWidth * extrusionWidth * 100)
                skirtPolygons.remove(n--);
        }

        storage.skirt.add(skirtPolygons);

        int lenght = storage.skirt.polygonLength();
        if (skirtNr + 1 >= count && lenght > 0 && lenght < minLength) // make brim have more lines when total length is too small
            count++;
    }


    //Add a skirt under the wipetower to make it stick better.
    Polygons wipe_tower = storage.wipeTower.offset(-extrusionWidth / 2);
    while(wipe_tower.size() > 0)
    {
        storage.skirt.add(wipe_tower);
        wipe_tower = wipe_tower.offset(-extrusionWidth);
    }

    if (overshoot > 0)
    {
        storage.skirt = storage.skirt.offset(-overshoot, ClipperLib::jtRound);
    }
}

}//namespace cura
