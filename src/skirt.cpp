/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skirt.h"
#include "support.h"

#include <queue> 

namespace cura 
{

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count, int minLength)
{
    if (count == 0) return;
    
    bool externalOnly = (distance > 0);
    
    Polygons support;
    if (storage.support.generated) 
        support = storage.support.supportLayers[0].supportAreas
                .unionPolygons(storage.support.supportLayers[0].roofs);
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

    
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        int offsetDistance = distance + extrusionWidth * skirtNr + extrusionWidth / 2;

        Polygons skirtPolygons(storage.primeTower.ground_poly.offset(offsetDistance));
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


    //Add a skirt under the prime tower to make it stick better.
    Polygons prime_tower = storage.primeTower.ground_poly.offset(-extrusionWidth / 2);
    std::queue<Polygons> prime_tower_insets;
    while(prime_tower.size() > 0)
    {
        prime_tower_insets.emplace(prime_tower);
        prime_tower = prime_tower.offset(-extrusionWidth);
    }
    while (!prime_tower_insets.empty())
    {
        Polygons& inset = prime_tower_insets.back();
        storage.skirt.add(inset);
        prime_tower_insets.pop();
    }
    
    if (count == 1 && distance > 0)
    {
        storage.skirt = storage.skirt.convexHull(); 
    } 
}

}//namespace cura
