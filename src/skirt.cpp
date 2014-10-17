/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skirt.h"
#include "support.h"

namespace cura {

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count, int minLength, int initialLayerHeight)
{
    bool externalOnly = (distance > 0);
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        int offsetDistance = distance + extrusionWidth * skirtNr + extrusionWidth / 2;
        
        Polygons skirtPolygons(storage.wipeTower.offset(offsetDistance));
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.layers.size() < 1) continue;
            SliceLayer* layer = &mesh.layers[0];
            for(unsigned int i=0; i<layer->parts.size(); i++)
            {
                if (externalOnly)
                {
                    Polygons p;
                    p.add(layer->parts[i].outline[0]);
                    skirtPolygons = skirtPolygons.unionPolygons(p.offset(offsetDistance, ClipperLib::jtRound));
                }
                else
                    skirtPolygons = skirtPolygons.unionPolygons(layer->parts[i].outline.offset(offsetDistance, ClipperLib::jtRound));
            }
        }
        
        SupportPolyGenerator supportGenerator(storage.support, initialLayerHeight);
        skirtPolygons = skirtPolygons.unionPolygons(supportGenerator.polygons.offset(offsetDistance, ClipperLib::jtRound));

        //Remove small inner skirt holes. Holes have a negative area, remove anything smaller then 100x extrusion "area"
        for(unsigned int n=0; n<skirtPolygons.size(); n++)
        {
            double area = skirtPolygons[n].area();
            if (area < 0 && area > -extrusionWidth * extrusionWidth * 100)
                skirtPolygons.remove(n--);
        }

        storage.skirt.add(skirtPolygons);
        
        int lenght = storage.skirt.polygonLength();
        if (skirtNr + 1 >= count && lenght > 0 && lenght < minLength)
            count++;
    }
}

}//namespace cura
