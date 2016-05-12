#include "multiVolumes.h"

namespace cura 
{
 
void carveMultipleVolumes(std::vector<Slicer*> &volumes)
{
    //Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
    for (unsigned int volume_1_idx = 0; volume_1_idx < volumes.size(); volume_1_idx++)
    {
        Slicer& volume_1 = *volumes[volume_1_idx];
        if (volume_1.mesh->getSettingBoolean("infill_mesh"))
        {
            continue;
        }
        for (unsigned int volume_2_idx = 0; volume_2_idx < volume_1_idx; volume_2_idx++)
        {
            Slicer& volume_2 = *volumes[volume_2_idx];
            if (volume_2.mesh->getSettingBoolean("infill_mesh"))
            {
                continue;
            }
            if (!volume_1.mesh->getAABB().hit(volume_2.mesh->getAABB()))
            {
                continue;
            }
            for (unsigned int layerNr = 0; layerNr < volume_1.layers.size(); layerNr++)
            {
                SlicerLayer& layer1 = volume_1.layers[layerNr];
                SlicerLayer& layer2 = volume_2.layers[layerNr];
                layer1.polygons = layer1.polygons.difference(layer2.polygons);
            }
        }
    }
}
 
//Expand each layer a bit and then keep the extra overlapping parts that overlap with other volumes.
//This generates some overlap in dual extrusion, for better bonding in touching parts.
void generateMultipleVolumesOverlap(std::vector<Slicer*> &volumes, int overlap)
{
    if (volumes.size() < 2 || overlap <= 0)
    {
        return;
    }

    for (unsigned int layerNr = 0; layerNr < volumes[0]->layers.size(); layerNr++)
    {
        Polygons full_layer;
        for (Slicer* volume : volumes)
        {
            SlicerLayer& layer1 = volume->layers[layerNr];
            full_layer = full_layer.unionPolygons(layer1.polygons.offset(20)); // TODO: put hard coded value in a variable with an explanatory name (and make var a parameter, and perhaps even a setting?)
        }
        full_layer = full_layer.offset(-20); // TODO: put hard coded value in a variable with an explanatory name (and make var a parameter, and perhaps even a setting?)

        for (Slicer* volume : volumes)
        {
            SlicerLayer& layer1 = volume->layers[layerNr];
            layer1.polygons = full_layer.intersection(layer1.polygons.offset(overlap / 2));
        }
    }
}
 

}//namespace cura
