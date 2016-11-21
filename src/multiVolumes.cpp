#include "multiVolumes.h"

namespace cura 
{
 
void carveMultipleVolumes(std::vector<Slicer*> &volumes, bool alternate_carve_order)
{
    //Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
    for (unsigned int volume_1_idx = 0; volume_1_idx < volumes.size(); volume_1_idx++)
    {
        Slicer& volume_1 = *volumes[volume_1_idx];
        if (volume_1.mesh->getSettingBoolean("infill_mesh") 
            || volume_1.mesh->getSettingBoolean("anti_overhang_mesh")
            || volume_1.mesh->getSettingBoolean("support_mesh")
            )
        {
            continue;
        }
        for (unsigned int volume_2_idx = 0; volume_2_idx < volume_1_idx; volume_2_idx++)
        {
            Slicer& volume_2 = *volumes[volume_2_idx];
            if (volume_2.mesh->getSettingBoolean("infill_mesh")
                || volume_2.mesh->getSettingBoolean("anti_overhang_mesh")
                || volume_2.mesh->getSettingBoolean("support_mesh")
                )
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
                if (alternate_carve_order && layerNr % 2 == 0)
                {
                    layer2.polygons = layer2.polygons.difference(layer1.polygons);
                }
                else
                {
                    layer1.polygons = layer1.polygons.difference(layer2.polygons);
                }
            }
        }
    }
}
 
//Expand each layer a bit and then keep the extra overlapping parts that overlap with other volumes.
//This generates some overlap in dual extrusion, for better bonding in touching parts.
void generateMultipleVolumesOverlap(std::vector<Slicer*> &volumes)
{
    if (volumes.size() < 2)
    {
        return;
    }

    int offset_to_merge_other_merged_volumes = 20;
    for (Slicer* volume : volumes)
    {
        int overlap = volume->mesh->getSettingInMicrons("multiple_mesh_overlap");
        if (volume->mesh->getSettingBoolean("infill_mesh")
            || volume->mesh->getSettingBoolean("anti_overhang_mesh")
            || volume->mesh->getSettingBoolean("support_mesh")
            || overlap == 0)
        {
            continue;
        }
        AABB3D aabb(volume->mesh->getAABB());
        aabb.expandXY(overlap); // expand to account for the case where two models and their bounding boxes are adjacent along the X or Y-direction
        for (unsigned int layer_nr = 0; layer_nr < volume->layers.size(); layer_nr++)
        {
            Polygons all_other_volumes;
            for (Slicer* other_volume : volumes)
            {
                if (other_volume->mesh->getSettingBoolean("infill_mesh")
                    || other_volume->mesh->getSettingBoolean("anti_overhang_mesh")
                    || other_volume->mesh->getSettingBoolean("support_mesh")
                    || !other_volume->mesh->getAABB().hit(aabb)
                    || other_volume == volume
                )
                {
                    continue;
                }
                SlicerLayer& other_volume_layer = other_volume->layers[layer_nr];
                all_other_volumes = all_other_volumes.unionPolygons(other_volume_layer.polygons.offset(offset_to_merge_other_merged_volumes));
            }

            SlicerLayer& volume_layer = volume->layers[layer_nr];
            volume_layer.polygons = volume_layer.polygons.unionPolygons(all_other_volumes.intersection(volume_layer.polygons.offset(overlap / 2)));
        }
    }
}
 

}//namespace cura
