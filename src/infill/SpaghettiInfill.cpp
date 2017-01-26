/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "SpaghettiInfill.h"

namespace cura {


void SpaghettiInfill::generateSpaghettiInfill(SliceMeshStorage& mesh)
{
    coord_t layer_height = mesh.getSettingInMicrons("layer_height");
    double layer_height_mm = INT2MM(layer_height);
    int spaghetti_max_layer_count = std::max(1, static_cast<int>(mesh.getSettingInMicrons("spaghetti_max_height") / layer_height));
    // TODO: account for the initial layer height

    if (mesh.getSettingInAngleDegrees("spaghetti_max_infill_angle") >= 90)
    {
        return; // infill cannot be combined into pillars
    }
    coord_t connection_inset_dist = tan(mesh.getSettingInAngleRadians("spaghetti_max_infill_angle")) * layer_height;

    std::list<SpaghettiInfill::InfillPillar> pillar_base;
    size_t min_layer = mesh.getSettingAsCount("bottom_layers") + 1;
    size_t max_layer = mesh.layers.size() - 1 - mesh.getSettingAsCount("top_layers");
    for (size_t layer_idx = min_layer; layer_idx <= max_layer; layer_idx++) //Skip every few layers, but extrude more.
    {
        SliceLayer& layer = mesh.layers[layer_idx];

        // add infill parts to pillar_base
        for (SliceLayerPart& slice_layer_part : layer.parts)
        {
            std::vector<PolygonsPart> part_infill_parts = slice_layer_part.getOwnInfillArea().splitIntoParts();

            // add parts to pillar_base
            for (PolygonsPart& infill_part : part_infill_parts)
            {
                SpaghettiInfill::InfillPillar& pillar = addPartToPillarBase(infill_part, pillar_base, connection_inset_dist);
                pillar.top_slice_layer_part = &slice_layer_part;
                pillar.last_layer_added = layer_idx;
                pillar.layer_count++;
            }
        }


        // handle finished pillars
        for (auto it = pillar_base.begin(); it != pillar_base.end();)
        {
            InfillPillar& pillar = *it;
            if (pillar.layer_count >= spaghetti_max_layer_count
                || pillar.last_layer_added < static_cast<int>(layer_idx)
            )
            {
                SliceLayerPart& slice_layer_part = *pillar.top_slice_layer_part;
                double volume = pillar.total_area_mm2 * layer_height_mm;
                assert(volume > 0.0);
                slice_layer_part.spaghetti_infill_volumes.emplace_back(pillar.top_part, volume);
                auto to_be_erased = it;
                ++it;
                pillar_base.erase(to_be_erased);
            }
            else
            {
                ++it;
            }
        }
    }
    // handle unfinished pillars
    for (auto it = pillar_base.begin(); it != pillar_base.end(); ++it)
    {
        InfillPillar& pillar = *it;
        SliceLayerPart& slice_layer_part = *pillar.top_slice_layer_part;
        double volume = pillar.total_area_mm2 * layer_height_mm;
        assert(volume > 0.0);
        slice_layer_part.spaghetti_infill_volumes.emplace_back(pillar.top_part, volume);
    }
}

// TODO: make settings for the below

bool SpaghettiInfill::InfillPillar::isConnected(const PolygonsPart& infill_part)
{
    Polygons insetted = infill_part.offset(-connection_inset_dist);
    if (insetted.intersection(top_part).size() > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

SpaghettiInfill::InfillPillar& SpaghettiInfill::addPartToPillarBase(PolygonsPart& infill_part, std::list<SpaghettiInfill::InfillPillar>& pillar_base, coord_t connection_inset_dist)
{
    std::list<SpaghettiInfill::InfillPillar>::iterator ret = pillar_base.end();
    for (auto it = pillar_base.begin(); it != pillar_base.end(); ++it)
    {
        InfillPillar& pillar = *it;
        if (pillar.isConnected(infill_part))
        {
            pillar.total_area_mm2 += INT2MM(INT2MM(infill_part.area()));
            pillar.top_part = infill_part;
            if (ret != pillar_base.end())
            { // connecting two pillars of the layer below via one area on this layer
                pillar.total_area_mm2 += ret->total_area_mm2;
                pillar_base.erase(ret);
            }
            ret = it;
        }
    }
    if (ret == pillar_base.end())
    { // couldn't connect to any existing pillar
        pillar_base.emplace_back(infill_part, connection_inset_dist);
        return pillar_base.back();
    }
    return *ret;
}


}//namespace cura
