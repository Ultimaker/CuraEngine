/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "SpaghettiInfill.h"

namespace cura {


void SpaghettiInfill::generateSpaghettiInfill(SliceMeshStorage& mesh)
{
    const coord_t line_width = mesh.getSettingInMicrons("infill_line_width");
    coord_t spaghetti_max_height = mesh.getSettingInMicrons("spaghetti_max_height");

    coord_t filling_area_inset = mesh.getSettingInMicrons("spaghetti_inset");

    coord_t connection_inset_dist;
    if (mesh.getSettingInAngleDegrees("spaghetti_max_infill_angle") >= 90)
    {
        connection_inset_dist = MM2INT(500); // big enough for all standard printers.
    }
    else
    {
        connection_inset_dist = tan(mesh.getSettingInAngleRadians("spaghetti_max_infill_angle")) * mesh.getSettingInMicrons("layer_height"); // Horizontal component of the spaghetti_max_infill_angle
    }

    std::list<SpaghettiInfill::InfillPillar> pillar_base;
    coord_t current_z = 0;

    size_t max_layer = mesh.layers.size() - 1 - mesh.getSettingAsCount("top_layers");
    for (size_t layer_idx = 0; layer_idx <= max_layer; layer_idx++) //Skip every few layers, but extrude more.
    {
        const coord_t layer_height = (layer_idx == 0)? mesh.getSettingInMicrons("layer_height_0") : mesh.getSettingInMicrons("layer_height");
        current_z += layer_height;
        if (static_cast<int>(layer_idx) < mesh.getSettingAsCount("bottom_layers"))
        { // nothing to add to pillar base
            continue;
        }
        SliceLayer& layer = mesh.layers[layer_idx];

        // add infill parts to pillar_base
        for (SliceLayerPart& slice_layer_part : layer.parts)
        {
            std::vector<PolygonsPart> part_infill_parts = slice_layer_part.getOwnInfillArea().splitIntoParts();

            // add parts to pillar_base
            for (PolygonsPart& infill_part : part_infill_parts)
            {
                coord_t bottom_z = current_z - layer_height;
                SpaghettiInfill::InfillPillar& pillar = addPartToPillarBase(infill_part, pillar_base, connection_inset_dist, layer_height, bottom_z);
                pillar.top_slice_layer_part = &slice_layer_part;
                pillar.last_layer_added = layer_idx;
            }
        }


        // handle finished pillars
        for (auto pillar_it = pillar_base.begin(); pillar_it != pillar_base.end();)
        {
            InfillPillar& pillar = *pillar_it;
            if (current_z - pillar.bottom_z >= spaghetti_max_height
                || pillar.last_layer_added < static_cast<int>(layer_idx)
            )
            {
                pillar.addToTopSliceLayerPart(filling_area_inset, line_width);
                auto to_be_erased = pillar_it;
                ++pillar_it;
                pillar_base.erase(to_be_erased);
            }
            else
            {
                ++pillar_it;
            }
        }
    }
    // handle unfinished pillars
    for (auto it = pillar_base.begin(); it != pillar_base.end(); ++it)
    {
        it->addToTopSliceLayerPart(filling_area_inset, line_width);
    }
}

void SpaghettiInfill::InfillPillar::addToTopSliceLayerPart(coord_t filling_area_inset, coord_t line_width)
{
    SliceLayerPart& slice_layer_part = *top_slice_layer_part;
    double volume = total_volume_mm3;
    assert(volume > 0.0);

    // get filling area
    Polygons filling_area = top_part.offset(-filling_area_inset);
    assert(top_part.size() > 0 && top_part[0].size() > 0 && "the top part must be a non-zero area!");
    if (filling_area.size() == 0)
    {
        AABB aabb(top_part);
        Point inside = (aabb.min + aabb.max) / 2;
        PolygonUtils::moveInside(top_part, inside);

        filling_area = PolygonsPart();
        PolygonRef poly = filling_area.newPoly();
        poly.emplace_back(inside + Point(-line_width / 2 - 10, line_width / 2 + 10));
        poly.emplace_back(inside + Point(line_width / 2 + 10, line_width / 2 + 10));
        poly.emplace_back(inside + Point(line_width / 2 + 10, -line_width / 2 - 10));
        poly.emplace_back(inside + Point(-line_width / 2 - 10, -line_width / 2 - 10));
    }
    slice_layer_part.spaghetti_infill_volumes.emplace_back(filling_area, volume);
}

bool SpaghettiInfill::InfillPillar::isConnected(const PolygonsPart& infill_part) const
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

SpaghettiInfill::InfillPillar& SpaghettiInfill::addPartToPillarBase(const PolygonsPart& infill_part, std::list<SpaghettiInfill::InfillPillar>& pillar_base, coord_t connection_inset_dist, int layer_height, coord_t bottom_z)
{
    std::list<SpaghettiInfill::InfillPillar>::iterator ret = pillar_base.end();
    for (auto it = pillar_base.begin(); it != pillar_base.end(); ++it)
    {
        InfillPillar& pillar = *it;
        if (pillar.isConnected(infill_part))
        {
            pillar.total_volume_mm3 += INT2MM(INT2MM(infill_part.area())) * INT2MM(layer_height);
            pillar.top_part = infill_part;
            if (ret != pillar_base.end())
            { // connecting two pillars of the layer below via one area on this layer
                pillar.total_volume_mm3 += ret->total_volume_mm3;
                pillar_base.erase(ret);
            }
            ret = it;
        }
    }
    if (ret == pillar_base.end())
    { // couldn't connect to any existing pillar
        pillar_base.emplace_back(infill_part, connection_inset_dist, layer_height, bottom_z);
        return pillar_base.back();
    }
    return *ret;
}


}//namespace cura
