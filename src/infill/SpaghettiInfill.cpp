//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SpaghettiInfill.h"
#include "../sliceDataStorage.h"
#include "../settings/types/AngleDegrees.h" //For the infill angle.
#include "../settings/types/AngleRadians.h" //For the infill angle.
#include "../utils/logoutput.h"

namespace cura {

void SpaghettiInfill::generateTotalSpaghettiInfill(SliceMeshStorage& mesh)
{
    double total_volume_mm3 = 0.0;

    const int max_layer = mesh.layers.size() - 1 - mesh.settings.get<size_t>("top_layers");
    for (int layer_idx = 0; layer_idx <= max_layer; layer_idx++)
    {
        const coord_t layer_height = (layer_idx == 0) ? mesh.settings.get<coord_t>("layer_height_0") : mesh.settings.get<coord_t>("layer_height");
        if (layer_idx < static_cast<LayerIndex>(mesh.settings.get<size_t>("bottom_layers")))
        { // nothing to add
            continue;
        }
        const SliceLayer& layer = mesh.layers[layer_idx];
        for (const SliceLayerPart& slice_layer_part : layer.parts)
        {
            const Polygons& part_infill = slice_layer_part.getOwnInfillArea();
            total_volume_mm3 += INT2MM(INT2MM(part_infill.area())) * INT2MM(layer_height);
        }
    }

    // find top most filling area
    SliceLayerPart* top_filling_layer_part = nullptr;
    for (int layer_idx = mesh.layers.size() - 1; layer_idx >= 0; layer_idx--)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        for (SliceLayerPart& part : layer.parts)
        {
            if (part.getOwnInfillArea().size() > 0)
            { // WARNING: we just use the very first part we encounter; there's no way for the user to choose which filling area will be used
                top_filling_layer_part = &part;
                break;
            }
        }
        if (top_filling_layer_part)
        {
            break; // it's already found
        }
    }

    if (!top_filling_layer_part)
    { // there is no infill in the whole mesh!
        logError("Spaghetti Infill mesh doesn't have any volume!\n");
        return;
    }
    assert(top_filling_layer_part->getOwnInfillArea().size() > 0);
    const PolygonsPart top_infill_part = top_filling_layer_part->getOwnInfillArea().splitIntoParts()[0]; // WARNING: we just use the very first part we encounter; there's no way for the user to choose which filling area will be used

    const coord_t filling_area_inset = mesh.settings.get<coord_t>("spaghetti_inset");
    const coord_t line_width = mesh.settings.get<coord_t>("infill_line_width");
    const Polygons filling_area = SpaghettiInfill::getFillingArea(top_infill_part, filling_area_inset, line_width);

    // add total volume spaghetti infill to top of print
    top_filling_layer_part->spaghetti_infill_volumes.emplace_back(filling_area, total_volume_mm3);
}

void SpaghettiInfill::generateSpaghettiInfill(SliceMeshStorage& mesh)
{
    const bool total_volume_at_once = !mesh.settings.get<bool>("spaghetti_infill_stepped");

    if (total_volume_at_once)
    {
        generateTotalSpaghettiInfill(mesh);
        return;
    }

    const coord_t line_width = mesh.settings.get<coord_t>("infill_line_width");
    coord_t spaghetti_max_height = mesh.settings.get<coord_t>("spaghetti_max_height");

    const coord_t filling_area_inset = mesh.settings.get<coord_t>("spaghetti_inset");

    std::list<SpaghettiInfill::InfillPillar> pillar_base;
    coord_t current_z = 0;

    LayerIndex max_layer = mesh.layers.size() - 1 - mesh.settings.get<size_t>("top_layers");
    for (LayerIndex layer_idx = 0; layer_idx <= max_layer; layer_idx++) //Skip every few layers, but extrude more.
    {
        const coord_t layer_height = (layer_idx == 0) ? mesh.settings.get<coord_t>("layer_height_0") : mesh.settings.get<coord_t>("layer_height");
        current_z += layer_height;
        if (layer_idx < static_cast<LayerIndex>(mesh.settings.get<size_t>("bottom_layers")))
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
                SpaghettiInfill::InfillPillar& pillar = addPartToPillarBase(mesh, infill_part, pillar_base, layer_height, bottom_z);
                pillar.top_slice_layer_part = &slice_layer_part;
                pillar.last_layer_added = layer_idx;
            }
        }


        // handle finished pillars
        for (auto pillar_it = pillar_base.begin(); pillar_it != pillar_base.end();)
        {
            InfillPillar& pillar = *pillar_it;
            if (current_z - pillar.bottom_z >= spaghetti_max_height
                || pillar.last_layer_added < layer_idx
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

Polygons SpaghettiInfill::getFillingArea(const PolygonsPart& infill_area, const coord_t filling_area_inset, const coord_t line_width)
{
    Polygons filling_area = infill_area.offset(-filling_area_inset);
    assert(infill_area.size() > 0 && infill_area[0].size() > 0 && "the top part must be a non-zero area!");
    if (filling_area.size() == 0)
    {
        AABB aabb(infill_area);
        Point inside = (aabb.min + aabb.max) / 2;
        PolygonUtils::moveInside(infill_area, inside);

        filling_area = PolygonsPart();
        PolygonRef poly = filling_area.newPoly();
        poly.emplace_back(inside + Point(-line_width / 2 - 10, line_width / 2 + 10));
        poly.emplace_back(inside + Point(line_width / 2 + 10, line_width / 2 + 10));
        poly.emplace_back(inside + Point(line_width / 2 + 10, -line_width / 2 - 10));
        poly.emplace_back(inside + Point(-line_width / 2 - 10, -line_width / 2 - 10));
    }
    return filling_area;
}

SpaghettiInfill::InfillPillar::InfillPillar(const SliceMeshStorage& mesh, const PolygonsPart& _top_part, const coord_t layer_height, const coord_t bottom_z)
: top_part(_top_part) // TODO: prevent copy construction! Is that possible?
, total_volume_mm3(INT2MM(INT2MM(top_part.area())) * INT2MM(layer_height))
, connection_inset_dist(mesh.settings.get<AngleDegrees>("spaghetti_max_infill_angle") >= 90 ? MM2INT(500) : (tan(mesh.settings.get<AngleRadians>("spaghetti_max_infill_angle")) * mesh.settings.get<coord_t>("layer_height")))
, bottom_z(bottom_z)
{
}

void SpaghettiInfill::InfillPillar::addToTopSliceLayerPart(const coord_t filling_area_inset, const coord_t line_width)
{
    SliceLayerPart& slice_layer_part = *top_slice_layer_part;
    double volume = total_volume_mm3;
    assert(volume > 0.0);

    // get filling area
    Polygons filling_area = SpaghettiInfill::getFillingArea(top_part, filling_area_inset, line_width);
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

SpaghettiInfill::InfillPillar& SpaghettiInfill::addPartToPillarBase(const SliceMeshStorage& mesh, const PolygonsPart& infill_part, std::list<SpaghettiInfill::InfillPillar>& pillar_base, const coord_t layer_height, const coord_t bottom_z)
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
        pillar_base.emplace_back(mesh, infill_part, layer_height, bottom_z);
        return pillar_base.back();
    }
    return *ret;
}


}//namespace cura
