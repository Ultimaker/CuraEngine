// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "slice_data/SliceMeshStorage.h"

#include "ExtruderTrain.h"
#include "geometry/OpenPolyline.h"
#include "infill/DensityProvider.h" // for destructor
#include "infill/LightningGenerator.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/SubDivCube.h" // For the destructor
#include "mesh.h"

namespace cura
{

SliceMeshStorage::SliceMeshStorage(Mesh* mesh, const size_t slice_layer_count)
    : settings(mesh->settings_)
    , mesh_name(mesh->mesh_name_)
    , layer_nr_max_filled_layer(0)
    , bounding_box(mesh->getAABB())
    , base_subdiv_cube(nullptr)
    , cross_fill_provider(nullptr)
    , lightning_generator(nullptr)
{
    layers.resize(slice_layer_count);
}

bool SliceMeshStorage::getExtruderIsUsed(const size_t extruder_nr) const
{
    if (settings.get<bool>("anti_overhang_mesh") || settings.get<bool>("support_mesh"))
    { // object is not printed as object, but as support.
        return false;
    }
    if (settings.get<bool>("magic_spiralize"))
    {
        if (settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ == extruder_nr)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    if (settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ == extruder_nr)
    {
        return true;
    }
    if (settings.get<size_t>("wall_line_count") > 0 && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ == extruder_nr)
    {
        return true;
    }
    if ((settings.get<size_t>("wall_line_count") > 1 || settings.get<bool>("alternate_extra_perimeter"))
        && settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr_ == extruder_nr)
    {
        return true;
    }
    if (settings.get<coord_t>("infill_line_distance") > 0 && settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr_ == extruder_nr)
    {
        return true;
    }
    if ((settings.get<size_t>("top_layers") > 0 || settings.get<size_t>("bottom_layers") > 0) && settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr_ == extruder_nr)
    {
        return true;
    }
    const size_t roofing_layer_count = std::min(settings.get<size_t>("roofing_layer_count"), settings.get<size_t>("top_layers"));
    if (roofing_layer_count > 0 && settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr_ == extruder_nr)
    {
        return true;
    }
    const size_t flooring_layer_count = std::min(settings.get<size_t>("flooring_layer_count"), settings.get<size_t>("bottom_layers"));
    if (flooring_layer_count > 0 && settings.get<ExtruderTrain&>("flooring_extruder_nr").extruder_nr_ == extruder_nr)
    {
        return true;
    }
    return false;
}

bool SliceMeshStorage::getExtruderIsUsed(const size_t extruder_nr, const LayerIndex& layer_nr) const
{
    if (layer_nr < 0 || layer_nr >= static_cast<int>(layers.size()))
    {
        return false;
    }
    if (settings.get<bool>("anti_overhang_mesh") || settings.get<bool>("support_mesh"))
    { // object is not printed as object, but as support.
        return false;
    }
    const SliceLayer& layer = layers[layer_nr];
    if (settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ == extruder_nr
        && (settings.get<size_t>("wall_line_count") > 0 || settings.get<size_t>("skin_outline_count") > 0))
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if ((part.hasWallAtInsetIndex(0)) || ! part.spiral_wall.empty())
            {
                return true;
            }
        }
    }
    if (settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ == extruder_nr
        && layer.open_polylines.size() > 0)
    {
        return true;
    }
    if ((settings.get<size_t>("wall_line_count") > 1 || settings.get<bool>("alternate_extra_perimeter"))
        && settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr_ == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.hasWallAtInsetIndex(1))
            {
                return true;
            }
        }
    }
    if (settings.get<coord_t>("infill_line_distance") > 0 && settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr_ == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.getOwnInfillArea().size() > 0)
            {
                return true;
            }
        }
    }
    if (settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr_ == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            for (const SkinPart& skin_part : part.skin_parts)
            {
                if (! skin_part.skin_fill.empty())
                {
                    return true;
                }
            }
        }
    }
    if (settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr_ == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            for (const SkinPart& skin_part : part.skin_parts)
            {
                if (! skin_part.roofing_fill.empty())
                {
                    return true;
                }
            }
        }
    }
    if (settings.get<ExtruderTrain&>("flooring_extruder_nr").extruder_nr_ == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            for (const SkinPart& skin_part : part.skin_parts)
            {
                if (! skin_part.flooring_fill.empty())
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool SliceMeshStorage::isPrinted() const
{
    return ! settings.get<bool>("infill_mesh") && ! settings.get<bool>("cutting_mesh") && ! settings.get<bool>("anti_overhang_mesh");
}

Point2LL SliceMeshStorage::getZSeamHint() const
{
    Point2LL pos(settings.get<coord_t>("z_seam_x"), settings.get<coord_t>("z_seam_y"));
    if (settings.get<bool>("z_seam_relative"))
    {
        Point3LL middle = bounding_box.getMiddle();
        pos += Point2LL(middle.x_, middle.y_);
    }
    return pos;
}

} // namespace cura
