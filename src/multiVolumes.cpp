// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "multiVolumes.h"

#include <algorithm>

#include "Application.h"
#include "Slice.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "settings/EnumSettings.h"
#include "settings/types/LayerIndex.h"
#include "slicer.h"
#include "utils/OpenPolylineStitcher.h"

namespace cura
{

void carveMultipleVolumes(std::vector<Slicer*>& volumes)
{
    // Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
    const bool alternate_carve_order = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<bool>("alternate_carve_order");
    std::vector<Slicer*> ranked_volumes = volumes;
    std::stable_sort(
        ranked_volumes.begin(),
        ranked_volumes.end(),
        [](Slicer* volume_1, Slicer* volume_2)
        {
            return volume_1->mesh->settings_.get<int>("infill_mesh_order") < volume_2->mesh->settings_.get<int>("infill_mesh_order");
        });
    for (unsigned int volume_1_idx = 1; volume_1_idx < volumes.size(); volume_1_idx++)
    {
        Slicer& volume_1 = *ranked_volumes[volume_1_idx];
        if (volume_1.mesh->settings_.get<bool>("infill_mesh") || volume_1.mesh->settings_.get<bool>("anti_overhang_mesh") || volume_1.mesh->settings_.get<bool>("support_mesh")
            || volume_1.mesh->settings_.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
        {
            continue;
        }
        for (unsigned int volume_2_idx = 0; volume_2_idx < volume_1_idx; volume_2_idx++)
        {
            Slicer& volume_2 = *ranked_volumes[volume_2_idx];
            if (volume_2.mesh->settings_.get<bool>("infill_mesh") || volume_2.mesh->settings_.get<bool>("anti_overhang_mesh") || volume_2.mesh->settings_.get<bool>("support_mesh")
                || volume_2.mesh->settings_.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
            {
                continue;
            }
            if (! volume_1.mesh->getAABB().hit(volume_2.mesh->getAABB()))
            {
                continue;
            }
            for (LayerIndex layerNr = 0; layerNr < volume_1.layers.size(); layerNr++)
            {
                SlicerLayer& layer1 = volume_1.layers[layerNr];
                SlicerLayer& layer2 = volume_2.layers[layerNr];
                if (alternate_carve_order && layerNr % 2 == 0 && volume_1.mesh->settings_.get<int>("infill_mesh_order") == volume_2.mesh->settings_.get<int>("infill_mesh_order"))
                {
                    layer2.polygons_ = layer2.polygons_.difference(layer1.polygons_);
                }
                else
                {
                    layer1.polygons_ = layer1.polygons_.difference(layer2.polygons_);
                }
            }
        }
    }
}

// Expand each layer a bit and then keep the extra overlapping parts that overlap with other volumes.
// This generates some overlap in dual extrusion, for better bonding in touching parts.
void generateMultipleVolumesOverlap(std::vector<Slicer*>& volumes)
{
    if (volumes.size() < 2)
    {
        return;
    }

    int offset_to_merge_other_merged_volumes = 20;
    for (Slicer* volume : volumes)
    {
        ClipperLib::PolyFillType fill_type = volume->mesh->settings_.get<bool>("meshfix_union_all") ? ClipperLib::pftNonZero : ClipperLib::pftEvenOdd;

        coord_t overlap = volume->mesh->settings_.get<coord_t>("multiple_mesh_overlap");
        if (volume->mesh->settings_.get<bool>("infill_mesh") || volume->mesh->settings_.get<bool>("anti_overhang_mesh") || volume->mesh->settings_.get<bool>("support_mesh")
            || overlap == 0)
        {
            continue;
        }
        AABB3D aabb(volume->mesh->getAABB());
        aabb.expandXY(overlap); // expand to account for the case where two models and their bounding boxes are adjacent along the X or Y-direction
        for (LayerIndex layer_nr = 0; layer_nr < volume->layers.size(); layer_nr++)
        {
            Shape all_other_volumes;
            for (Slicer* other_volume : volumes)
            {
                if (other_volume->mesh->settings_.get<bool>("infill_mesh") || other_volume->mesh->settings_.get<bool>("anti_overhang_mesh")
                    || other_volume->mesh->settings_.get<bool>("support_mesh") || ! other_volume->mesh->getAABB().hit(aabb) || other_volume == volume)
                {
                    continue;
                }
                SlicerLayer& other_volume_layer = other_volume->layers[layer_nr];
                all_other_volumes = all_other_volumes.unionPolygons(other_volume_layer.polygons_.offset(offset_to_merge_other_merged_volumes), fill_type);
            }

            SlicerLayer& volume_layer = volume->layers[layer_nr];
            volume_layer.polygons_ = volume_layer.polygons_.unionPolygons(all_other_volumes.intersection(volume_layer.polygons_.offset(overlap / 2)), fill_type);
        }
    }
}

void MultiVolumes::carveCuttingMeshes(std::vector<Slicer*>& volumes, std::vector<Mesh>& meshes)
{
    // Modifier mesh that change the extruder are treated differently: their volume is removed from regular meshes, then they are intersected with them, and the remainder
    // is subsequently treated as a regular mesh
    bool has_extruder_change_mesh = false;
    for (const Mesh& cutting_mesh : meshes)
    {
        has_extruder_change_mesh |= cutting_mesh.settings_.get<bool>("cutting_mesh") && cutting_mesh.settings_.has("extruder_nr");
    }

    std::unordered_map<LayerIndex, Shape> layer_printable_mesh_unions;
    if (has_extruder_change_mesh)
    {
        // Before we make any change to the actual sliced meshes, compute the full union of all the printable meshes on each layer
        for (size_t printable_mesh_idx = 0; printable_mesh_idx < meshes.size(); ++printable_mesh_idx)
        {
            if (! meshes[printable_mesh_idx].isPrinted())
            {
                continue;
            }

            Slicer& printable_mesh_volume = *volumes[printable_mesh_idx];
            for (LayerIndex layer_nr = 0; layer_nr < printable_mesh_volume.layers.size(); ++layer_nr)
            {
                const Shape& printable_mesh_area = printable_mesh_volume.layers[layer_nr].polygons_;
                auto iterator = layer_printable_mesh_unions.find(layer_nr);
                if (iterator != layer_printable_mesh_unions.end())
                {
                    layer_printable_mesh_unions[layer_nr] = layer_printable_mesh_unions[layer_nr].unionPolygons(printable_mesh_area);
                }
                else
                {
                    layer_printable_mesh_unions[layer_nr] = printable_mesh_area;
                }
            }
        }
    }

    for (size_t carving_mesh_idx = 0; carving_mesh_idx < volumes.size(); ++carving_mesh_idx)
    {
        Mesh& cutting_mesh = meshes[carving_mesh_idx];
        if (! cutting_mesh.settings_.get<bool>("cutting_mesh"))
        {
            continue;
        }

        bool is_extruder_change_mesh = cutting_mesh.settings_.has("extruder_nr");

        Slicer& cutting_mesh_volume = *volumes[carving_mesh_idx];
        for (LayerIndex layer_nr = 0; layer_nr < cutting_mesh_volume.layers.size(); ++layer_nr)
        {
            Shape& cutting_mesh_polygons = cutting_mesh_volume.layers[layer_nr].polygons_;
            OpenLinesSet& cutting_mesh_polylines = cutting_mesh_volume.layers[layer_nr].open_polylines_;
            Shape cutting_mesh_area_recomputed;
            Shape* cutting_mesh_area;
            coord_t surface_line_width = cutting_mesh.settings_.get<coord_t>("wall_line_width_0");
            { // compute cutting_mesh_area
                if (cutting_mesh.settings_.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::BOTH)
                {
                    cutting_mesh_area_recomputed = cutting_mesh_polygons.unionPolygons(cutting_mesh_polylines.offset(surface_line_width / 2));
                    cutting_mesh_area = &cutting_mesh_area_recomputed;
                }
                else if (cutting_mesh.settings_.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
                {
                    // break up polygons into polylines
                    // they have to be polylines, because they might break up further when doing the cutting
                    for (Polygon& poly : cutting_mesh_polygons)
                    {
                        poly.push_back(poly.front());
                        cutting_mesh_polylines.emplace_back(poly.getPoints());
                    }

                    cutting_mesh_polygons.clear();
                    cutting_mesh_area_recomputed = cutting_mesh_polylines.offset(surface_line_width / 2);
                    cutting_mesh_area = &cutting_mesh_area_recomputed;
                }
                else
                {
                    cutting_mesh_area = &cutting_mesh_polygons;
                }

                if (is_extruder_change_mesh)
                {
                    auto iterator = layer_printable_mesh_unions.find(layer_nr);
                    if (iterator != layer_printable_mesh_unions.end())
                    {
                        *cutting_mesh_area = cutting_mesh_area->intersection(iterator->second);
                    }
                    else
                    {
                        // There is no printable object at this layer, just invalidate the cutting mesh area
                        cutting_mesh_area->clear();
                    }
                }
            }

            Shape new_outlines;
            OpenLinesSet new_polylines;
            for (unsigned int carved_mesh_idx = 0; carved_mesh_idx < volumes.size(); carved_mesh_idx++)
            {
                const Mesh& carved_mesh = meshes[carved_mesh_idx];
                // Do not apply cutting_mesh for meshes which have settings (cutting_mesh, anti_overhang_mesh, support_mesh).
                if (carved_mesh.settings_.get<bool>("cutting_mesh") || carved_mesh.settings_.get<bool>("anti_overhang_mesh") || carved_mesh.settings_.get<bool>("support_mesh"))
                {
                    continue;
                }
                Slicer& carved_volume = *volumes[carved_mesh_idx];
                Shape& carved_mesh_layer = carved_volume.layers[layer_nr].polygons_;

                Shape intersection = cutting_mesh_polygons.intersection(carved_mesh_layer);
                new_outlines.push_back(intersection);
                if (cutting_mesh.settings_.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL) // niet te geleuven
                {
                    new_polylines.push_back(carved_mesh_layer.intersection(cutting_mesh_polylines));
                }

                carved_mesh_layer = carved_mesh_layer.difference(*cutting_mesh_area);
            }
            cutting_mesh_polygons = new_outlines.unionPolygons();
            if (cutting_mesh.settings_.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
            {
                cutting_mesh_polylines.clear();
                OpenPolylineStitcher::stitch(new_polylines, cutting_mesh_polylines, cutting_mesh_polygons, surface_line_width);
            }
        }

        if (is_extruder_change_mesh)
        {
            // Starting now, consider this mesh as a regular mesh
            cutting_mesh.settings_.remove("cutting_mesh");
        }
    }
}

} // namespace cura
