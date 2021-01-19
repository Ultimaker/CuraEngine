//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "multiVolumes.h"

#include <algorithm>
#include <unordered_set>

#include "Application.h"
#include "Slice.h"
#include "slicer.h"
#include "utils/PolylineStitcher.h"
#include "settings/EnumSettings.h"
#include "utils/PolygonUtils.h"

#include "utils/SparseCellGrid3D.h"

namespace cura 
{
 
void carveMultipleVolumes(std::vector<Slicer*> &volumes)
{
    //Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
    const bool alternate_carve_order = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("alternate_carve_order");
    std::vector<Slicer*> ranked_volumes = volumes;
    std::sort(ranked_volumes.begin(), ranked_volumes.end(),
              [](Slicer* volume_1, Slicer* volume_2)
                {
                    return volume_1->mesh->settings.get<int>("infill_mesh_order") < volume_2->mesh->settings.get<int>("infill_mesh_order");
                } );
    for (unsigned int volume_1_idx = 1; volume_1_idx < volumes.size(); volume_1_idx++)
    {
        Slicer& volume_1 = *ranked_volumes[volume_1_idx];
        if (volume_1.mesh->settings.get<bool>("infill_mesh") 
            || volume_1.mesh->settings.get<bool>("anti_overhang_mesh")
            || volume_1.mesh->settings.get<bool>("support_mesh")
            || volume_1.mesh->settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE
            )
        {
            continue;
        }
        for (unsigned int volume_2_idx = 0; volume_2_idx < volume_1_idx; volume_2_idx++)
        {
            Slicer& volume_2 = *ranked_volumes[volume_2_idx];
            if (volume_2.mesh->settings.get<bool>("infill_mesh")
                || volume_2.mesh->settings.get<bool>("anti_overhang_mesh")
                || volume_2.mesh->settings.get<bool>("support_mesh")
                || volume_2.mesh->settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE
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
                if (alternate_carve_order && layerNr % 2 == 0 && volume_1.mesh->settings.get<int>("infill_mesh_order") == volume_2.mesh->settings.get<int>("infill_mesh_order"))
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
        ClipperLib::PolyFillType fill_type = volume->mesh->settings.get<bool>("meshfix_union_all") ? ClipperLib::pftNonZero : ClipperLib::pftEvenOdd;

        coord_t overlap = volume->mesh->settings.get<coord_t>("multiple_mesh_overlap");
        if (volume->mesh->settings.get<bool>("infill_mesh")
            || volume->mesh->settings.get<bool>("anti_overhang_mesh")
            || volume->mesh->settings.get<bool>("support_mesh")
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
                if (other_volume->mesh->settings.get<bool>("infill_mesh")
                    || other_volume->mesh->settings.get<bool>("anti_overhang_mesh")
                    || other_volume->mesh->settings.get<bool>("support_mesh")
                    || !other_volume->mesh->getAABB().hit(aabb)
                    || other_volume == volume
                )
                {
                    continue;
                }
                SlicerLayer& other_volume_layer = other_volume->layers[layer_nr];
                all_other_volumes = all_other_volumes.unionPolygons(other_volume_layer.polygons.offset(offset_to_merge_other_merged_volumes), fill_type);
            }

            SlicerLayer& volume_layer = volume->layers[layer_nr];
            volume_layer.polygons = volume_layer.polygons.unionPolygons(all_other_volumes.intersection(volume_layer.polygons.offset(overlap / 2)), fill_type);
        }
    }
}

void MultiVolumes::carveCuttingMeshes(std::vector<Slicer*>& volumes, const std::vector<Mesh>& meshes)
{
    for (unsigned int carving_mesh_idx = 0; carving_mesh_idx < volumes.size(); carving_mesh_idx++)
    {
        const Mesh& cutting_mesh = meshes[carving_mesh_idx];
        if (!cutting_mesh.settings.get<bool>("cutting_mesh"))
        {
            continue;
        }
        Slicer& cutting_mesh_volume = *volumes[carving_mesh_idx];
        for (unsigned int layer_nr = 0; layer_nr < cutting_mesh_volume.layers.size(); layer_nr++)
        {
            Polygons& cutting_mesh_polygons = cutting_mesh_volume.layers[layer_nr].polygons;
            Polygons& cutting_mesh_polylines = cutting_mesh_volume.layers[layer_nr].openPolylines;
            Polygons cutting_mesh_area_recomputed;
            Polygons* cutting_mesh_area;
            coord_t surface_line_width = cutting_mesh.settings.get<coord_t>("wall_line_width_0");
            { // compute cutting_mesh_area
                if (cutting_mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::BOTH)
                {
                    cutting_mesh_area_recomputed = cutting_mesh_polygons.unionPolygons(cutting_mesh_polylines.offsetPolyLine(surface_line_width / 2));
                    cutting_mesh_area = &cutting_mesh_area_recomputed;
                }
                else if (cutting_mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
                {
                    // break up polygons into polylines
                    // they have to be polylines, because they might break up further when doing the cutting
                    for (PolygonRef poly : cutting_mesh_polygons)
                    {
                        poly.add(poly[0]);
                    }
                    cutting_mesh_polylines.add(cutting_mesh_polygons);
                    cutting_mesh_polygons.clear();
                    cutting_mesh_area_recomputed = cutting_mesh_polylines.offsetPolyLine(surface_line_width / 2);
                    cutting_mesh_area = &cutting_mesh_area_recomputed;
                }
                else
                {
                    cutting_mesh_area = &cutting_mesh_polygons;
                }
            }
            
            Polygons new_outlines;
            Polygons new_polylines;
            for (unsigned int carved_mesh_idx = 0; carved_mesh_idx < volumes.size(); carved_mesh_idx++)
            {
                const Mesh& carved_mesh = meshes[carved_mesh_idx];
                //Do not apply cutting_mesh for meshes which have settings (cutting_mesh, anti_overhang_mesh, support_mesh).
                if (carved_mesh.settings.get<bool>("cutting_mesh") || carved_mesh.settings.get<bool>("anti_overhang_mesh")
                    || carved_mesh.settings.get<bool>("support_mesh"))
                {
                    continue;
                }
                Slicer& carved_volume = *volumes[carved_mesh_idx];
                Polygons& carved_mesh_layer = carved_volume.layers[layer_nr].polygons;

                Polygons intersection = cutting_mesh_polygons.intersection(carved_mesh_layer);
                new_outlines.add(intersection);
                if (cutting_mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL) // niet te geleuven
                {
                    new_polylines.add(carved_mesh_layer.intersectionPolyLines(cutting_mesh_polylines));
                }

                carved_mesh_layer = carved_mesh_layer.difference(*cutting_mesh_area);
            }
            cutting_mesh_polygons = new_outlines.unionPolygons();
            if (cutting_mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
            {
                cutting_mesh_polylines.clear();
                cutting_mesh_polygons.clear();
                PolylineStitcher<Polygons, Polygon, Point>::stitch(new_polylines, cutting_mesh_polylines, cutting_mesh_polygons, surface_line_width);
            }
        }
    }
}

void MultiVolumes::generateInterlockingStructure(std::vector<Slicer*>& volumes)
{
    /*
    if ( ! Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("interlocking_structure_gen"))
    {
        return;
    }
    */

    
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    
    const std::vector<ExtruderTrain>& extruders = Application::getInstance().current_slice->scene.extruders;
    std::vector<coord_t> line_width_per_extruder(extruder_count);
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        line_width_per_extruder[extruder_nr] = extruders[extruder_nr].settings.get<coord_t>("wall_line_width_0");
    }
    coord_t cell_size = (line_width_per_extruder[0] + line_width_per_extruder[1]) * 2;
    // TODO make robust against if there's only 1 extruder
    
    
    size_t max_layer_count = 0;
    for (Slicer* mesh : volumes)
    {
        max_layer_count = std::max(max_layer_count, mesh->layers.size());
    }
    
    
    SparseCellGrid3D<Cell> grid(cell_size);

    populateGridWithBoundaryVoxels(volumes, grid);
    
    std::vector<Polygons> layer_regions(max_layer_count);
    std::vector<coord_t> layer_heights(max_layer_count);
    computeLayerRegions(volumes, layer_regions, layer_heights);

    std::vector<Polygons> layer_skins(max_layer_count);
    computeLayerSkins(layer_regions, layer_skins);
    
    removeBoundaryCells(grid, layer_regions, layer_heights);
    
    removeSkinCells(grid, layer_skins, layer_heights, cell_size);
    
    dilateCells(grid, extruder_count);
    
    cleanUpNonInterface(grid);
    
    std::vector<std::vector<Polygon>> cell_area_per_extruder_per_layer;
    generateMicrostructure(cell_area_per_extruder_per_layer, line_width_per_extruder, cell_size);

    applyMicrostructureToOutlines(grid, cell_area_per_extruder_per_layer, volumes, cell_size);
}

MultiVolumes::Cell::Cell()
: has_extruder(Application::getInstance().current_slice->scene.extruders.size(), false)
{
    assert(has_extruder.size() >= 1);
}

void MultiVolumes::populateGridWithBoundaryVoxels(const std::vector<Slicer*>& volumes, SparseCellGrid3D<Cell>& grid)
{
    const Cell default_cell;

    // mark all cells which contain some boundary
    for (Slicer* mesh : volumes)
    {
        size_t extruder_nr = mesh->mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").settings.get<size_t>("extruder_nr");
        for (size_t layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
        {
            SlicerLayer& layer = mesh->layers[layer_nr];
            coord_t z = layer.z;
            for (ConstPolygonRef poly : layer.polygons)
            {
                Point last = poly.back();
                for (Point p : poly)
                {
                    grid.processLineCells(std::make_pair(Point3(last.X, last.Y, z), Point3(p.X, p.Y, z)),
                                          [extruder_nr, &grid, &default_cell](SparseCellGrid3D<Cell>::GridPoint3 grid_loc) -> bool
                                          {
                                              Cell& cell = grid.getCell(grid_loc, default_cell);
                                              assert(extruder_nr < cell.has_extruder.size());
                                              cell.has_extruder[extruder_nr] = true;
                                              return true; // keep going marking cells along this line
                                          }
                    );
                    
                    last = p;
                }
            }
        }
    }
}

void MultiVolumes::computeLayerRegions(const std::vector<Slicer*>& volumes, std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights)
{
    for (unsigned int layer_nr = 0; layer_nr < layer_regions.size(); layer_nr++)
    {
        Polygons& layer_region = layer_regions[layer_nr];
        coord_t z;
        for (Slicer* mesh : volumes)
        {
            if (layer_nr >= mesh->layers.size()) break;
            SlicerLayer& layer = mesh->layers[layer_nr];
            z = layer.z;
            layer_region.add(layer.polygons);
        }
        layer_region = layer_region.offset(100).offset(-100); // TODO hardcoded value
        layer_heights[layer_nr] = z;
    }
}

void MultiVolumes::computeLayerSkins(const std::vector<Polygons>& layer_regions, std::vector<Polygons>& layer_skin)
{
    for (size_t layer_nr = 0; layer_nr < layer_regions.size(); layer_nr++)
    {
        Polygons empty;
        const Polygons& below = (layer_nr >= 1)? layer_regions[layer_nr - 1] : empty;
        const Polygons& above = (layer_nr + 1 < layer_regions.size())? layer_regions[layer_nr + 1] : empty;
        Polygons bottom_skin = layer_regions[layer_nr].difference(below);
        Polygons top_skin = layer_regions[layer_nr].difference(above);
        layer_skin[layer_nr] = bottom_skin.unionPolygons(top_skin);
    }
}

void MultiVolumes::removeBoundaryCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights)
{
    for (size_t layer_nr = 0; layer_nr < layer_regions.size(); layer_nr++)
    {
        coord_t z = layer_heights[layer_nr];
        for (ConstPolygonRef poly : layer_regions[layer_nr])
        {
            Point last = poly.back();
            for (Point p : poly)
            {
                grid.processLineCells(std::make_pair(Point3(last.X, last.Y, z), Point3(p.X, p.Y, z)),
                                      [&grid](SparseCellGrid3D<Cell>::GridPoint3 grid_loc) -> bool
                                      {
                                          for (coord_t offset_x : {-1, 0, 1})
                                              for (coord_t offset_y : {-1, 0, 1})
                                                  for (coord_t offset_z : {-1, 0, 1})
                                                  {
                                                      grid.removeCell(grid_loc + Point3(offset_x, offset_y, offset_z));
                                                  }
                                                  return true; // keep going removing cells
                                      }
                );
                last = p;
            }
        }
    }
}

void MultiVolumes::removeSkinCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_skin, std::vector<coord_t>& layer_heights, coord_t cell_size)
{
    for (size_t layer_nr = 0; layer_nr < layer_skin.size(); layer_nr++)
    {
        std::vector<Point> skin_points = PolygonUtils::spreadDotsArea(layer_skin[layer_nr], cell_size);
        for (Point skin_point : skin_points)
        {
            for (coord_t offset_x : {-1, 0, 1})
                for (coord_t offset_y : {-1, 0, 1})
                    for (coord_t offset_z : {-1, 0, 1})
                    {
                        grid.removeCell(grid.toGridPoint(Point3(skin_point.X, skin_point.Y, layer_heights[layer_nr])) + Point3(offset_x, offset_y, offset_z));
                    }
        }
    }
}

void MultiVolumes::dilateCells(SparseCellGrid3D<Cell>& grid, size_t extruder_count)
{
    std::vector<std::unordered_set<Point3>> dilated_cells_per_extruder(extruder_count);
    const std::vector<Point3> rel_dilation_locations = {
        Point3(-1,0,0), Point3(1,0,0),
        Point3(0,-1,0), Point3(0,1,0),
        Point3(0,0,-1), Point3(0,0,1) };
    for (auto& pair : grid.m_grid)
    {
        const Cell& cell = pair.second;
        for (size_t extruder_nr = 0; extruder_nr < cell.has_extruder.size(); extruder_nr++)
        {
            if (cell.has_extruder[extruder_nr])
            {
                for (Point3 rel_dilation_location : rel_dilation_locations)
                {
                    dilated_cells_per_extruder[extruder_nr].insert(pair.first + rel_dilation_location);
                }
            }
        }
    }

    const Cell default_cell;

    // apply dilation
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const std::unordered_set<Point3>& dilated_cells = dilated_cells_per_extruder[extruder_nr];
        for (Point3 grid_loc : dilated_cells)
        {
            grid.getCell(grid_loc, default_cell).has_extruder[extruder_nr] = true;
        }
    }
}

// remove cells which are not in the intersection region
void MultiVolumes::cleanUpNonInterface(SparseCellGrid3D<Cell>& grid)
{
    for (auto it = grid.m_grid.begin(); it != grid.m_grid.end(); )
    {
        int occupied_extruder_count = 0;
        const Cell& cell = it->second;
        for (bool has_extr : cell.has_extruder)
        {
            occupied_extruder_count += has_extr;
        }
        if (occupied_extruder_count <= 1)
        {
            it = grid.m_grid.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void MultiVolumes::generateMicrostructure(std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer, const std::vector<coord_t>& line_width_per_extruder, coord_t cell_size)
{
    cell_area_per_extruder_per_layer.resize(2);
    for (size_t layer_nr : {0, 1})
    {
        cell_area_per_extruder_per_layer[layer_nr].resize(line_width_per_extruder.size());
        for (size_t extruder_nr : {0, 1})
        {
            Point offset(extruder_nr? line_width_per_extruder[0] * 2 : 0, 0);
            Point area_size(line_width_per_extruder[extruder_nr] * 2, cell_size);
            if (layer_nr)
            {
                std::swap(offset.X, offset.Y);
                std::swap(area_size.X, area_size.Y);
            }
            PolygonRef poly = cell_area_per_extruder_per_layer[layer_nr][extruder_nr];
            poly.emplace_back(offset);
            poly.emplace_back(offset + Point(area_size.X, 0));
            poly.emplace_back(offset + area_size);
            poly.emplace_back(offset + Point(0, area_size.Y));
        }
    }
}

void MultiVolumes::applyMicrostructureToOutlines(SparseCellGrid3D<Cell>& grid, std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer, std::vector<Slicer*>& volumes, coord_t cell_size)
{
    for (auto key_val : grid.m_grid)
    {
        Point3 grid_loc = key_val.first;
        Point3 bottom_corner = grid.toLowerCorner(grid_loc);
        for (Slicer* mesh : volumes)
        {
            size_t extruder_nr = mesh->mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").settings.get<size_t>("extruder_nr");
            for (unsigned int layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
            {
                SlicerLayer& layer = mesh->layers[layer_nr];
                coord_t z = layer.z;
                if (z < bottom_corner.z) continue;
                if (z > bottom_corner.z + cell_size) break;
                
                Polygon area_here = cell_area_per_extruder_per_layer[layer_nr % 2][extruder_nr];
                Polygon area_other = cell_area_per_extruder_per_layer[layer_nr % 2][ ! extruder_nr];
                
                area_here.translate(Point(bottom_corner.x, bottom_corner.y));
                area_other.translate(Point(bottom_corner.x, bottom_corner.y));
                
                Polygons areas_here;
                areas_here.add(area_here);
                Polygons areas_other;
                areas_other.add(area_other);
                
                layer.polygons = layer.polygons.unionPolygons(areas_here).difference(areas_other);
            }
        }
    }
}

}//namespace cura
