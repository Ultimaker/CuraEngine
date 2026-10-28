// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "InterlockingGenerator.h"

#include <algorithm> // max

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/view.hpp>
#include <range/v3/view/zip.hpp>

#include "Application.h"
#include "Slice.h"
#include "Slicer.h"
#include "SlicerLayer.h"
#include "geometry/PointMatrix.h"
#include "types/LayerIndex.h"
#include "utils/DilationKernel.h"
#include "utils/PolygonUtils.h"
#include "utils/VoxelUtils.h"

namespace curaengine
{

// TODO: optimization: only go up to the min layer count + a couple of layers instead of max_layer_count

void InterlockingGenerator::generateInterlockingStructure(std::vector<Slicer*>& volumes)
{
    Settings& global_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const ::curaengine::geometry::PointMatrix rotation(global_settings.get<::curaengine::types::AngleDegrees>("interlocking_orientation"));
    const ::curaengine::utils::coord_t beam_layer_count = global_settings.get<int>("interlocking_beam_layer_count");
    const int interface_depth = global_settings.get<int>("interlocking_depth");
    const int boundary_avoidance = global_settings.get<int>("interlocking_boundary_avoidance");

    for (size_t mesh_a_idx = 0; mesh_a_idx < volumes.size(); mesh_a_idx++)
    {
        Slicer& mesh_a = *volumes[mesh_a_idx];
        size_t extruder_nr_a = mesh_a.mesh->settings_.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_;
        for (size_t mesh_b_idx = mesh_a_idx + 1; mesh_b_idx < volumes.size(); mesh_b_idx++)
        {
            Slicer& mesh_b = *volumes[mesh_b_idx];
            size_t extruder_nr_b = mesh_b.mesh->settings_.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_;

            if (! mesh_a.mesh->isModelMesh() || ! mesh_b.mesh->isModelMesh())
            {
                continue;
            }

            if (extruder_nr_a == extruder_nr_b || ! mesh_a.mesh->getAABB().expand(ignored_gap_).hit(mesh_b.mesh->getAABB()))
            {
                // early out for when meshes don't share any overlap in their bounding box
                continue;
            }

            ::curaengine::utils::coord_t beam_width_a = mesh_a.mesh->settings_.get<::curaengine::utils::coord_t>("interlocking_beam_width");
            ::curaengine::utils::coord_t beam_width_b = mesh_b.mesh->settings_.get<::curaengine::utils::coord_t>("interlocking_beam_width");

            // TODO: why are these two kernels different kernel types?!
            const ::curaengine::utils::DilationKernel interface_dilation(::curaengine::utils::GridPoint3(interface_depth, interface_depth, interface_depth), ::curaengine::utils::DilationKernel::Type::PRISM);

            const bool air_filtering = boundary_avoidance > 0;
            const ::curaengine::utils::DilationKernel air_dilation(::curaengine::utils::GridPoint3(boundary_avoidance, boundary_avoidance, boundary_avoidance), ::curaengine::utils::DilationKernel::Type::PRISM);

            const ::curaengine::utils::coord_t cell_width = beam_width_a + beam_width_b;
            const ::curaengine::geometry::Point3LL cell_size(cell_width, cell_width, 2 * beam_layer_count);

            InterlockingGenerator gen(mesh_a, mesh_b, beam_width_a, beam_width_b, rotation, cell_size, beam_layer_count, interface_dilation, air_dilation, air_filtering);
            gen.generateInterlockingStructure();
        }
    }
}

std::pair<::curaengine::geometry::Shape, ::curaengine::geometry::Shape> InterlockingGenerator::growBorderAreasPerpendicular(const ::curaengine::geometry::Shape& a, const ::curaengine::geometry::Shape& b, const ::curaengine::utils::coord_t& detect) const
{
    const ::curaengine::utils::coord_t min_line = std::min(mesh_a_.mesh->settings_.get<::curaengine::utils::coord_t>("min_wall_line_width"), mesh_b_.mesh->settings_.get<::curaengine::utils::coord_t>("min_wall_line_width"));

    const ::curaengine::geometry::Shape total_shrunk = a.offset(min_line).unionPolygons(b.offset(min_line)).offset(2 * -min_line);

    ::curaengine::geometry::Shape from_border_a = a.difference(total_shrunk);
    ::curaengine::geometry::Shape from_border_b = b.difference(total_shrunk);

    ::curaengine::geometry::Shape temp_a, temp_b;
    for (::curaengine::utils::coord_t i = 0; i < (detect / min_line) + 2; ++i)
    {
        temp_a = from_border_a.offset(min_line);
        temp_b = from_border_b.offset(min_line);
        from_border_a = temp_a.difference(temp_b);
        from_border_b = temp_b.difference(temp_a);
    }

    return { from_border_a, from_border_b };
}

void InterlockingGenerator::handleThinAreas(const std::unordered_set<::curaengine::utils::GridPoint3>& has_all_meshes) const
{
    Settings& global_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const ::curaengine::utils::coord_t boundary_avoidance = global_settings.get<int>("interlocking_boundary_avoidance");

    const ::curaengine::utils::coord_t number_of_beams_detect = boundary_avoidance;
    const ::curaengine::utils::coord_t number_of_beams_expand = boundary_avoidance - 1;
    constexpr ::curaengine::utils::coord_t rounding_errors = 5;

    const ::curaengine::utils::coord_t max_beam_width = std::max(beam_width_a_, beam_width_b_);
    const ::curaengine::utils::coord_t detect = (max_beam_width * number_of_beams_detect) + rounding_errors;
    const ::curaengine::utils::coord_t expand = (max_beam_width * number_of_beams_expand) + rounding_errors;
    const ::curaengine::utils::coord_t close_gaps = std::min(mesh_a_.mesh->settings_.get<::curaengine::utils::coord_t>("line_width"), mesh_b_.mesh->settings_.get<::curaengine::utils::coord_t>("line_width")) / 4;

    // Make an inclusionary polygon, to only actually handle thin areas near actual microstructures (so not in skin for example).
    std::vector<::curaengine::geometry::Shape> near_interlock_per_layer;
    near_interlock_per_layer.assign(std::min(mesh_a_.layers.size(), mesh_b_.layers.size()), ::curaengine::geometry::Shape());
    for (const auto& cell : has_all_meshes)
    {
        const ::curaengine::geometry::Point3LL bottom_corner = vu_.toLowerCorner(cell);
        for (::curaengine::utils::coord_t layer_nr = bottom_corner.z_; layer_nr < bottom_corner.z_ + cell_size_.z_ && layer_nr < static_cast<::curaengine::utils::coord_t>(near_interlock_per_layer.size()); ++layer_nr)
        {
            near_interlock_per_layer[static_cast<size_t>(layer_nr)].push_back(vu_.toPolygon(cell));
        }
    }
    for (auto& near_interlock : near_interlock_per_layer)
    {
        near_interlock = near_interlock.offset(rounding_errors).offset(-rounding_errors).unionPolygons().offset(detect);
        near_interlock.applyMatrix(rotation_.inverse());
    }

    // Only alter layers when they are present in both meshes, zip should take care if that.
    for (auto [layer_nr, layer] : ranges::views::zip(mesh_a_.layers, mesh_b_.layers) | ranges::views::enumerate)
    {
        ::curaengine::geometry::Shape& polys_a = std::get<0>(layer).polygons_;
        ::curaengine::geometry::Shape& polys_b = std::get<1>(layer).polygons_;

        const auto [from_border_a, from_border_b] = growBorderAreasPerpendicular(polys_a, polys_b, detect);

        // Get the areas of each mesh that are _not_ thin (large), by performing a morphological open.
        const ::curaengine::geometry::Shape large_a{ polys_a.offset(-detect).offset(detect) };
        const ::curaengine::geometry::Shape large_b{ polys_b.offset(-detect).offset(detect) };

        // Derive the area that the thin areas need to expand into (so the added areas to the thin strips) from the information we already have.
        const ::curaengine::geometry::Shape thin_expansion_a{
            large_b.intersection(polys_a.difference(large_a).offset(expand)).intersection(near_interlock_per_layer[layer_nr]).intersection(from_border_a).offset(rounding_errors)
        };
        const ::curaengine::geometry::Shape thin_expansion_b{
            large_a.intersection(polys_b.difference(large_b).offset(expand)).intersection(near_interlock_per_layer[layer_nr]).intersection(from_border_b).offset(rounding_errors)
        };

        // Expanded thin areas of the opposing polygon should 'eat into' the larger areas of the polygon,
        // and conversely, add the expansions to their own thin areas.
        polys_a = polys_a.unionPolygons(thin_expansion_a).difference(thin_expansion_b).offset(close_gaps).offset(-close_gaps);
        polys_b = polys_b.unionPolygons(thin_expansion_b).difference(thin_expansion_a).offset(close_gaps).offset(-close_gaps);
    }
}

void InterlockingGenerator::generateInterlockingStructure() const
{
    std::vector<std::unordered_set<::curaengine::utils::GridPoint3>> voxels_per_mesh = getShellVoxels(interface_dilation_);

    std::unordered_set<::curaengine::utils::GridPoint3>& has_any_mesh = voxels_per_mesh[0];
    std::unordered_set<::curaengine::utils::GridPoint3>& has_all_meshes = voxels_per_mesh[1];
    has_any_mesh.merge(has_all_meshes); // perform union and intersection simultaneously. Cannibalizes voxels_per_mesh

    const std::vector<::curaengine::geometry::Shape> layer_regions = computeUnionedVolumeRegions();

    if (air_filtering_)
    {
        std::unordered_set<::curaengine::utils::GridPoint3> air_cells;
        addBoundaryCells(layer_regions, air_dilation_, air_cells);

        for (const ::curaengine::utils::GridPoint3& p : air_cells)
        {
            has_all_meshes.erase(p);
        }

        handleThinAreas(has_all_meshes);
    }

    applyMicrostructureToOutlines(has_all_meshes, layer_regions);
}

std::vector<std::unordered_set<::curaengine::utils::GridPoint3>> InterlockingGenerator::getShellVoxels(const ::curaengine::utils::DilationKernel& kernel) const
{
    std::vector<std::unordered_set<::curaengine::utils::GridPoint3>> voxels_per_mesh(2);

    // mark all cells which contain some boundary
    for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
    {
        Slicer* mesh = (mesh_idx == 0) ? &mesh_a_ : &mesh_b_;
        std::unordered_set<::curaengine::utils::GridPoint3>& mesh_voxels = voxels_per_mesh[mesh_idx];

        std::vector<::curaengine::geometry::Shape> rotated_polygons_per_layer(mesh->layers.size());
        for (size_t layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
        {
            SlicerLayer& layer = mesh->layers[layer_nr];
            rotated_polygons_per_layer[layer_nr] = layer.polygons_;
            rotated_polygons_per_layer[layer_nr].applyMatrix(rotation_);
        }

        addBoundaryCells(rotated_polygons_per_layer, kernel, mesh_voxels);
    }

    return voxels_per_mesh;
}

void InterlockingGenerator::addBoundaryCells(const std::vector<::curaengine::geometry::Shape>& layers, const ::curaengine::utils::DilationKernel& kernel, std::unordered_set<::curaengine::utils::GridPoint3>& cells) const
{
    auto voxel_emplacer = [&cells](::curaengine::utils::GridPoint3 p)
    {
        cells.emplace(p);
        return true;
    };

    for (size_t layer_nr = 0; layer_nr < layers.size(); layer_nr++)
    {
        const ::curaengine::utils::coord_t z = static_cast<::curaengine::utils::coord_t>(layer_nr);
        vu_.walkDilatedPolygons(layers[layer_nr], z, kernel, voxel_emplacer);
        ::curaengine::geometry::Shape skin = layers[layer_nr];
        if (layer_nr > 0)
        {
            skin = skin.xorPolygons(layers[layer_nr - 1]);
        }
        skin = skin.offset(-cell_size_.x_ / 2).offset(cell_size_.x_ / 2); // remove superfluous small areas, which would anyway be included because of walkPolygons
        vu_.walkDilatedAreas(skin, z, kernel, voxel_emplacer);
    }
}

std::vector<::curaengine::geometry::Shape> InterlockingGenerator::computeUnionedVolumeRegions() const
{
    const auto max_layer_count = std::max(mesh_a_.layers.size(), mesh_b_.layers.size()) + 1; // introduce ghost layer on top for correct skin computation of topmost layer.
    std::vector<::curaengine::geometry::Shape> layer_regions(max_layer_count);

    for (::curaengine::types::LayerIndex layer_nr = 0; layer_nr < ::curaengine::types::LayerIndex(max_layer_count); layer_nr++)
    {
        ::curaengine::geometry::Shape& layer_region = layer_regions[static_cast<size_t>(layer_nr)];
        for (Slicer* mesh : { &mesh_a_, &mesh_b_ })
        {
            if (layer_nr >= mesh->layers.size())
            {
                break;
            }
            const SlicerLayer& layer = mesh->layers[static_cast<size_t>(layer_nr)];
            layer_region.push_back(layer.polygons_);
        }
        layer_region = layer_region.offset(ignored_gap_).offset(-ignored_gap_); // Morphological close to merge meshes into single volume
        layer_region.applyMatrix(rotation_);
    }
    return layer_regions;
}

std::vector<std::vector<::curaengine::geometry::Shape>> InterlockingGenerator::generateMicrostructure() const
{
    std::vector<std::vector<::curaengine::geometry::Shape>> cell_area_per_mesh_per_layer;
    cell_area_per_mesh_per_layer.resize(2);
    cell_area_per_mesh_per_layer[0].resize(2);
    const ::curaengine::utils::coord_t beam_w_sum = beam_width_a_ + beam_width_b_;
    const ::curaengine::utils::coord_t middle = cell_size_.x_ * beam_width_a_ / beam_w_sum;
    const ::curaengine::utils::coord_t width[2] = { middle, cell_size_.x_ - middle };
    for (size_t mesh_idx : { 0ul, 1ul })
    {
        ::curaengine::geometry::Point2LL offset(mesh_idx ? middle : 0, 0);
        ::curaengine::geometry::Point2LL area_size(width[mesh_idx], cell_size_.y_);

        ::curaengine::geometry::Polygon& poly = cell_area_per_mesh_per_layer[0][mesh_idx].newLine();
        poly.emplace_back(offset);
        poly.emplace_back(offset + ::curaengine::geometry::Point2LL(area_size.X, 0));
        poly.emplace_back(offset + area_size);
        poly.emplace_back(offset + ::curaengine::geometry::Point2LL(0, area_size.Y));
    }
    cell_area_per_mesh_per_layer[1] = cell_area_per_mesh_per_layer[0];
    for (::curaengine::geometry::Shape& polys : cell_area_per_mesh_per_layer[1])
    {
        for (::curaengine::geometry::Polygon& poly : polys)
        {
            for (::curaengine::geometry::Point2LL& p : poly)
            {
                std::swap(p.X, p.Y);
            }
        }
    }
    return cell_area_per_mesh_per_layer;
}

void InterlockingGenerator::applyMicrostructureToOutlines(const std::unordered_set<::curaengine::utils::GridPoint3>& cells, const std::vector<::curaengine::geometry::Shape>& layer_regions) const
{
    std::vector<std::vector<::curaengine::geometry::Shape>> cell_area_per_mesh_per_layer = generateMicrostructure();

    const ::curaengine::geometry::PointMatrix unapply_rotation = rotation_.inverse();
    const size_t max_layer_count = std::max(mesh_a_.layers.size(), mesh_b_.layers.size());

    std::vector<::curaengine::geometry::Shape> structure_per_layer[2]; // for each mesh the structure on each layer

    // Every `beam_layer_count` number of layers are combined to an interlocking beam layer
    // to store these we need ceil(max_layer_count / beam_layer_count) of these layers
    // the formula is rewritten as (max_layer_count + beam_layer_count - 1) / beam_layer_count, so it works for integer division
    size_t num_interlocking_layers = (max_layer_count + static_cast<size_t>(beam_layer_count_) - 1ul) / static_cast<size_t>(beam_layer_count_);
    structure_per_layer[0].resize(num_interlocking_layers);
    structure_per_layer[1].resize(num_interlocking_layers);

    // Only compute cell structure for half the layers, because since our beams are two layers high, every odd layer of the structure will be the same as the layer below.
    for (const ::curaengine::utils::GridPoint3& grid_loc : cells)
    {
        ::curaengine::geometry::Point3LL bottom_corner = vu_.toLowerCorner(grid_loc);
        for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
        {
            for (::curaengine::types::LayerIndex layer_nr = bottom_corner.z_; layer_nr < bottom_corner.z_ + cell_size_.z_ && layer_nr < max_layer_count; layer_nr += beam_layer_count_)
            {
                ::curaengine::geometry::Shape areas_here = cell_area_per_mesh_per_layer[static_cast<size_t>(layer_nr / beam_layer_count_) % cell_area_per_mesh_per_layer.size()][mesh_idx];
                areas_here.translate(::curaengine::geometry::Point2LL(bottom_corner.x_, bottom_corner.y_));
                structure_per_layer[mesh_idx][static_cast<size_t>(layer_nr / beam_layer_count_)].push_back(areas_here);
            }
        }
    }

    for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
    {
        for (size_t layer_nr = 0; layer_nr < structure_per_layer[mesh_idx].size(); layer_nr++)
        {
            ::curaengine::geometry::Shape& layer_structure = structure_per_layer[mesh_idx][layer_nr];
            layer_structure = layer_structure.unionPolygons();
            layer_structure.applyMatrix(unapply_rotation);
        }
    }

    for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
    {
        Slicer* mesh = (mesh_idx == 0) ? &mesh_a_ : &mesh_b_;
        for (size_t layer_nr = 0; layer_nr < max_layer_count; layer_nr++)
        {
            if (layer_nr >= mesh->layers.size())
            {
                break;
            }

            ::curaengine::geometry::Shape layer_outlines = layer_regions[layer_nr];
            layer_outlines.applyMatrix(unapply_rotation);

            const ::curaengine::geometry::Shape areas_here = structure_per_layer[mesh_idx][layer_nr / static_cast<size_t>(beam_layer_count_)].intersection(layer_outlines);
            const ::curaengine::geometry::Shape& areas_other = structure_per_layer[! mesh_idx][layer_nr / static_cast<size_t>(beam_layer_count_)];

            SlicerLayer& layer = mesh->layers[layer_nr];
            layer.polygons_ = layer.polygons_
                                  .difference(areas_other) // reduce layer areas inward with beams from other mesh
                                  .unionPolygons(areas_here); // extend layer areas outward with newly added beams
        }
    }
}

} // namespace curaengine
