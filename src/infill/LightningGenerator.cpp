// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/LightningGenerator.h"

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "infill/LightningLayer.h"
#include "infill/LightningTreeNode.h"
#include "sliceDataStorage.h"
#include "utils/SparsePointGridInclusive.h"
#include "utils/linearAlg2D.h"

/* Possible future tasks/optimizations,etc.:
 * - Improve connecting heuristic to favor connecting to shorter trees
 * - Change which node of a tree is the root when that would be better in reconnectRoots.
 * - (For implementation in Infill classes & elsewhere): Outline offset, infill-overlap & perimeter gaps.
 * - Allow for polylines, i.e. merge Tims PR about polyline fixes
 * - Unit Tests?
 * - Optimization: let the square grid store the closest point on boundary
 * - Optimization: only compute the closest dist to / point on boundary for the outer cells and flood-fill the rest
 * - Make a pass with Arachne over the output. Somehow.
 * - Generate all to-be-supported points at once instead of sequentially: See branch interlocking_gen PolygonUtils::spreadDots (Or work with sparse grids.)
 * - Lots of magic values ... to many to parameterize. But are they the best?
 * - Move more complex computations from LightningGenerator constructor to elsewhere.
 */

using namespace cura;

LightningGenerator::LightningGenerator(const SliceMeshStorage& mesh)
{
    const auto infill_extruder = mesh.settings.get<ExtruderTrain&>("infill_extruder_nr");
    const auto layer_thickness = infill_extruder.settings_.get<coord_t>(
        "layer_height"); // Note: There's not going to be a layer below the first one, so the 'initial layer height' doesn't have to be taken into account.
    const auto infill_line_width = infill_extruder.settings_.get<coord_t>("infill_line_width");
    const auto infill_wall_line_count = static_cast<coord_t>(mesh.settings.get<size_t>("infill_wall_line_count"));
    const auto line_distance = infill_extruder.settings_.get<coord_t>("infill_line_distance");
    const auto overhang_angle = infill_extruder.settings_.get<AngleRadians>("lightning_infill_overhang_angle");
    const auto prune_angle = infill_extruder.settings_.get<AngleRadians>("lightning_infill_prune_angle");
    const auto straightening_angle = infill_extruder.settings_.get<AngleRadians>("lightning_infill_straightening_angle");

    std::vector<Shape> areas_per_layer;
    areas_per_layer.reserve(mesh.layers.size());
    for (const SliceLayer& current_layer : mesh.layers)
    {
        Shape infill_area_here;
        for (auto& part : current_layer.parts)
        {
            infill_area_here.push_back(part.getOwnInfillArea());
        }

        areas_per_layer.push_back(infill_area_here);
    }

    generate(layer_thickness, infill_line_width, infill_wall_line_count, line_distance, overhang_angle, prune_angle, straightening_angle, areas_per_layer);
}

LightningGenerator::LightningGenerator(const SupportStorage& support)
{
    if (! support.generated)
    {
        return;
    }

    const Settings& settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const auto support_extruder = settings.get<ExtruderTrain&>("support_extruder_nr");
    const Settings& extruder_settings = support_extruder.settings_;
    const auto layer_thickness = extruder_settings.get<coord_t>("layer_height");
    const auto infill_line_width = extruder_settings.get<coord_t>("support_line_width");
    const auto infill_wall_line_count = static_cast<coord_t>(settings.get<size_t>("support_wall_count"));
    const auto line_distance = extruder_settings.get<coord_t>("support_line_distance");
    const auto overhang_angle = extruder_settings.get<AngleRadians>("lightning_infill_overhang_angle");
    const auto prune_angle = extruder_settings.get<AngleRadians>("lightning_infill_prune_angle");
    const auto straightening_angle = extruder_settings.get<AngleRadians>("lightning_infill_straightening_angle");

    std::vector<Shape> areas_per_layer;
    areas_per_layer.reserve(support.supportLayers.size());
    for (const SupportLayer& support_layer : support.supportLayers)
    {
        Shape supper_area_here;
        for (const SupportInfillPart& part : support_layer.support_infill_parts)
        {
            supper_area_here.push_back(part.outline_);
        }

        areas_per_layer.push_back(supper_area_here);
    }

    generate(layer_thickness, infill_line_width, infill_wall_line_count, line_distance, overhang_angle, prune_angle, straightening_angle, areas_per_layer);
}

void LightningGenerator::generateInitialInternalOverhangs(const coord_t infill_line_width, const coord_t infill_wall_line_count, const std::vector<Shape>& shape_per_layer)
{
    overhang_per_layer.resize(shape_per_layer.size());
    const coord_t infill_wall_offset = -infill_wall_line_count * infill_line_width;

    Shape infill_area_above;
    // Iterate from top to bottom, to subtract the overhang areas above from the overhang areas on the layer below, to get only overhang in the top layer where it is overhanging.
    for (const auto& [layer_nr, layer_shape] : shape_per_layer | ranges::views::enumerate | ranges::views::reverse)
    {
        Shape infill_area_here = layer_shape.offset(infill_wall_offset);

        // Remove the part of the infill area that is already supported by the walls.
        Shape overhang = infill_area_here.offset(-wall_supporting_radius).difference(infill_area_above);

        overhang_per_layer[layer_nr] = overhang;
        infill_area_above = std::move(infill_area_here);
    }
}

const LightningLayer& LightningGenerator::getTreesForLayer(const size_t& layer_id) const
{
    assert(layer_id < lightning_layers.size());
    return lightning_layers[layer_id];
}

void LightningGenerator::generate(
    const coord_t layer_thickness,
    const coord_t line_width,
    const coord_t wall_line_count,
    const coord_t line_distance,
    const AngleRadians& overhang_angle,
    const AngleRadians& prune_angle,
    const AngleRadians& straightening_angle,
    const std::vector<Shape>& shape_per_layer)
{
    supporting_radius = std::max(line_distance, line_width) / 2;
    wall_supporting_radius = layer_thickness * std::tan(overhang_angle);
    prune_length = layer_thickness * std::tan(prune_angle);
    straightening_max_distance = layer_thickness * std::tan(straightening_angle);

    generateInitialInternalOverhangs(line_width, wall_line_count, shape_per_layer);
    generateTrees(line_width, wall_line_count, shape_per_layer);
}

void LightningGenerator::generateTrees(const coord_t infill_line_width, const coord_t infill_wall_line_count, const std::vector<Shape>& shape_per_layer)
{
    lightning_layers.resize(shape_per_layer.size());
    const coord_t infill_wall_offset = -infill_wall_line_count * infill_line_width;

    std::vector<Shape> infill_outlines;
    infill_outlines.insert(infill_outlines.end(), shape_per_layer.size(), Shape());

    // For-each layer from top to bottom:
    for (const auto& [layer_nr, layer_shape] : shape_per_layer | ranges::views::enumerate | ranges::views::reverse)
    {
        infill_outlines[layer_nr].push_back(layer_shape.offset(infill_wall_offset));
    }

    // For various operations its beneficial to quickly locate nearby features on the polygon:
    const size_t top_layer_id = shape_per_layer.size() - 1;
    auto outlines_locator_ptr = PolygonUtils::createLocToLineGrid(infill_outlines[top_layer_id], locator_cell_size);

    // For-each layer from top to bottom:
    for (int layer_id = top_layer_id; layer_id >= 0; layer_id--)
    {
        LightningLayer& current_lightning_layer = lightning_layers[layer_id];
        Shape& current_outlines = infill_outlines[layer_id];
        const auto& outlines_locator = *outlines_locator_ptr;

        // register all trees propagated from the previous layer as to-be-reconnected
        std::vector<LightningTreeNodeSPtr> to_be_reconnected_tree_roots = current_lightning_layer.tree_roots;

        current_lightning_layer.generateNewTrees(overhang_per_layer[layer_id], current_outlines, outlines_locator, supporting_radius, wall_supporting_radius);

        current_lightning_layer.reconnectRoots(to_be_reconnected_tree_roots, current_outlines, outlines_locator, supporting_radius, wall_supporting_radius);

        // Initialize trees for next lower layer from the current one.
        if (layer_id == 0)
        {
            return;
        }
        const Shape& below_outlines = infill_outlines[layer_id - 1];
        outlines_locator_ptr = PolygonUtils::createLocToLineGrid(below_outlines, locator_cell_size);
        const auto& below_outlines_locator = *outlines_locator_ptr;

        std::vector<LightningTreeNodeSPtr>& lower_trees = lightning_layers[layer_id - 1].tree_roots;
        for (auto& tree : current_lightning_layer.tree_roots)
        {
            tree->propagateToNextLayer(lower_trees, below_outlines, below_outlines_locator, prune_length, straightening_max_distance, locator_cell_size / 2);
        }
    }
}
