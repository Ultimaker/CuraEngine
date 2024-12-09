// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/MeshInsetsGenerator.h"

#include <ExtruderTrain.h>
#include <WallToolPaths.h>

#include <range/v3/algorithm/find_if.hpp>

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "sliceDataStorage.h"

namespace cura
{

MeshInsetsGenerator::MeshInsetsGenerator(const std::shared_ptr<SliceMeshStorage>& mesh)
    : MeshFeatureGenerator(mesh)
{
}
bool MeshInsetsGenerator::isActive() const
{
    if (! MeshFeatureGenerator::isActive())
    {
        return false;
    }

    if (getMesh()->settings.get<size_t>("wall_line_count") == 0)
    {
        return false;
    }

    return true;
}

void MeshInsetsGenerator::generateFeatures(
    const SliceDataStorage& storage,
    const LayerIndex& layer_index,
    const std::vector<ExtruderPlanPtr>& extruder_plans,
    const SliceLayerPart& part) const
{
    // if (extruder_nr != mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ && extruder_nr !=
    // mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr_)
    // {
    //     return added_something;
    // }

    const Settings mesh_settings = getMesh()->settings;

    const size_t extruder_nr_outer_walls = mesh_settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_;
    ExtruderPlanPtr extruder_plan_outer_walls = ExtruderPlan::find(extruder_plans, extruder_nr_outer_walls);
    assert(extruder_plan_outer_walls && "Unable to find extruder plan for outer walls");
    auto feature_extrusion_outer_walls = std::make_shared<FeatureExtrusion>();

    const size_t extruder_nr_inner_walls = mesh_settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr_;
    ExtruderPlanPtr extruder_plan_inner_walls = ExtruderPlan::find(extruder_plans, extruder_nr_inner_walls);
    assert(extruder_plan_inner_walls && "Unable to find extruder plan for inner walls");
    auto feature_extrusion_inner_walls = std::make_shared<FeatureExtrusion>();

    for (const VariableWidthLines& toolpath : part.wall_toolpaths)
    {
        for (const ExtrusionLine& extrusion_line : toolpath)
        {
            auto move_sequence = std::make_shared<ContinuousExtruderMoveSequence>(extrusion_line.is_closed_);

            for (const ExtrusionJunction& extrusion_junction : extrusion_line)
            {
            }

            if (extrusion_line.is_outer_wall())
            {
                feature_extrusion_outer_walls->appendExtruderMoveSequence(move_sequence);
            }
            else
            {
                feature_extrusion_inner_walls->appendExtruderMoveSequence(move_sequence);
            }
        }
    }

    extruder_plan_outer_walls->appendFeatureExtrusion(feature_extrusion_outer_walls);
    extruder_plan_inner_walls->appendFeatureExtrusion(feature_extrusion_inner_walls);

    // if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && extruder_nr ==
    // mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_)
    // {
    //     addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
    // }
}

} // namespace cura