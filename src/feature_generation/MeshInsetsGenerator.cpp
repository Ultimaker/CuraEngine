// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/MeshInsetsGenerator.h"

#include <ExtruderTrain.h>
#include <WallToolPaths.h>
#include <settings/PathConfigStorage.h>

#include <range/v3/algorithm/find_if.hpp>

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/ExtrusionMove.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/WallFeatureExtrusion.h"
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
    const SliceDataStorage& /*storage*/,
    const LayerPlanPtr& layer_plan,
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
    const MeshPathConfigs& mesh_configs = layer_plan->getConfigsStorage()->mesh_configs.at(getMesh());

    const size_t extruder_nr_inner_walls = mesh_settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr_;
    ExtruderPlanPtr extruder_plan_inner_walls = ExtruderPlan::find(extruder_plans, extruder_nr_inner_walls);
    assert(extruder_plan_inner_walls && "Unable to find extruder plan for inner walls");

    std::vector<std::shared_ptr<WallFeatureExtrusion>> feature_extrusions;

    auto find_or_make_feature_extrusion = [this, &feature_extrusions, &mesh_configs](const size_t inset_index)
    {
        auto iterator = ranges::find_if(
            feature_extrusions,
            [&inset_index](const std::shared_ptr<WallFeatureExtrusion>& feature_extrusion)
            {
                return feature_extrusion->getInsetIndex() == inset_index;
            });

        if (iterator != feature_extrusions.end())
        {
            return *iterator;
        }

        const GCodePathConfig& config = inset_index == 0 ? mesh_configs.inset0_config : mesh_configs.insetX_config;

        auto feature_extrusion
            = std::make_shared<WallFeatureExtrusion>(inset_index == 0 ? PrintFeatureType::OuterWall : PrintFeatureType::InnerWall, config.line_width, getMesh(), inset_index);
        feature_extrusions.push_back(feature_extrusion);
        return feature_extrusion;
    };

    for (const VariableWidthLines& toolpath : part.wall_toolpaths)
    {
        for (const ExtrusionLine& extrusion_line : toolpath)
        {
            if (extrusion_line.empty())
            {
                continue;
            }

            auto move_sequence = std::make_shared<ContinuousExtruderMoveSequence>(extrusion_line.is_closed_, extrusion_line.front().p_);

            for (const auto& extrusion_junctions : extrusion_line.junctions_ | ranges::views::sliding(2))
            {
                const ExtrusionJunction& start = extrusion_junctions[0];
                const ExtrusionJunction& end = extrusion_junctions[1];

                const GCodePathConfig& config = extrusion_line.inset_idx_ == 0 ? mesh_configs.inset0_config : mesh_configs.insetX_config;

                move_sequence->appendExtruderMove(std::make_shared<ExtrusionMove>(end.p_, start.w_, config.getSpeed(), end.w_));
            }

            find_or_make_feature_extrusion(extrusion_line.inset_idx_)->appendExtruderMoveSequence(move_sequence);
        }
    }

    for (const std::shared_ptr<WallFeatureExtrusion>& feature_extrusion : feature_extrusions)
    {
        extruder_plan_outer_walls->appendFeatureExtrusion(feature_extrusion);
    }

    // if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && extruder_nr ==
    // mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_)
    // {
    //     addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
    // }
}

} // namespace cura