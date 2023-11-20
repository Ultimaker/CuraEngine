// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/MeshPathConfigs.h"

#include "ExtruderTrain.h"
#include "PrintFeature.h"

#include <range/v3/view/iota.hpp>

namespace cura
{

MeshPathConfigs::MeshPathConfigs(const SliceMeshStorage& mesh, const coord_t layer_thickness, const LayerIndex layer_nr, const std::vector<Ratio>& line_width_factor_per_extruder)
    : inset0_config{ .type = PrintFeatureType::OuterWall,
                     .line_width = static_cast<coord_t>(
                         mesh.settings.get<coord_t>("wall_line_width_0") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]),
                     .layer_thickness = layer_thickness,
                     .flow = mesh.settings.get<Ratio>("wall_0_material_flow") * (layer_nr == 0 ? mesh.settings.get<Ratio>("wall_0_material_flow_layer_0") : Ratio{ 1.0 }),
                     .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_wall_0"),
                                            .acceleration = mesh.settings.get<Acceleration>("acceleration_wall_0"),
                                            .jerk = mesh.settings.get<Velocity>("jerk_wall_0") } }
    , insetX_config{ .type = PrintFeatureType::InnerWall,
                     .line_width = static_cast<coord_t>(
                         mesh.settings.get<coord_t>("wall_line_width_x") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr]),
                     .layer_thickness = layer_thickness,
                     .flow = mesh.settings.get<Ratio>("wall_x_material_flow") * (layer_nr == 0 ? mesh.settings.get<Ratio>("wall_x_material_flow_layer_0") : Ratio{ 1.0 }),
                     .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_wall_x"),
                                            .acceleration = mesh.settings.get<Acceleration>("acceleration_wall_x"),
                                            .jerk = mesh.settings.get<Velocity>("jerk_wall_x") } }
    , inset0_roofing_config{ .type = PrintFeatureType::OuterWall,
                             .line_width = static_cast<coord_t>(
                                 mesh.settings.get<coord_t>("wall_line_width_0")
                                 * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]),
                             .layer_thickness = layer_thickness,
                             .flow
                             = mesh.settings.get<Ratio>("wall_0_material_flow_roofing") * (layer_nr == 0 ? mesh.settings.get<Ratio>("wall_0_material_flow_layer_0") : Ratio{ 1.0 }),
                             .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_wall_0_roofing"),
                                                    .acceleration = mesh.settings.get<Acceleration>("acceleration_wall_0_roofing"),
                                                    .jerk = mesh.settings.get<Velocity>("jerk_wall_0_roofing") } }
    , insetX_roofing_config{ .type = PrintFeatureType::InnerWall,
                             .line_width = static_cast<coord_t>(
                                 mesh.settings.get<coord_t>("wall_line_width_x")
                                 * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr]),
                             .layer_thickness = layer_thickness,
                             .flow
                             = mesh.settings.get<Ratio>("wall_x_material_flow_roofing") * (layer_nr == 0 ? mesh.settings.get<Ratio>("wall_x_material_flow_layer_0") : Ratio{ 1.0 }),
                             .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_wall_x_roofing"),
                                                    .acceleration = mesh.settings.get<Acceleration>("acceleration_wall_x_roofing"),
                                                    .jerk = mesh.settings.get<Velocity>("jerk_wall_x_roofing") } }
    , bridge_inset0_config{ .type = PrintFeatureType::OuterWall,
                            .line_width = static_cast<coord_t>(
                                mesh.settings.get<coord_t>("wall_line_width_0")
                                * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]),
                            .layer_thickness = layer_thickness,
                            .flow = mesh.settings.get<Ratio>("bridge_wall_material_flow"),
                            .speed_derivatives = { .speed = mesh.settings.get<Velocity>("bridge_wall_speed"),
                                                   .acceleration = mesh.settings.get<Acceleration>("acceleration_wall_0"),
                                                   .jerk = mesh.settings.get<Velocity>("jerk_wall_0") },
                            .is_bridge_path = true,
                            .fan_speed = mesh.settings.get<Ratio>("bridge_fan_speed") * 100.0 }
    , bridge_insetX_config{ .type = PrintFeatureType::InnerWall,
                            .line_width = static_cast<coord_t>(
                                mesh.settings.get<coord_t>("wall_line_width_x")
                                * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr]),
                            .layer_thickness = layer_thickness,
                            .flow = mesh.settings.get<Ratio>("bridge_wall_material_flow"),
                            .speed_derivatives = { .speed = mesh.settings.get<Velocity>("bridge_wall_speed"),
                                                   .acceleration = mesh.settings.get<Acceleration>("acceleration_wall_x"),
                                                   .jerk = mesh.settings.get<Velocity>("jerk_wall_x") },
                            .is_bridge_path = true,
                            .fan_speed = mesh.settings.get<Ratio>("bridge_fan_speed") * 100.0 }
    , skin_config{ .type = PrintFeatureType::Skin,
                   .line_width = static_cast<coord_t>(
                       mesh.settings.get<coord_t>("skin_line_width") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]),
                   .layer_thickness = layer_thickness,
                   .flow = mesh.settings.get<Ratio>("skin_material_flow") * (layer_nr == 0 ? mesh.settings.get<Ratio>("skin_material_flow_layer_0") : Ratio{ 1.0 }),
                   .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_topbottom"),
                                          .acceleration = mesh.settings.get<Acceleration>("acceleration_topbottom"),
                                          .jerk = mesh.settings.get<Velocity>("jerk_topbottom") } }
    , bridge_skin_config{ .type = PrintFeatureType::Skin,
                          .line_width = static_cast<coord_t>(
                              mesh.settings.get<coord_t>("skin_line_width")
                              * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]),
                          .layer_thickness = layer_thickness,
                          .flow = mesh.settings.get<Ratio>("bridge_skin_material_flow"),
                          .speed_derivatives = { .speed = mesh.settings.get<Velocity>("bridge_skin_speed"),
                                                 .acceleration = mesh.settings.get<Acceleration>("acceleration_topbottom"),
                                                 .jerk = mesh.settings.get<Velocity>("jerk_topbottom") },
                          .is_bridge_path = true,
                          .fan_speed = mesh.settings.get<Ratio>("bridge_fan_speed") * 100.0 }
    , bridge_skin_config2{ .type = PrintFeatureType::Skin,
                           .line_width = static_cast<coord_t>(
                               mesh.settings.get<coord_t>("skin_line_width")
                               * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]),
                           .layer_thickness = layer_thickness,
                           .flow = mesh.settings.get<Ratio>("bridge_skin_material_flow_2"),
                           .speed_derivatives = { .speed = mesh.settings.get<Velocity>("bridge_skin_speed_2"),
                                                  .acceleration = mesh.settings.get<Acceleration>("acceleration_topbottom"),
                                                  .jerk = mesh.settings.get<Velocity>("jerk_topbottom") },
                           .is_bridge_path = true,
                           .fan_speed = mesh.settings.get<Ratio>("bridge_fan_speed_2") * 100.0 }
    , bridge_skin_config3{ .type = PrintFeatureType::Skin,
                           .line_width = static_cast<coord_t>(
                               mesh.settings.get<coord_t>("skin_line_width")
                               * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]),
                           .layer_thickness = layer_thickness,
                           .flow = mesh.settings.get<Ratio>("bridge_skin_material_flow_3"),
                           .speed_derivatives = { .speed = mesh.settings.get<Velocity>("bridge_skin_speed_3"),
                                                  .acceleration = mesh.settings.get<Acceleration>("acceleration_topbottom"),
                                                  .jerk = mesh.settings.get<Velocity>("jerk_topbottom") },
                           .is_bridge_path = true,
                           .fan_speed = mesh.settings.get<Ratio>("bridge_fan_speed_3") * 100.0 }
    , roofing_config{ .type = PrintFeatureType::Skin,
                      .line_width = mesh.settings.get<coord_t>("roofing_line_width"),
                      .layer_thickness = layer_thickness,
                      .flow = mesh.settings.get<Ratio>("roofing_material_flow") * (layer_nr == 0 ? mesh.settings.get<Ratio>("material_flow_layer_0") : Ratio{ 1.0 }),
                      .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_roofing"),
                                             .acceleration = mesh.settings.get<Acceleration>("acceleration_roofing"),
                                             .jerk = mesh.settings.get<Velocity>("jerk_roofing") } }
    , ironing_config{ .type = PrintFeatureType::Skin,
                      .line_width = mesh.settings.get<coord_t>("ironing_line_spacing"),
                      .layer_thickness = layer_thickness,
                      .flow = mesh.settings.get<Ratio>("ironing_flow"),
                      .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_ironing"),
                                             .acceleration = mesh.settings.get<Acceleration>("acceleration_ironing"),
                                             .jerk = mesh.settings.get<Velocity>("jerk_ironing") } }

{
    infill_config.reserve(MAX_INFILL_COMBINE);

    for (const auto combine_idx : ranges::views::iota(1, MAX_INFILL_COMBINE + 1))
    {
        infill_config.emplace_back(GCodePathConfig{
            .type = PrintFeatureType::Infill,
            .line_width = static_cast<coord_t>(
                mesh.settings.get<coord_t>("infill_line_width") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr]),
            .layer_thickness = layer_thickness,
            .flow = mesh.settings.get<Ratio>("infill_material_flow") * (layer_nr == 0 ? mesh.settings.get<Ratio>("material_flow_layer_0") : Ratio{ 1.0 }) * combine_idx,
            .speed_derivatives = { .speed = mesh.settings.get<Velocity>("speed_infill"),
                                   .acceleration = mesh.settings.get<Acceleration>("acceleration_infill"),
                                   .jerk = mesh.settings.get<Velocity>("jerk_infill") } });
    }
}

} // namespace cura