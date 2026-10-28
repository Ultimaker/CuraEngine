// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/gcode_paths_modify_request.h"

#include "SliceMeshStorage.h"

namespace curaengine::plugins
{
[[nodiscard]] constexpr ::cura::plugins::v0::SpaceFillType gcode_paths_modify_request::getSpaceFillType(const curaengine::SpaceFillType space_fill_type) noexcept
{
    switch (space_fill_type)
    {
    case SpaceFillType::None:
        return ::cura::plugins::v0::SpaceFillType::NONE;
    case SpaceFillType::Polygons:
        return ::cura::plugins::v0::SpaceFillType::POLYGONS;
    case SpaceFillType::PolyLines:
        return ::cura::plugins::v0::SpaceFillType::POLY_LINES;
    case SpaceFillType::Lines:
        return ::cura::plugins::v0::SpaceFillType::LINES;
    default:
        return ::cura::plugins::v0::SpaceFillType::NONE;
    }
}

[[nodiscard]] constexpr ::cura::plugins::v0::PrintFeature gcode_paths_modify_request::getPrintFeature(const curaengine::PrintFeatureType print_feature_type) noexcept
{
    switch (print_feature_type)
    {
    case PrintFeatureType::NoneType:
        return ::cura::plugins::v0::PrintFeature::NONETYPE;
    case PrintFeatureType::OuterWall:
        return ::cura::plugins::v0::PrintFeature::OUTERWALL;
    case PrintFeatureType::InnerWall:
        return ::cura::plugins::v0::PrintFeature::INNERWALL;
    case PrintFeatureType::Skin:
        return ::cura::plugins::v0::PrintFeature::SKIN;
    case PrintFeatureType::Support:
        return ::cura::plugins::v0::PrintFeature::SUPPORT;
    case PrintFeatureType::SkirtBrim:
        return ::cura::plugins::v0::PrintFeature::SKIRTBRIM;
    case PrintFeatureType::Infill:
        return ::cura::plugins::v0::PrintFeature::INFILL;
    case PrintFeatureType::SupportInfill:
        return ::cura::plugins::v0::PrintFeature::SUPPORTINFILL;
    case PrintFeatureType::MoveUnretracted:
        return ::cura::plugins::v0::PrintFeature::MOVEUNRETRACTED;
    case PrintFeatureType::MoveRetracted:
        return ::cura::plugins::v0::PrintFeature::MOVERETRACTED;
    case PrintFeatureType::SupportInterface:
        return ::cura::plugins::v0::PrintFeature::SUPPORTINTERFACE;
    case PrintFeatureType::PrimeTower:
        return ::cura::plugins::v0::PrintFeature::PRIMETOWER;
    case PrintFeatureType::MoveWhileRetracting:
        return ::cura::plugins::v0::PrintFeature::MOVEWHILERETRACTING;
    case PrintFeatureType::MoveWhileUnretracting:
        return ::cura::plugins::v0::PrintFeature::MOVEWHILEUNRETRACTING;
    case PrintFeatureType::StationaryRetractUnretract:
        return ::cura::plugins::v0::PrintFeature::STATIONARYRETRACTUNRETRACT;
    case PrintFeatureType::NumPrintFeatureTypes:
        return ::cura::plugins::v0::PrintFeature::NUMPRINTFEATURETYPES;
    default:
        return ::cura::plugins::v0::PrintFeature::NONETYPE;
    }
}

gcode_paths_modify_request::value_type
    gcode_paths_modify_request::operator()(const gcode_paths_modify_request::native_value_type& paths, const size_t extruder_nr, const LayerIndex layer_nr) const
{
    value_type message{};
    message.set_extruder_nr(static_cast<int64_t>(extruder_nr));
    message.set_layer_nr(layer_nr);

    // Construct the repeated GCodepath message
    auto* gcode_paths = message.mutable_gcode_paths();
    for (const auto& path : paths)
    {
        auto* gcode_path = gcode_paths->Add();
        // Construct the OpenPath from the points in a GCodePath
        for (const auto& point : path.points)
        {
            auto* points = gcode_path->mutable_path()->add_path();
            points->set_x(point.x_);
            points->set_y(point.y_);
            points->set_z(point.z_);
        }
        gcode_path->set_space_fill_type(getSpaceFillType(path.space_fill_type));
        gcode_path->set_flow(path.flow);
        gcode_path->set_width_factor(path.width_factor);
        gcode_path->set_spiralize(path.spiralize);
        gcode_path->set_speed_factor(path.speed_factor);
        gcode_path->set_speed_back_pressure_factor(path.speed_back_pressure_factor);
        gcode_path->set_retract(path.retract);
        gcode_path->set_retract_for_nozzle_switch(path.retract_for_nozzle_switch);
        gcode_path->set_unretract_before_last_travel_move(path.unretract_before_last_travel_move);
        gcode_path->set_perform_z_hop(path.perform_z_hop);
        gcode_path->set_perform_prime(path.perform_prime);
        gcode_path->set_skip_agressive_merge_hint(false); // FIXME: Not used anymore.
        gcode_path->set_done(path.done);
        gcode_path->set_fan_speed(path.getFanSpeed());
        gcode_path->set_mesh_name(path.mesh ? path.mesh->mesh_name : "");
        gcode_path->set_feature(getPrintFeature(path.config.type));
        gcode_path->mutable_speed_derivatives()->set_velocity(path.config.getSpeed());
        gcode_path->mutable_speed_derivatives()->set_acceleration(path.config.getAcceleration());
        gcode_path->mutable_speed_derivatives()->set_jerk(path.config.getJerk());
        gcode_path->set_line_width(path.config.getLineWidth());
        gcode_path->set_layer_thickness(path.config.getLayerThickness());
        gcode_path->set_flow_ratio(path.config.getFlowRatio());
        gcode_path->set_is_bridge_path(path.config.isBridgePath());
        gcode_path->set_z_offset(path.config.z_offset);
        gcode_path->set_temperature_delta(path.config.temperature_delta);
    }

    return message;
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
