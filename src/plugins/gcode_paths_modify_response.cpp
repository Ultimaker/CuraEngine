// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/gcode_paths_modify_response.h"

#include <unordered_map>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

#include "pathPlanning/SpeedDerivatives.h"
#include "SliceMeshStorage.h"

namespace cura::plugins
{
[[nodiscard]] constexpr PrintFeatureType gcode_paths_modify_response::getPrintFeatureType(const v0::PrintFeature feature) noexcept
{
    switch (feature)
    {
    case v0::PrintFeature::NONETYPE:
        return PrintFeatureType::NoneType;
    case v0::PrintFeature::OUTERWALL:
        return PrintFeatureType::OuterWall;
    case v0::PrintFeature::INNERWALL:
        return PrintFeatureType::InnerWall;
    case v0::PrintFeature::SKIN:
        return PrintFeatureType::Skin;
    case v0::PrintFeature::SUPPORT:
        return PrintFeatureType::Support;
    case v0::PrintFeature::SKIRTBRIM:
        return PrintFeatureType::SkirtBrim;
    case v0::PrintFeature::INFILL:
        return PrintFeatureType::Infill;
    case v0::PrintFeature::SUPPORTINFILL:
        return PrintFeatureType::SupportInfill;
    case v0::PrintFeature::MOVEUNRETRACTED:
        return PrintFeatureType::MoveUnretracted;
    case v0::PrintFeature::MOVERETRACTED:
        return PrintFeatureType::MoveRetracted;
    case v0::PrintFeature::SUPPORTINTERFACE:
        return PrintFeatureType::SupportInterface;
    case v0::PrintFeature::PRIMETOWER:
        return PrintFeatureType::PrimeTower;
    case v0::PrintFeature::MOVEWHILERETRACTING:
        return PrintFeatureType::MoveWhileRetracting;
    case v0::PrintFeature::MOVEWHILEUNRETRACTING:
        return PrintFeatureType::MoveWhileUnretracting;
    case v0::PrintFeature::STATIONARYRETRACTUNRETRACT:
        return PrintFeatureType::StationaryRetractUnretract;
    case v0::PrintFeature::NUMPRINTFEATURETYPES:
        return PrintFeatureType::NumPrintFeatureTypes;
    default:
        return PrintFeatureType::NoneType;
    }
}

[[nodiscard]] constexpr SpaceFillType gcode_paths_modify_response::getSpaceFillType(const v0::SpaceFillType space_fill_type) noexcept
{
    switch (space_fill_type)
    {
    case v0::SpaceFillType::NONE:
        return SpaceFillType::None;
    case v0::SpaceFillType::POLYGONS:
        return SpaceFillType::Polygons;
    case v0::SpaceFillType::POLY_LINES:
        return SpaceFillType::PolyLines;
    case v0::SpaceFillType::LINES:
        return SpaceFillType::Lines;
    default:
        return SpaceFillType::None;
    }
}

[[nodiscard]] GCodePathConfig gcode_paths_modify_response::buildConfig(const v0::GCodePath& path)
{
    return {
        .z_offset = path.z_offset(),
        .type = getPrintFeatureType(path.feature()),
        .line_width = path.line_width(),
        .layer_thickness = path.layer_thickness(),
        .flow = path.flow_ratio(),
        .speed_derivatives
        = SpeedDerivatives{ .speed = path.speed_derivatives().velocity(), .acceleration = path.speed_derivatives().acceleration(), .jerk = path.speed_derivatives().jerk() },
        .is_bridge_path = path.is_bridge_path(),
        .fan_speed = path.fan_speed(),
        .temperature_delta = path.temperature_delta(),
    };
}

gcode_paths_modify_response::native_value_type
    gcode_paths_modify_response::operator()(gcode_paths_modify_response::native_value_type& original_value, const gcode_paths_modify_response::value_type& message) const
{
    std::vector<GCodePath> paths;
    using map_t = std::unordered_map<std::string, std::shared_ptr<const SliceMeshStorage>>;
    auto meshes = original_value
                | ranges::views::filter(
                      [](const auto& path)
                      {
                          return path.mesh != nullptr;
                      })
                | ranges::views::transform(
                      [](const auto& path) -> map_t::value_type
                      {
                          return { path.mesh->mesh_name, path.mesh };
                      })
                | ranges::to<map_t>;

    for (const auto& gcode_path_msg : message.gcode_paths())
    {
        GCodePath path{
            .z_offset = gcode_path_msg.z_offset(),
            .config = buildConfig(gcode_path_msg),
            .mesh = gcode_path_msg.mesh_name().empty() ? nullptr : meshes.at(gcode_path_msg.mesh_name()),
            .space_fill_type = getSpaceFillType(gcode_path_msg.space_fill_type()),
            .flow = gcode_path_msg.flow(),
            .width_factor = gcode_path_msg.width_factor(),
            .spiralize = gcode_path_msg.spiralize(),
            .speed_factor = gcode_path_msg.speed_factor(),
            .speed_back_pressure_factor = gcode_path_msg.speed_back_pressure_factor(),
            .retract = gcode_path_msg.retract(),
            .retract_for_nozzle_switch = gcode_path_msg.retract_for_nozzle_switch(),
            .unretract_before_last_travel_move = gcode_path_msg.unretract_before_last_travel_move(),
            .perform_z_hop = gcode_path_msg.perform_z_hop(),
            .perform_prime = gcode_path_msg.perform_prime(),
            .done = gcode_path_msg.done(),
            .fan_speed = gcode_path_msg.fan_speed(),
        };

        path.points = gcode_path_msg.path().path()
                    | ranges::views::transform(
                          [](const auto& point_msg)
                          {
                              return Point3LL{ point_msg.x(), point_msg.y(), point_msg.z() };
                          })
                    | ranges::to_vector;

        paths.emplace_back(path);
    }

    return paths;
}
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
