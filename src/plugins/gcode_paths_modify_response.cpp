// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/gcode_paths_modify_response.h"

#include <unordered_map>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

#include "SliceMeshStorage.h"
#include "path_planning/SpeedDerivatives.h"

namespace curaengine::plugins
{
[[nodiscard]] constexpr PrintFeatureType gcode_paths_modify_response::getPrintFeatureType(const ::cura::plugins::v0::PrintFeature feature) noexcept
{
    switch (feature)
    {
    case ::cura::plugins::v0::PrintFeature::NONETYPE:
        return PrintFeatureType::NoneType;
    case ::cura::plugins::v0::PrintFeature::OUTERWALL:
        return PrintFeatureType::OuterWall;
    case ::cura::plugins::v0::PrintFeature::INNERWALL:
        return PrintFeatureType::InnerWall;
    case ::cura::plugins::v0::PrintFeature::SKIN:
        return PrintFeatureType::Skin;
    case ::cura::plugins::v0::PrintFeature::SUPPORT:
        return PrintFeatureType::Support;
    case ::cura::plugins::v0::PrintFeature::SKIRTBRIM:
        return PrintFeatureType::SkirtBrim;
    case ::cura::plugins::v0::PrintFeature::INFILL:
        return PrintFeatureType::Infill;
    case ::cura::plugins::v0::PrintFeature::SUPPORTINFILL:
        return PrintFeatureType::SupportInfill;
    case ::cura::plugins::v0::PrintFeature::MOVEUNRETRACTED:
        return PrintFeatureType::MoveUnretracted;
    case ::cura::plugins::v0::PrintFeature::MOVERETRACTED:
        return PrintFeatureType::MoveRetracted;
    case ::cura::plugins::v0::PrintFeature::SUPPORTINTERFACE:
        return PrintFeatureType::SupportInterface;
    case ::cura::plugins::v0::PrintFeature::PRIMETOWER:
        return PrintFeatureType::PrimeTower;
    case ::cura::plugins::v0::PrintFeature::MOVEWHILERETRACTING:
        return PrintFeatureType::MoveWhileRetracting;
    case ::cura::plugins::v0::PrintFeature::MOVEWHILEUNRETRACTING:
        return PrintFeatureType::MoveWhileUnretracting;
    case ::cura::plugins::v0::PrintFeature::STATIONARYRETRACTUNRETRACT:
        return PrintFeatureType::StationaryRetractUnretract;
    case ::cura::plugins::v0::PrintFeature::NUMPRINTFEATURETYPES:
        return PrintFeatureType::NumPrintFeatureTypes;
    default:
        return PrintFeatureType::NoneType;
    }
}

[[nodiscard]] constexpr SpaceFillType gcode_paths_modify_response::getSpaceFillType(const ::cura::plugins::v0::SpaceFillType space_fill_type) noexcept
{
    switch (space_fill_type)
    {
    case ::cura::plugins::v0::SpaceFillType::NONE:
        return SpaceFillType::None;
    case ::cura::plugins::v0::SpaceFillType::POLYGONS:
        return SpaceFillType::Polygons;
    case ::cura::plugins::v0::SpaceFillType::POLY_LINES:
        return SpaceFillType::PolyLines;
    case ::cura::plugins::v0::SpaceFillType::LINES:
        return SpaceFillType::Lines;
    default:
        return SpaceFillType::None;
    }
}

[[nodiscard]] GCodePathConfig gcode_paths_modify_response::buildConfig(const ::cura::plugins::v0::GCodePath& path)
{
    return {
        .z_offset = path.z_offset(),
        .type = getPrintFeatureType(path.feature()),
        .line_width = path.line_width(),
        .layer_thickness = path.layer_thickness(),
        .flow = path.flow_ratio(),
        .speed_derivatives
        = ::curaengine::path_planning::SpeedDerivatives{ .speed = path.speed_derivatives().velocity(), .acceleration = path.speed_derivatives().acceleration(), .jerk = path.speed_derivatives().jerk() },
        .is_bridge_path = path.is_bridge_path(),
        .fan_speed = path.fan_speed(),
        .temperature_delta = path.temperature_delta(),
    };
}

gcode_paths_modify_response::native_value_type
    gcode_paths_modify_response::operator()(gcode_paths_modify_response::native_value_type& original_value, const gcode_paths_modify_response::value_type& message) const
{
    std::vector<::curaengine::path_planning::GCodePath> paths;
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
        ::curaengine::path_planning::GCodePath path{
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
                              return ::curaengine::geometry::Point3LL{ point_msg.x(), point_msg.y(), point_msg.z() };
                          })
                    | ranges::to_vector;

        paths.emplace_back(path);
    }

    return paths;
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
