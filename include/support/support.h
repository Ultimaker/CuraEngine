// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H
#define CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H

#include <cmath>
#include <memory>
#include <string>

#include <range/v3/back.hpp>
#include <range/v3/front.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/transform.hpp>

#include "sliceDataStorage.h"
#include "support/support_area.h"
#include "utils/polygon.h"

namespace cura::support
{
coord_t ModelSlopeDistance(const Settings& settings, const std::string& setting_angle_key)
{
    const auto layer_height = settings.get<coord_t>("layer_height");
    const auto support_angle = settings.get<AngleRadians>(setting_angle_key);
    const auto tan_angle = std::tan(support_angle) - 0.01; // The X/Y component of the support angle. 0.01 to make 90 degrees work too.
    return static_cast<coord_t>(tan_angle) * layer_height; // Maximum horizontal distance that can be bridged.
}

SupportArea MakeModelSupportAreaDiff(const auto& layers_window, const coord_t offset, std::shared_ptr<SliceMeshStorage> mesh, support::SupportAreaType area_type)
{
    const Polygons model_outline = std::get<1>(ranges::front(layers_window)).getOutlines();
    const Polygons other_model_outline = std::get<1>(ranges::back(layers_window)).getOutlines();
    const Polygons model_diff_outline = model_outline.difference(other_model_outline.offset(offset));
    const auto layer_idx = std::get<0>(ranges::front(layers_window));
    return { .mesh = mesh, .layer_idx = layer_idx, .outline = std::make_shared<Polygons>(model_diff_outline), .area_type = area_type };
}

namespace views
{
auto ComputeOverhang(std::shared_ptr<SliceMeshStorage> mesh)
{
    const auto max_dist_from_lower_layer = ModelSlopeDistance(mesh->settings, "support_angle");
    auto mesh_layer_outline_window = mesh->layers
                                   | ranges::views::enumerate
                                   | ranges::views::reverse
                                   | ranges::views::sliding(2)
                                   | ranges::views::transform([mesh, max_dist_from_lower_layer](const auto& layers_window) { return MakeModelSupportAreaDiff(layers_window, max_dist_from_lower_layer, mesh, support::SupportAreaType::OVERHANG); })
                                   | ranges::views::reverse;
    return ranges::make_view_closure(mesh_layer_outline_window);
}

auto ComputeFoundation(std::shared_ptr<SliceMeshStorage> mesh)
{
    const auto min_dist_from_upper_layer = ModelSlopeDistance(mesh->settings, "support_bottom_stair_step_min_slope");
    auto mesh_layer_outline_window = mesh->layers
                                   | ranges::views::enumerate
                                   | ranges::views::sliding(2)
                                   | ranges::views::transform([min_dist_from_upper_layer, mesh](const auto& layers_window) { return MakeModelSupportAreaDiff(layers_window, min_dist_from_upper_layer, mesh, support::SupportAreaType::FOUNDATION); });
    return ranges::make_view_closure(mesh_layer_outline_window);
}

constexpr auto supportable_meshes = ranges::views::filter([](std::shared_ptr<SliceMeshStorage> mesh) {
                                                              return mesh->settings.get<bool>("support_enable")
                                                                  && ! mesh->settings.get<bool>("anti_overhang_mesh")
                                                                  && ! mesh->settings.get<bool>("infill_mesh")
                                                                  && ! mesh->settings.get<bool>("support_mesh"); });

constexpr auto foundationable_meshes = ranges::views::filter([](std::shared_ptr<SliceMeshStorage> mesh) {
                                                                 return mesh->settings.get<bool>("support_enable")
                                                                     && ! mesh->settings.get<bool>("anti_overhang_mesh")
                                                                     && ! mesh->settings.get<bool>("infill_mesh"); });

constexpr auto meshes_overhangs = ranges::views::transform(&ComputeOverhang) | ranges::views::join;
constexpr auto meshes_foundation = ranges::views::transform(&ComputeFoundation) | ranges::views::join;

} // namespace views

} // namespace cura::support

#endif // CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H
