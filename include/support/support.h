// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H
#define CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <range/v3/back.hpp>
#include <range/v3/empty.hpp>
#include <range/v3/front.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/take_while.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>

#include "sliceDataStorage.h"
#include "support/support_area.h"
#include "utils/Simplify.h"
#include "utils/polygon.h"
#include "utils/views/to_shared_ptr.h"

namespace cura::support
{
using support_area_graph_t = std::unordered_multimap<std::shared_ptr<SupportArea>, std::shared_ptr<SupportArea>>;

coord_t ModelSlopeDistance(const Settings& settings, const std::string& setting_angle_key)
{
    const auto layer_height = settings.get<coord_t>("layer_height");
    const auto support_angle = settings.get<AngleRadians>(setting_angle_key);
    const auto tan_angle = std::tan(support_angle) - 0.01; // The X/Y component of the support angle. 0.01 to make 90 degrees work too.
    return static_cast<coord_t>(tan_angle) * layer_height; // Maximum horizontal distance that can be bridged.
}

SupportArea MakeModelSupportAreaDiff(const auto& layers_window, const coord_t offset, std::shared_ptr<SliceMeshStorage> mesh, support::SupportAreaType area_type, const Simplify& simplify)
{
    Polygons& model_outline = std::get<1>(ranges::front(layers_window)).Outlines();
    Polygons& other_model_outline = std::get<1>(ranges::back(layers_window)).Outlines();
    const Polygons model_diff_outline = simplify.polygon(model_outline.difference(other_model_outline.offset(offset)));
    const auto layer_idx = std::get<0>(ranges::front(layers_window));
    return { .mesh = mesh, .layer_idx = layer_idx, .outline = std::make_shared<Polygons>(model_diff_outline), .area_type = area_type, .area = model_diff_outline.area(), .bounding_box = AABB { model_diff_outline } };
}

namespace views
{
auto ComputeOverhang(std::shared_ptr<SliceMeshStorage> mesh)
{
    spdlog::get("support")->info("Compute support overhangs for {}", mesh->mesh_name);
    const auto max_dist_from_lower_layer = ModelSlopeDistance(mesh->settings, "support_angle");
    const auto minimum_support_area = mesh->settings.get<double>("minimum_support_area");
    const Simplify simplify(mesh->settings);
    auto mesh_layer_outline_window = mesh->layers
                                   | ranges::views::enumerate
                                   | ranges::views::reverse
                                   | ranges::views::sliding(2)
                                   | ranges::views::transform([mesh, max_dist_from_lower_layer, simplify](const auto& layers_window) { return MakeModelSupportAreaDiff(layers_window, max_dist_from_lower_layer, mesh, support::SupportAreaType::OVERHANG, simplify); })
                                   | ranges::views::reverse
                                   | ranges::views::filter([minimum_support_area](const SupportArea& support_area){ return support_area.area >= minimum_support_area; });
    return ranges::make_view_closure(mesh_layer_outline_window);
}

auto ComputeFoundation(std::shared_ptr<SliceMeshStorage> mesh)
{
    spdlog::get("support")->info("Compute support foundations for {}", mesh->mesh_name);
    const auto min_dist_from_upper_layer = ModelSlopeDistance(mesh->settings, "support_bottom_stair_step_min_slope");
    const Simplify simplify(mesh->settings);
    auto mesh_layer_outline_window = mesh->layers
                                   | ranges::views::enumerate
                                   | ranges::views::sliding(2)
                                   | ranges::views::transform([min_dist_from_upper_layer, mesh, simplify](const auto& layers_window) { return MakeModelSupportAreaDiff(layers_window, min_dist_from_upper_layer, mesh, support::SupportAreaType::FOUNDATION, simplify); });
    return ranges::make_view_closure(mesh_layer_outline_window);
}

constexpr auto drop_down(auto&& overhangs, auto&& foundations)
{
    spdlog::get("support")->info("Dropdown support from overhangs to foundations");
    auto foundations_ = foundations | ranges::to_vector; // Caching the creation of foundation, since these were still a lazy view at this time (TODO: check if this is actually needed)

    // Create the support forest, which is a view of `support_area_graph_t::value_type` (linking area's top-down) of naively dropped down overhangs, designated as `SUPPORT`, going down to either a foundation or the build plate.
    // Note at this time the offsets and subtractions or intersection with the model isn't applied yet.
    auto support_forest = overhangs
                        | ranges::views::transform([&foundations, foundations_](auto overhang) {
                                                       auto layers_below = ranges::views::iota(0UL, overhang->layer_idx)
                                                                         | ranges::views::reverse
                                                                         | ranges::views::transform([overhang, foundations_](auto layer_idx) -> SupportArea {
                                                                                                       // Create a support area based on the foundation above it on the current layer
                                                                                                        SupportArea support_area {
                                                                                                            .mesh = overhang->mesh,
                                                                                                            .layer_idx = layer_idx,
                                                                                                            .outline = overhang->outline,
                                                                                                            .area_type = SupportAreaType::SUPPORT,
                                                                                                            .area = overhang->area,
                                                                                                            .bounding_box = overhang->bounding_box
                                                                                                        };

                                                                                                        // Check against collision with foundations; if collided: alter the outline with that foundation and update area and bounding_box, if completely eaten away set type to NONE
                                                                                                        auto intersecting_foundations = foundations_
                                                                                                                                      | ranges::views::filter([layer_idx](auto foundation){ return foundation->layer_idx == layer_idx; })
                                                                                                                                      | ranges::views::filter([&support_area](auto foundation){ return support_area.bounding_box.hit(foundation->bounding_box); });
                                                                                                        if (! ranges::empty(intersecting_foundations))
                                                                                                        {
                                                                                                            Polygons outline = *support_area.outline;
                                                                                                            for (const auto& foundation : intersecting_foundations)
                                                                                                            {
                                                                                                                outline = outline.difference(*foundation->outline);
                                                                                                            }
                                                                                                            if (outline.empty())
                                                                                                            {
                                                                                                                spdlog::get("support")->debug("Support touch-down for layer: {} while processing mesh: {}", layer_idx, support_area.mesh->mesh_name);
                                                                                                                support_area.area_type = SupportAreaType::NONE;
                                                                                                            }
                                                                                                        }
                                                                                                        return support_area; })
                                                                         | ranges::views::take_while([](const SupportArea& support_area){ return support_area.area_type != SupportAreaType::NONE; })
                                                                         | cura::views::to_shared_ptr
                                                                         | ranges::views::sliding(2)
                                                                         | ranges::views::transform([](auto layers_window) -> support_area_graph_t::value_type { return { ranges::front(layers_window), ranges::back(layers_window) }; })
                                                                         | ranges::views::reverse;
                                                       return layers_below; });
    // TODO: @jellespijker connect overhangs to first SupportArea below it
    // TODO: @jellespijker connect foundations to first SupportArea above it
    return ranges::make_view_closure(support_forest | ranges::views::join);
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

constexpr auto meshes_overhangs = ranges::views::transform(&ComputeOverhang) | ranges::views::join | cura::views::to_shared_ptr;
constexpr auto meshes_foundations = ranges::views::transform(&ComputeFoundation) | ranges::views::join | cura::views::to_shared_ptr;

} // namespace views

} // namespace cura::support

#endif // CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H
