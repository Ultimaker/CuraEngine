// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H
#define CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <range/v3/action/sort.hpp>
#include <range/v3/algorithm/for_each.hpp>
#include <range/v3/back.hpp>
#include <range/v3/empty.hpp>
#include <range/v3/front.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/chunk_by.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/for_each.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/partial_sum.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/tail.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/take_while.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/unique.hpp>
#include <spdlog/spdlog.h>

#include "sliceDataStorage.h"
#include "support/support_area.h"
#include "utils/Simplify.h"
#include "utils/polygon.h"
#include "utils/views/dfs.h"
#include "utils/views/get.h"
#include "utils/views/to_shared_ptr.h"

namespace cura::support
{
using shared_mesh_t = std::shared_ptr<SliceMeshStorage>;
using shared_support_area_t = std::shared_ptr<SupportArea>;
using support_area_graph_t = std::unordered_multimap<shared_support_area_t, shared_support_area_t>;

coord_t ModelSlopeDistance(const Settings& settings, const std::string& setting_angle_key)
{
    const auto layer_height = settings.get<coord_t>("layer_height");
    const auto support_angle = settings.get<AngleRadians>(setting_angle_key);
    const auto tan_angle = std::tan(support_angle) - 0.01; // The X/Y component of the support angle. 0.01 to make 90 degrees work too.
    return static_cast<coord_t>(tan_angle) * layer_height; // Maximum horizontal distance that can be bridged.
}

/*!
 * Finds the delta layers from a specific layer_idx counting down/up until the requested distance is reached
 * @param mesh
 * @param layer_idx
 * @param setting_distance_key
 * @param search_downwards
 * @return
 */
auto LayerIndexDiff(shared_mesh_t mesh, size_t layer_idx, const std::string& setting_distance_key, const bool search_downwards)
{
    const auto delta_distance = mesh->settings.get<coord_t>(setting_distance_key);
    ranges::any_view<SliceLayer> layer_diff;
    if (search_downwards)
    {
        layer_diff = mesh->layers | ranges::views::take(layer_idx) | ranges::views::reverse;
    }
    else
    {
        layer_diff = mesh->layers | ranges::views::take(layer_idx);
    }

    return ranges::distance(layer_diff | cura::views::get(&SliceLayer::thickness) | ranges::views::partial_sum(std::plus{}) | ranges::views::take_while([delta_distance](const auto distance) { return distance <= delta_distance; }));
}

namespace views
{
namespace details
{
SupportArea MakeModelSupportAreaDiff(const auto& layers_window, const coord_t offset, shared_mesh_t mesh, support::SupportAreaType area_type, const Simplify& simplify)
{
    Polygons const& model_outline = std::get<1>(ranges::front(layers_window)).Outlines();
    Polygons const& other_model_outline = std::get<1>(ranges::back(layers_window)).Outlines();
    const Polygons model_diff_outline = simplify.polygon(model_outline.difference(other_model_outline.offset(offset)));
    const auto layer_idx = std::get<0>(ranges::front(layers_window));
    return { .mesh = mesh,
             .layer_idx = layer_idx,
             .outline = std::make_shared<Polygons>(model_diff_outline),
             .area_type = area_type,
             .area = std::make_shared<double>(model_diff_outline.area()),
             .bounding_box = std::make_shared<AABB>(model_diff_outline) };
}

auto ComputeOverhang(shared_mesh_t mesh)
{
    spdlog::get("support")->info("Compute support overhangs for {}", mesh->mesh_name);
    const auto max_dist_from_lower_layer = ModelSlopeDistance(mesh->settings, "support_angle");
    const auto minimum_support_area = mesh->settings.get<double>("minimum_support_area");
    const Simplify simplify(mesh->settings);
    auto mesh_layer_outline_window =
        mesh->layers | ranges::views::enumerate | ranges::views::reverse | ranges::views::sliding(2)
        | ranges::views::transform([mesh, max_dist_from_lower_layer, simplify](const auto& layers_window) { return MakeModelSupportAreaDiff(layers_window, max_dist_from_lower_layer, mesh, support::SupportAreaType::OVERHANG, simplify); })
        | ranges::views::reverse | ranges::views::filter([minimum_support_area](const SupportArea& support_area) { return *support_area.area >= minimum_support_area; });
    return ranges::make_view_closure(mesh_layer_outline_window);
}

auto ComputeFoundation(shared_mesh_t mesh)
{
    spdlog::get("support")->info("Compute support foundations for {}", mesh->mesh_name);
    const auto min_dist_from_upper_layer = ModelSlopeDistance(mesh->settings, "support_bottom_stair_step_min_slope");
    const Simplify simplify(mesh->settings);
    auto mesh_layer_outline_window =
        mesh->layers | ranges::views::enumerate | ranges::views::sliding(2)
        | ranges::views::transform([min_dist_from_upper_layer, mesh, simplify](const auto& layers_window) { return MakeModelSupportAreaDiff(layers_window, min_dist_from_upper_layer, mesh, support::SupportAreaType::FOUNDATION, simplify); });
    return ranges::make_view_closure(mesh_layer_outline_window);
}
} // namespace details

constexpr auto supportable_meshes = ranges::views::filter(
    [](shared_mesh_t mesh) { return mesh->settings.get<bool>("support_enable") && ! mesh->settings.get<bool>("anti_overhang_mesh") && ! mesh->settings.get<bool>("infill_mesh") && ! mesh->settings.get<bool>("support_mesh"); });

constexpr auto foundationable_meshes = ranges::views::filter([](shared_mesh_t mesh) { return mesh->settings.get<bool>("support_enable") && ! mesh->settings.get<bool>("anti_overhang_mesh") && ! mesh->settings.get<bool>("infill_mesh"); });

constexpr auto meshes_overhangs = ranges::views::transform(&details::ComputeOverhang) | ranges::views::join | cura::views::to_shared_ptr;
constexpr auto meshes_foundations = ranges::views::transform(&details::ComputeFoundation) | ranges::views::join | cura::views::to_shared_ptr;

} // namespace views

namespace actions
{

constexpr auto drop_down(auto&& overhangs, auto&& foundations)
{
    spdlog::get("support")->info("Dropdown support from overhangs to foundations");

    // Create the support forest, which is a view of `support_area_graph_t::value_type` (linking area's top-down) of naively dropped down overhangs, designated as `SUPPORT`, going down to either a foundation or the build plate.
    // Note at this time the offsets and subtractions or intersection with the model isn't applied yet.
    // TODO: @jellespijker take support_infill_sparse_thickness into account ( this should probably be done at the end )
    auto support_forest = overhangs
                        | ranges::views::transform(
                              [](shared_support_area_t overhang)
                              {
                                  const auto top_layer_idx = overhang->layer_idx - LayerIndexDiff(overhang->mesh, overhang->layer_idx, "support_top_distance", true);
                                  auto overhang_support_interface = ranges::views::iota(top_layer_idx, top_layer_idx + 1)
                                                                  | ranges::views::transform(
                                                                        [overhang](auto layer_idx) -> SupportArea
                                                                        {
                                                                            const auto support_offset = overhang->mesh->settings.get<coord_t>("support_offset");
                                                                            auto outline = std::make_shared<Polygons>(overhang->outline->offset(support_offset));
                                                                            return { .mesh = overhang->mesh,
                                                                                     .layer_idx = layer_idx,
                                                                                     .outline = outline,
                                                                                     .area_type = SupportAreaType::OVERHANG_INTERFACE,
                                                                                     .area = std::make_shared<double>(outline->area()),
                                                                                     .bounding_box = std::make_shared<AABB>(*outline) };
                                                                        })
                                                                  | cura::views::to_shared_ptr;

                                  auto layers_below_support_interface =
                                      ranges::views::iota(0UL, top_layer_idx - 1)
                                      | ranges::views::transform(
                                          [overhang](auto layer_idx) -> SupportArea
                                          { return { .mesh = overhang->mesh, .layer_idx = layer_idx, .outline = overhang->outline, .area_type = SupportAreaType::SUPPORT, .area = overhang->area, .bounding_box = overhang->bounding_box }; })
                                      | cura::views::to_shared_ptr;

                                  auto support_column = ranges::views::concat(layers_below_support_interface, overhang_support_interface) | ranges::views::reverse | ranges::views::sliding(2)
                                                      | ranges::views::transform(
                                                            [](auto layers_window) -> support_area_graph_t::value_type
                                                            {
                                                                ranges::back(layers_window)->outline = ranges::front(layers_window)->outline;
                                                                ranges::back(layers_window)->area = ranges::front(layers_window)->area;
                                                                ranges::back(layers_window)->bounding_box = ranges::front(layers_window)->bounding_box;
                                                                return { ranges::front(layers_window), ranges::back(layers_window) };
                                                            });

                                  return support_column;
                              });
    // NOTE: at this stage the outlines of the support area are still exactly the same as the overhang / all supports are also dropped until buildplate
    return support_forest | ranges::views::join | ranges::to<support::support_area_graph_t>;
}

auto join_areas(support_area_graph_t& support_graph)
{
    spdlog::get("support")->info("Join support areas");

    // Get all overhang_interfaces offset them with half the support_join_distance (leave the actual Polygon pointer untouched to avoid loosing data from this destructive open-close)
    using cache_t = std::tuple<shared_support_area_t, AABB, Polygons, coord_t>;
    auto overhang_interfaces = support_graph | ranges::views::keys | ranges::views::filter([](const shared_support_area_t& support_area) { return support_area->area_type == SupportAreaType::OVERHANG_INTERFACE; })
                             | ranges::views::transform(
                                   [](const shared_support_area_t& support_area) -> cache_t
                                   {
                                       const auto support_join_distance = support_area->mesh->settings.get<coord_t>("support_join_distance");
                                       const Polygons joined_outline{ support_area->outline->offset(support_join_distance) };
                                       return { support_area, AABB{ joined_outline }, joined_outline, support_join_distance };
                                   })
                             | ranges::to_vector;
    overhang_interfaces = ranges::actions::sort(overhang_interfaces, {}, [](const cache_t& support_area) { return std::get<0>(support_area)->layer_idx; });

    // Get a range of ranges for all the intersecting overhang_interfaces sorted on layer_height, smallest first
    auto connected_supports = overhang_interfaces
                            | ranges::views::transform(
                                  [overhang_interfaces](const cache_t& support_area)
                                  {
                                      return ranges::views::concat(ranges::views::single(support_area),
                                                                   overhang_interfaces | ranges::views::filter([support_area](const cache_t& other) { return std::get<0>(other) != std::get<0>(support_area); })
                                                                       | ranges::views::filter([support_area](const cache_t& other)
                                                                                               { return std::get<1>(support_area).hit(std::get<1>(other)) && ! std::get<2>(support_area).intersection(std::get<2>(other)).empty(); }));
                                  });

    for (auto connected_areas : connected_supports)
    {
        cache_t current_area = ranges::front(connected_areas);
        shared_support_area_t current_overhang_interface = std::get<0>(current_area);
        spdlog::get("support")->debug("Joining areas");
        for (const cache_t& other : connected_areas | ranges::views::drop(1))
        {
            // traverse over the other support_chain until the layer is reached and merge them
            // TODO: @jellespijker Double check how this behaves with making it a support interface
            const std::function<std::nullptr_t(shared_support_area_t, std::nullptr_t)> handle_node = [&current_overhang_interface, current_area, other](auto support_area, auto _)
            {
                if (support_area->layer_idx == current_overhang_interface->layer_idx)
                {
                    spdlog::get("support")->debug("Joining areas at layer: {} for mesh {} with {}", current_overhang_interface->layer_idx, current_overhang_interface->mesh->mesh_name, support_area->mesh->mesh_name);
                    support_area->area_type = SupportAreaType::MERGED;
                    auto avg_join_distance = (std::get<3>(current_area) + std::get<3>(other)) / 2;
                    current_overhang_interface->outline = std::make_shared<Polygons>(std::get<2>(current_area).unionPolygons(std::get<2>(other)).offset(-avg_join_distance)); // Merge the cached union offsets: "Cache is king!"
                    current_overhang_interface->bounding_box = std::make_shared<AABB>(*current_overhang_interface->outline);
                    current_overhang_interface->area = std::make_shared<double>(current_overhang_interface->outline->area());
                }
                return nullptr;
            };
            std::unordered_set<shared_support_area_t> visited{};
            cura::actions::dfs(current_overhang_interface, support_graph, handle_node, visited);
        }
        // The first value in the subrange is the one that remains
        // merge the outline by adding the intersection area(s) and the other normal outline
        // update aabb and area
        // Do this for the first layers below the lowest layer_idx
        // TODO: @jellespijker keep book keeping up to date for chained intersections, keep in mind height changes
    }

    // Loop again over all foundations trickling all changes downwards

    // TODO: @jellespijker update map accordingly

    return support_graph;
}

} // namespace actions

} // namespace cura::support

#endif // CURAENGINE_INCLUDE_SUPPORT_SUPPORT_H
