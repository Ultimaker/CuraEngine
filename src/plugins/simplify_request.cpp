// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/simplify_request.h"

#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>

using namespace curaengine::utils;

namespace curaengine::plugins
{
simplify_request::value_type simplify_request::operator()(
    const simplify_request::native_value_type& polygons,
    const coord_t max_resolution,
    [[maybe_unused]] const coord_t max_deviation,
    [[maybe_unused]] const coord_t max_area_deviation) const
{
    value_type message{};
    if (polygons.empty())
    {
        return message;
    }

    auto* msg_polygons = message.mutable_polygons();
    auto* msg_polygon = msg_polygons->add_polygons();
    auto* msg_outline = msg_polygon->mutable_outline();

    for (const auto& point : ranges::front(polygons))
    {
        auto* msg_outline_path = msg_outline->add_path();
        msg_outline_path->set_x(point.X);
        msg_outline_path->set_y(point.Y);
    }

    auto* msg_holes = msg_polygon->mutable_holes();
    for (const auto& polygon : polygons | ranges::views::drop(1))
    {
        auto* msg_hole = msg_holes->Add();
        for (const auto& point : polygon)
        {
            auto* msg_path = msg_hole->add_path();
            msg_path->set_x(point.X);
            msg_path->set_y(point.Y);
        }
    }

    message.set_max_resolution(max_resolution);
    message.set_max_deviation(max_resolution);
    message.set_max_area_deviation(max_resolution);
    return message;
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
