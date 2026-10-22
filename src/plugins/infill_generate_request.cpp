// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/infill_generate_request.h"

#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>

namespace cura::plugins
{
infill_generate_request::value_type
    infill_generate_request::operator()(const infill_generate_request::native_value_type& inner_contour, const std::string& pattern, const Settings& settings, const coord_t z)
        const
{
    value_type message{};
    message.set_pattern(pattern);
    auto* msg_settings = message.mutable_settings()->mutable_settings();
    for (const auto& [key, value] : settings.getFlattendSettings())
    {
        msg_settings->insert({ key, value });
    }

    // ------------------------------------------------------------
    // Add current z height to settings message
    // ------------------------------------------------------------
    msg_settings->insert({ "z", std::to_string(z) });

    if (inner_contour.empty())
    {
        return message;
    }

    auto* msg_polygons = message.mutable_infill_areas();
    auto* msg_polygon = msg_polygons->add_polygons();
    auto* msg_outline = msg_polygon->mutable_outline();

    for (const auto& point : ranges::front(inner_contour))
    {
        auto* msg_outline_path = msg_outline->add_path();
        msg_outline_path->set_x(point.X);
        msg_outline_path->set_y(point.Y);
    }

    auto* msg_holes = msg_polygon->mutable_holes();
    for (const auto& polygon : inner_contour | ranges::views::drop(1))
    {
        auto* msg_hole = msg_holes->Add();
        for (const auto& point : polygon)
        {
            auto* msg_path = msg_hole->add_path();
            msg_path->set_x(point.X);
            msg_path->set_y(point.Y);
        }
    }

    return message;
}
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
