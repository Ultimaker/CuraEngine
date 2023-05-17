// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_CONVERTERS_H
#define PLUGINS_CONVERTERS_H

#include <memory>

#include "plugin.grpc.pb.h"
#include "plugins/types.h"

namespace cura::plugins
{

struct plugin_request
{
    using value_type = proto::PluginRequest;
    using native_value_type = cura::plugins::SlotID;

    value_type operator()(const native_value_type & slot_id) const
    {
        value_type message{};
        message.set_id(slot_id);
        return message;
    }
};

struct plugin_response
{
    using value_type = proto::PluginResponse;
    using native_value_type = std::pair<std::string, std::string>;

    native_value_type operator()(const value_type& message) const
    {
        return std::make_pair( message.version(), message.plugin_hash() );
    }
};

struct simplify_request
{
    using value_type = proto::SimplifyRequest;
    using native_value_type = Polygons;

    value_type operator()(const native_value_type& polygons, const size_t max_deviation, const size_t max_angle) const
    {
        value_type message{};
        message.set_max_deviation(max_deviation);
        message.set_max_angle(max_angle);
        for (const auto& polygon : polygons.paths)
        {
            auto poly = message.polygons();
            for (const auto& path : polygons.paths)
            {
                auto* p = poly.add_paths();
                for (const auto& point : path)
                {
                    auto* pt = p->add_path();
                    pt->set_x(point.X);
                    pt->set_y(point.Y);
                }
            }
        }
        return message;
    }
};

struct simplify_response
{
    using value_type = proto::SimplifyResponse;
    using native_value_type = Polygons;

    native_value_type operator()(const value_type& message) const
    {
        native_value_type poly{};
        for (const auto& paths : message.polygons().paths())
        {
            Polygon p{};
            for (const auto& point : paths.path())
            {
                p.add(Point{ point.x(), point.y() });
            }
            poly.add(p);
        }
        return poly;
    }
};

struct postprocess_request
{
    using value_type = proto::PostprocessRequest;
    using native_value_type = std::string;

    value_type operator()(const native_value_type& gcode) const
    {
        value_type message{};
        message.set_gcode_word(gcode);
        return message;
    }
};

struct postprocess_response
{
    using value_type = proto::PostprocessResponse;
    using native_value_type = std::string;

    native_value_type operator()(const value_type& message) const
    {
        return message.gcode_word();
    }
};

} // namespace cura::plugins


#endif
