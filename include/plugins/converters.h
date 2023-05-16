// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_CONVERTERS_H
#define PLUGINS_CONVERTERS_H

#include <memory>

#include "plugin.grpc.pb.h"
#include "plugins/types.h"

namespace cura::plugins
{

namespace details
{
template<class Request>
struct plugin_request
{
    using value_type = Request;

    auto operator()(const cura::plugins::proto::SlotID& slot_id) const
    {
        proto::PluginRequest message{};
        message.set_id(slot_id);
        return std::make_shared<value_type>(message);
    }
};

template<class Response>
struct plugin_response
{
    using value_type = Response;

    auto operator()(const value_type& message) const
    {
        return std::make_pair( message.version(), message.plugin_hash() );
    }
};

template<class Request>
struct simplify_request
{
    using value_type = Request;

    auto operator()(const Polygons& polygons, const size_t max_deviation, const size_t max_angle) const
    {
        proto::SimplifyRequest message{};
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
        return std::make_shared<value_type>(message);
    }
};

template<class Response>
struct simplify_response
{
    using value_type = Response;

    auto operator()(const value_type& message) const
    {
        Polygons poly{};
        for (const auto& paths : message.polygons().paths())
        {
            Polygon p{};
            for (const auto& point : paths.path())
            {
                p.add(Point{ point.y(), point.y() });
            }
            poly.add(p);
        }
        return poly;
    }
};

template<class Request>
struct postprocess_request
{
    using value_type = Request;

    auto operator()(const std::string& gcode) const
    {
        proto::PostprocessRequest message{};
        message.set_gcode_word(gcode);
        return std::make_shared<value_type>(message);
    }
};

template<class Response>
struct postprocess_response
{
    using value_type = Response;

    auto operator()(const value_type& message) const
    {
        return message.gcode_word();
    }
};

} // namespace details


using plugin_request_t = details::plugin_request<proto::PluginRequest>;
using plugin_response_t = details::plugin_response<proto::PluginResponse>;
using simplify_request_t = details::simplify_request<proto::SimplifyRequest>;
using simplify_response_t = details::simplify_request<proto::SimplifyResponse>;
using postprocess_request_t = details::postprocess_request<proto::PostprocessRequest>;
using postprocess_response_t = details::postprocess_response<proto::PostprocessResponse>;

} // namespace cura::plugins


#endif
