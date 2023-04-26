// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_TYPES_H
#define CURAENGINE_INCLUDE_PLUGINS_TYPES_H

#include <memory>
#include <tuple>

#include "utils/IntPoint.h"
#include "utils/polygon.h"

#include "plugin.pb.h"

namespace cura::plugins
{
namespace converters
{
template<class Send, class Receive>
struct converter_base
{
    using send_t = Send;
    using receive_t = Receive;

    auto operator()(const SlotID slot_id)
    {
        proto::Plugin_args args{};
        args.set_id(slot_id);
        return std::make_shared<plugins::proto::Plugin_args>(args);
    }

    std::tuple<std::string, std::string> operator()(const proto::Plugin_ret& args)
    {
        return { args.version(), args.plugin_hash() };
    }
};

template<class Send, class Receive>
struct simplify_converter_fn : public converter_base<Send, Receive>
{
    Polygons operator()(const Receive& args) const noexcept
    {
        Polygons poly{};
        for (const auto& paths : args.polygons().paths())
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

    auto operator()(const Polygons& polygons, const size_t max_deviation, const size_t max_angle) const noexcept
    {
        Send args{};
        args.set_max_deviation(max_deviation);
        args.set_max_angle(max_angle);
        for (const auto& polygon : polygons.paths)
        {
            auto poly = args.polygons();
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
        return std::make_shared<Send>(args);
    }
};

template<class Send, class Receive>
struct postprocess_converter_fn : public converter_base<Send, Receive>
{
    std::string operator()(const Receive& args) const noexcept
    {
        return args.gcode();
    }

    auto operator()(const std::string& gcode) const noexcept
    {
        Send args{};
        args.set_gcode(gcode);
        return std::make_shared<Send>(args);
    }
};

} // namespace converters


} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_TYPES_H
