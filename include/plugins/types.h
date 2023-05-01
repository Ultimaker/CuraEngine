// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_TYPES_H
#define CURAENGINE_INCLUDE_PLUGINS_TYPES_H

#include <Arcus/Types.h>
#include <memory>
#include <tuple>

#include "utils/IntPoint.h"
#include "utils/polygon.h"

#include "plugin.pb.h"

namespace cura::plugins
{
using SlotID = proto::SlotID;

namespace details
{
template<size_t N>
struct CharRangeLiteral
{
    constexpr CharRangeLiteral(const char (&str)[N])
    {
        std::copy_n(str, N, value);
    }

    char value[N];
};

} // namespace details

namespace converters
{
template<class T>
class converter_base
{
    friend T;
public:
    auto operator()(auto& arg, auto&&... args)
    {
        if constexpr (std::is_same_v<decltype(arg), cura::plugins::proto::SlotID&>)
        {
            proto::Plugin_args msg{};
            msg.set_id(arg);
            return std::make_shared<proto::Plugin_args>(msg);
        }
        else if constexpr (std::is_same_v<decltype(arg), proto::Plugin_ret>)
        {
            return std::tuple<std::string, std::string>{ arg.version(), arg.plugin_hash() };
        }
        return 1;
        //return static_cast<T&>(*this).make(arg, std::forward<decltype(args)>(args)...);
    }
};

template<class Send, class Receive>
class simplify_converter_fn : public converter_base<simplify_converter_fn<Send, Receive>>
{
public:
    using receive_t = Receive;
    using send_t = Send;

private:
    Polygons make(const Receive& args)
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

    std::shared_ptr<Send> make(const Polygons& polygons, const size_t max_deviation, const size_t max_angle)
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
class postprocess_converter_fn : public converter_base<postprocess_converter_fn<Send, Receive>>
{
public:
    using receive_t = Receive;
    using send_t = Send;

    std::string make(const Receive& args)
    {
        return args.gcode();
    }

    std::shared_ptr<Send> make(const std::string& gcode)
    {
        Send args{};
        args.set_gcode(gcode);
        return std::make_shared<Send>(args);
    }
};

} // namespace converters


} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_TYPES_H
