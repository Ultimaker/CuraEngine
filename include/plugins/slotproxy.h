// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTPROXY_H
#define PLUGINS_SLOTPROXY_H

#include <functional>
#include <memory>
#include <optional>

#include <grpcpp/channel.h>

#include "plugins/converters.h"
#include "plugins/pluginproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"

namespace cura::plugins
{

template<plugins::SlotID Slot, class Validator, class Stub, class Prepare, class Request, class Response, auto Default>
class SlotProxy
{
    std::optional<PluginProxy<Slot, Validator, Stub, Prepare, Request, Response>> plugin_{ std::nullopt };

public:
    static inline constexpr plugins::SlotID slot_id{ Slot };

    constexpr SlotProxy() noexcept = default;
    SlotProxy(std::shared_ptr<grpc::Channel> channel) : plugin_{ std::move(channel) } {};

    auto operator()(auto&&... args)
    {
        if (plugin_.has_value())
        {
            return std::invoke(plugin_.value(), std::forward<decltype(args)>(args)...);
        }
        return std::invoke(Default, std::forward<decltype(args)>(args)...);
    }
};

} // namespace cura::plugins


#endif // PLUGINS_SLOTPROXY_H
