// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_BROADCAST_H
#define PLUGINS_BROADCAST_H

#include "plugins/converters.h"
#include "plugins/types.h"

namespace cura::plugins::details
{

template<CharRangeLiteral T1, CharRangeLiteral T2>
class is_broadcast_channel
{
    inline static constexpr bool value_() noexcept
    {
        constexpr std::string_view t1{ T1.value };
        constexpr std::string_view t2{ T2.value };
        return t1 == t2;
    }

public:
    inline static constexpr bool value = value_();
};

template<CharRangeLiteral T1, CharRangeLiteral T2>
inline constexpr bool is_broadcast_channel_v = is_broadcast_channel<T1, T2>::value;

template<CharRangeLiteral BroadcastChannel>
requires is_broadcast_channel_v<BroadcastChannel, "BroadcastSettings"> constexpr auto broadcast_request_message_factory(auto&&... args)
{
    return broadcast_settings_request{}(std::forward<decltype(args)>(args)...);
};


template<class Stub, CharRangeLiteral BroadcastChannel>
requires is_broadcast_channel_v<BroadcastChannel, "BroadcastSettings"> constexpr auto broadcast_request_factory()
{
    return agrpc::RPC<&Stub::PrepareAsyncBroadcastSettings>{};
}

} // namespace cura::plugins::details

#endif // PLUGINS_BROADCAST_H
