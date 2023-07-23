// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_BROADCAST_H
#define PLUGINS_BROADCAST_H

#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/converters.h"

#include <agrpc/asio_grpc.hpp>

#include <type_traits>

namespace cura::plugins::details
{

template<plugins::v0::SlotID T1, plugins::v0::SlotID T2>
struct is_broadcast_channel
{
    inline static constexpr bool value{ T1 == T2 };
};

template<plugins::v0::SlotID T1, plugins::v0::SlotID T2>
inline constexpr bool is_broadcast_channel_v = is_broadcast_channel<T1, T2>::value;

template<v0::SlotID S>
requires is_broadcast_channel_v<S, v0::SlotID::SETTINGS_BROADCAST>
constexpr auto broadcast_message_factory(auto&&... args)
{
    return broadcast_settings_request{}(std::forward<decltype(args)>(args)...);
};


template<class Stub, v0::SlotID S>
requires is_broadcast_channel_v<S, v0::SlotID::SETTINGS_BROADCAST>
constexpr auto broadcast_factory()
{
    return agrpc::RPC<&Stub::PrepareAsyncBroadcastSettings>{};
}

} // namespace cura::plugins::details

#endif // PLUGINS_BROADCAST_H