// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_BROADCAST_H
#define PLUGINS_BROADCAST_H

#include "plugins/converters.h"
#include "utils/types/char_range_literal.h"
#include "utils/types/generic.h"

#include <agrpc/asio_grpc.hpp>

namespace cura::plugins::details
{

template<v0::SlotID T1, v0::SlotID T2>
class is_broadcast_channel
{
    inline static constexpr bool value_() noexcept
    {
        return T1 == T2;
    }

public:
    inline static constexpr bool value = value_();
};

template<v0::SlotID T1, v0::SlotID T2>
inline constexpr bool is_broadcast_channel_v = is_broadcast_channel<T1, T2>::value;


template<v0::SlotID BroadcastSlot>
requires is_broadcast_channel_v<BroadcastSlot, v0::SlotID::BROADCAST_SETTINGS> constexpr auto broadcast_message_factory(auto&&... args)
{
    return broadcast_settings_request{}(std::forward<decltype(args)>(args)...);
};


template<class Stub, v0::SlotID BroadcastSlot>
requires is_broadcast_channel_v<BroadcastSlot, v0::SlotID::BROADCAST_SETTINGS> constexpr auto broadcast_factory()
{
    return agrpc::RPC<&Stub::PrepareAsyncBroadcastSettings>{};
}

} // namespace cura::plugins::details

#endif // PLUGINS_BROADCAST_H