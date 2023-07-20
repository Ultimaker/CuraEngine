// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_BROADCAST_H
#define PLUGINS_BROADCAST_H

#include "plugins/converters.h"
#include "utils/types/char_range_literal.h"
#include "utils/types/generic.h"

namespace cura::plugins::details
{

template<utils::CharRangeLiteral BroadcastChannel>
requires utils::is_broadcast_channel_v<BroadcastChannel, "BroadcastSettings">
constexpr auto broadcast_message_factory(auto&&... args)
{
    return broadcast_settings_request{}(std::forward<decltype(args)>(args)...);
};


template<class Stub, utils::CharRangeLiteral BroadcastChannel>
requires utils::is_broadcast_channel_v<BroadcastChannel, "BroadcastSettings">
constexpr auto broadcast_factory()
{
    return agrpc::RPC<&Stub::PrepareAsyncBroadcastSettings>{};
}

} // namespace cura::plugins::details

#endif // PLUGINS_BROADCAST_H