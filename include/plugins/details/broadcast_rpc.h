// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <agrpc/asio_grpc.hpp>

#include "plugins/broadcast_settings_request.h"
#include "plugins/details/broadcast_stub.h"
#include "plugins/details/is_broadcast_channel.h"

namespace curaengine::plugins::details
{
template<v0::SlotID S, class Stub>
    requires is_broadcast_channel_v<S, v0::SlotID::SETTINGS_BROADCAST>
struct broadcast_rpc : public broadcast_stub<broadcast_rpc<S, Stub>, broadcast_settings_request>
{
    using ClientRPC = agrpc::ClientRPC<&Stub::PrepareAsyncBroadcastSettings>;
};
} // namespace curaengine::plugins::details
