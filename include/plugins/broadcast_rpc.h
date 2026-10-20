// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <agrpc/asio_grpc.hpp>
#include <range/v3/utility/semiregular_box.hpp>

#include "cura/plugins/slots/broadcast/v0/broadcast.grpc.pb.h"
#include "plugins/broadcast_settings_request.h"
#include "plugins/broadcast_stub.h"
#include "plugins/is_broadcast_channel.h"

namespace cura::plugins::details
{
template<v0::SlotID S, class Stub>
    requires is_broadcast_channel_v<S, v0::SlotID::SETTINGS_BROADCAST>
struct broadcast_rpc : public broadcast_stub<broadcast_rpc<S, Stub>, broadcast_settings_request>
{
    using ClientRPC = agrpc::ClientRPC<&Stub::PrepareAsyncBroadcastSettings>;
};
} // namespace cura::plugins::details
