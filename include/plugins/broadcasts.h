// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_BROADCAST_H
#define PLUGINS_BROADCAST_H

#include "cura/plugins/slots/broadcast/v0/broadcast.grpc.pb.h"
#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/converters.h"

#include <agrpc/asio_grpc.hpp>
#include <range/v3/utility/semiregular_box.hpp>

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

template<class T, class C = empty>
struct broadcast_stub
{
    using derived_type = T;
    friend derived_type;

    constexpr auto operator()(auto&&... args)
    {
        return request_(std::forward<decltype(args)>(args)...);
    }

private:
    C request_{};
};

template<v0::SlotID S, class Stub>
requires is_broadcast_channel_v<S, v0::SlotID::SETTINGS_BROADCAST>
struct broadcast_rpc : public broadcast_stub<broadcast_rpc<S, Stub>, broadcast_settings_request>
{
    using ClientRPC = agrpc::ClientRPC<&Stub::PrepareAsyncBroadcastSettings>;
};


} // namespace cura::plugins::details

#endif // PLUGINS_BROADCAST_H