// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "cura/plugins/v0/slot_id.pb.h"

namespace cura::plugins::details
{
template<plugins::v0::SlotID T1, plugins::v0::SlotID T2>
struct is_broadcast_channel
{
    inline static constexpr bool value{ T1 == T2 };
};

template<plugins::v0::SlotID T1, plugins::v0::SlotID T2>
inline constexpr bool is_broadcast_channel_v = is_broadcast_channel<T1, T2>::value;
} // namespace cura::plugins::details
