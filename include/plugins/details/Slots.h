// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifndef ENABLE_PLUGINS
#include <utility>

#include "plugins/v0/SlotID.h"

namespace curaengine::plugins::details
{
struct Slots
{
    template<v0::SlotID S>
    constexpr auto modify(auto&& data, auto&&... args) noexcept
    {
        return std::forward<decltype(data)>(data);
    }

    template<v0::SlotID S>
    constexpr auto broadcast(auto&&... args) noexcept
    {
    }

    constexpr auto connect(auto&&... args) noexcept
    {
    }
};
} // namespace curaengine::plugins::details
#endif // ENABLE_PLUGINS
