// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <utility>

namespace curaengine::plugins::details
{
struct default_process
{
    constexpr auto operator()(auto&& arg, auto&&...)
    {
        return std::forward<decltype(arg)>(arg);
    };
};
} // namespace curaengine::plugins::details
#endif // ENABLE_PLUGINS
