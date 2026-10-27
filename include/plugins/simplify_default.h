// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <utility>

#include "utils/Simplify.h" // TODO: Remove once the simplify slot has been removed

namespace cura::plugins::details
{
struct simplify_default
{
    auto operator()(auto&& arg, auto&&... args)
    {
        const Simplify simplify{ std::forward<decltype(args)>(args)... };
        return simplify.polygon(std::forward<decltype(arg)>(arg));
    }
};
} // namespace cura::plugins::details
#endif // ENABLE_PLUGINS
