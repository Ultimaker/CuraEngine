// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <utility>

namespace cura::plugins::details
{
template<class T, class Msg, class Native>
struct converter
{
    using derived_type = T; ///< The derived type.
    using value_type = Msg; ///< The protobuf message type.
    using native_value_type = Native; ///< The native value type.
    friend derived_type;

    constexpr auto operator()(auto&&... args) const
    {
        return static_cast<const derived_type*>(this)->operator()(std::forward<decltype(args)>(args)...);
    }
};
} // namespace cura::plugins::details
#endif // ENABLE_PLUGINS
