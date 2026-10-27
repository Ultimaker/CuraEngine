// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <cstddef>
#include <google/protobuf/empty.pb.h>

namespace cura::plugins
{
struct empty
{
    using value_type = google::protobuf::Empty; ///< The protobuf message type.
    using native_value_type = std::nullptr_t; ///< The native value type.

    value_type operator()() const;

    constexpr native_value_type operator()(const value_type&) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
