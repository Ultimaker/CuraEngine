// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <sstream>
#include <thread>

#include <fmt/core.h>

namespace fmt
{
template<>
struct formatter<std::thread::id> : formatter<string_view>
{
    template<typename FormatContext>
    auto format(std::thread::id thread_id, FormatContext& ctx) const
    {
        std::ostringstream oss;
        oss << thread_id;
        return formatter<string_view>::format(oss.str(), ctx);
    }
};

} // namespace fmt
