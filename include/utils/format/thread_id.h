// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_FORMAT_THREAD_ID_H
#define UTILS_FORMAT_THREAD_ID_H

#include <sstream>
#include <string_view>
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
#endif // UTILS_FORMAT_THREAD_ID_H
