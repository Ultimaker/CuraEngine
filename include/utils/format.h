// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_UTILS_FORMAT_H
#define CURAENGINE_UTILS_FORMAT_H

#include <fmt/format.h>
#include <fmt/ranges.h>

namespace ClipperLib
{
class IntPoint; // Forward declaration
}

template<>
struct [[maybe_unused]] fmt::formatter<ClipperLib::IntPoint>
{
    constexpr auto parse(fmt::format_parse_context& ctx) -> decltype(ctx.begin())
    {
        if (ctx.begin() != ctx.end() && *ctx.begin() != '}')
        {
            throw fmt::format_error("invalid format");
        }
        return ctx.begin();
    }

    /** fmt formatter for IntPoints
     * This allows spdlog and/or fmt to output a single point as [X, Y].
     * With the inclusion of the fmt/ranges.h a container of points will be: [[X_0, Y_0], [X_1, Y_1], ..., [X_n, Y_n]]
     * Usage as:
     * \code{.cpp}
     * std::vector<Point> points {{100, 200}, {300, 400}, {500, 600}};
     * spdlog:debug("points: {}", points};
     * auto fmt::format("{}", points};
     * \endcode
     **/
    template<typename FormatContext>
    auto format(const ClipperLib::IntPoint& point, FormatContext& ctx) const -> decltype(ctx.out())
    {
        return fmt::format_to(ctx.out(), "[{}, {}]", point.X, point.Y);
    }
};


#endif // CURAENGINE_UTILS_FORMAT_H
