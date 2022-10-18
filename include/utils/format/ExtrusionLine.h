// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_EXTRUSIONLINE_H
#define CURAENGINE_EXTRUSIONLINE_H

#include "utils/ExtrusionLine.h"

#include <fmt/format.h>
#include <fmt/ranges.h>


template<>
struct [[maybe_unused]] fmt::formatter<cura::ExtrusionLine>
{
    constexpr auto parse(fmt::format_parse_context& ctx) -> decltype(ctx.begin())
    {
        if (ctx.begin() != ctx.end() && *ctx.begin() != '}')
        {
            throw fmt::format_error("invalid format");
        }
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const cura::ExtrusionLine& extrusion_line, FormatContext& ctx) const -> decltype(ctx.out())
    {
        return fmt::format_to(ctx.out(), "ExtrusionLine: <idx: {}, odd: {}, closed: {}>", extrusion_line.inset_idx, extrusion_line.is_odd, extrusion_line.is_closed);
    }
};


#endif // CURAENGINE_EXTRUSIONLINE_H
