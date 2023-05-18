// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_TYPES_H
#define PLUGINS_TYPES_H

#include <Arcus/Types.h>
#include <memory>
#include <tuple>

#include "utils/IntPoint.h"
#include "utils/concepts/generic.h"
#include "utils/polygon.h"

#include "plugin.grpc.pb.h"

namespace cura::plugins
{
using SlotID = proto::SlotID;

namespace details
{
template<size_t N>
struct CharRangeLiteral
{
    constexpr CharRangeLiteral(const char (&str)[N])
    {
        std::copy_n(str, N, value);
    }

    char value[N];
};

} // namespace details

} // namespace cura::plugins


// Custom formatter for humanreadable slot_id's
template<>
struct fmt::formatter<cura::plugins::SlotID>
{
    // The formatting function
    template<typename FormatContext>
    auto format(cura::plugins::SlotID slot_id, FormatContext& ctx)
    {
        std::string slot_name;

        switch (slot_id)
        {
        case cura::plugins::SlotID::SIMPLIFY:
            slot_name = "Simplify";
            break;
        case cura::plugins::SlotID::POSTPROCESS:
            slot_name = "Postprocess";
            break;
        default:
            slot_name = "Unknown";
            break;
        }

        return fmt::format_to(ctx.out(), "{}", slot_name);
    }

    // The parsing function
    template<typename ParseContext>
    auto parse(ParseContext& ctx)
    {
        // Not implemented for simplicity in this example
        return ctx.begin();
    }
};

#endif // PLUGINS_TYPES_H
