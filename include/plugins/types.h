// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_TYPES_H
#define PLUGINS_TYPES_H

#include <memory>
#include <tuple>

#include <fmt/format.h>
#include <grpcpp/support/string_ref.h>

#include "utils/IntPoint.h"
#include "utils/types/generic.h"
#include "utils/polygon.h"

#include "cura/plugins/v0/slot_id.pb.h"

namespace cura::plugins
{
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


namespace fmt
{
// Custom formatter for humanreadable slot_id's
template<>
struct formatter<cura::plugins::v0::SlotID>
{
    template<typename FormatContext>
    auto format(cura::plugins::v0::SlotID slot_id, FormatContext& ctx)
    {
        std::string slot_name;

        switch (slot_id)
        {
        case cura::plugins::v0::SlotID::SIMPLIFY:
            slot_name = "SimplifyService";
            break;
        case cura::plugins::v0::SlotID::POSTPROCESS:
            slot_name = "PostprocessService";
            break;
        default:
            slot_name = "Unknown";
            break;
        }

        return fmt::format_to(ctx.out(), "{}", slot_name);
    }

    template<typename ParseContext>
    auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }
};

template<>
struct formatter<grpc::string_ref>
{
    constexpr auto parse(format_parse_context& ctx)
    {
        return ctx.end();
    }

    template<typename FormatContext>
    auto format(const grpc::string_ref& str, FormatContext& ctx)
    {
        return format_to(ctx.out(), "{}", std::string_view{ str.data(), str.size() });
    }
};

} // namespace fmt
#endif // PLUGINS_TYPES_H
