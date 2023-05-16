// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_TYPES_H
#define CURAENGINE_INCLUDE_PLUGINS_TYPES_H

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

#endif // CURAENGINE_INCLUDE_PLUGINS_TYPES_H
