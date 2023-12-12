// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_METADATA_H
#define CURAENGINE_INCLUDE_PLUGINS_METADATA_H

#include <grpcpp/client_context.h>
#include <grpcpp/support/string_ref.h>
#include <map>
#include <string>
#include <string_view>

#include <fmt/format.h>

#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/types.h"

namespace cura::plugins
{

struct plugin_metadata
{
    std::string slot_version_range;
    std::string plugin_name;
    std::string plugin_version;
    std::string peer;
    std::set<int> broadcast_subscriptions;
};

struct slot_metadata
{
    plugins::v0::SlotID slot_id;
    std::string_view version;
    std::string_view engine_uuid;
};

} // namespace cura::plugins

namespace cura::plugins::v0
{

constexpr auto format_as(SlotID id)
{
    return fmt::underlying(id);
}

} // namespace cura::plugins::v0

#endif // CURAENGINE_INCLUDE_PLUGINS_METADATA_H
