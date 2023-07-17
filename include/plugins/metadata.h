// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_METADATA_H
#define CURAENGINE_INCLUDE_PLUGINS_METADATA_H

#include <grpcpp/client_context.h>
#include <map>
#include <string>

#include <grpcpp/support/string_ref.h>
#include <string_view>

#include "plugins/types.h"

namespace cura::plugins
{

struct plugin_metadata
{
    std::string_view slot_version;
    std::string_view plugin_name;
    std::string_view plugin_version;
    std::string_view peer;
    std::set<std::string_view> broadcast_subscriptions;
};

struct slot_metadata
{
    plugins::v0::SlotID slot_id;
    std::string_view version_range;
    std::string_view engine_uuid;
};

} // namespace cura::plugins

#endif // CURAENGINE_INCLUDE_PLUGINS_METADATA_H
