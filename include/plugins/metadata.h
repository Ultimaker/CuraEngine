// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_METADATA_H
#define CURAENGINE_INCLUDE_PLUGINS_METADATA_H

#include <map>
#include <string>

#include <grpcpp/support/string_ref.h>
#include <string_view>

#include "plugins/types.h"

namespace cura::plugins
{
struct plugin_metadata
{
    std::string name;  // cura-plugin-name (optional)
    std::string version;  // cura-plugin-version (optional)
    std::string peer;
    std::string slot_version;  // cura-slot-version (required)

    explicit plugin_metadata(const grpc::ClientContext& client_context)
    {
        const auto& metadata = client_context.GetServerInitialMetadata();
        if (auto it = metadata.find("cura-slot-version"); it != metadata.end())
        {
            slot_version = std::string{ it->second.data(), it->second.size() };
        }
        else
        {
            spdlog::error("'cura-slot-version' RPC metadata not set");
            throw std::runtime_error("'cura-slot-version' RPC metadata not set");
        }
        if (auto it = metadata.find("cura-plugin-name"); it != metadata.end())
        {
            name = std::string{ it->second.data(), it->second.size() };
        }
        else
        {
            spdlog::warn("'cura-plugin-name' RPC metadata not set");
        }
        if (auto it = metadata.find("cura-plugin-version"); it != metadata.end())
        {
            version = std::string{ it->second.data(), it->second.size() };
        }
        else
        {
            spdlog::warn("'cura-plugin-version' RPC metadata not set");
        }
        peer = client_context.peer();
    }
};

struct slot_metadata
{
    plugins::v0::SlotID slot_id;
    std::string_view version_range;
    std::string_view engine_uuid;
};

} // namespace cura::plugins

#endif // CURAENGINE_INCLUDE_PLUGINS_METADATA_H
