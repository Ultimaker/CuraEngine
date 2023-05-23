// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_METADATA_H
#define CURAENGINE_INCLUDE_PLUGINS_METADATA_H

#include <string>

namespace cura::plugins
{
struct plugin_metadata
{
    std::string name;
    std::string version;
    std::string peer;

    std::string slot_version;
};

struct slot_metadata
{
    std::string_view rpc_id;
    std::string_view version_range;
};

} // namespace cura::plugins

#endif // CURAENGINE_INCLUDE_PLUGINS_METADATA_H
