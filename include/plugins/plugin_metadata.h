// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <set>
#include <string>

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
} // namespace cura::plugins
