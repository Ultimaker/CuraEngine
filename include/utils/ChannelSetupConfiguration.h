// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS

#include <cstdint>
#include <string>

namespace cura::utils
{

struct ChannelSetupConfiguration
{
    std::string host;
    uint64_t port;
    uint64_t shibolet = 0UL; // TODO: Either get from startup (server would receive this as well) or remove completely.
};

} // namespace cura::utils

#endif // ENABLE_PLUGINS
