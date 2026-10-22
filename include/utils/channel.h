// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "utils/ChannelSetupConfiguration.h"

#include <grpcpp/create_channel.h>
#include <memory>
#include <string>

#ifndef ENABLE_REMOTE_PLUGINS
#define ENABLE_REMOTE_PLUGINS false
#endif

namespace cura::utils
{
std::shared_ptr<grpc::Channel> createChannel(const ChannelSetupConfiguration& config = { "localhost", 50010UL });

} // namespace cura::utils

#endif // ENABLE_PLUGINS
