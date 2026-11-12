// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <grpcpp/create_channel.h>
#include <memory>
#include <string>

#include "types/ChannelSetupConfiguration.h"

#ifndef ENABLE_REMOTE_PLUGINS
#define ENABLE_REMOTE_PLUGINS false
#endif

namespace curaengine::utils
{
std::shared_ptr<grpc::Channel> createChannel(const types::ChannelSetupConfiguration& config = { "localhost", 50010UL });

} // namespace curaengine::utils

#endif // ENABLE_PLUGINS
