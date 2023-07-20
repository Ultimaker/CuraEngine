// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CHANNEL_H
#define CHANNEL_H

#include <grpcpp/create_channel.h>
#include <memory>
#include <string>

#ifndef ENABLE_REMOTE_PLUGINS
#define ENABLE_REMOTE_PLUGINS false
#endif

namespace cura::utils
{
struct ChannelSetupConfiguration
{
public:
    std::string host;
    uint64_t port;
    uint64_t shibolet = 0UL; // TODO: Either get from startup (server would receive this as well) or remove completely.
};

std::shared_ptr<grpc::Channel> createChannel(const ChannelSetupConfiguration& config = { "localhost", 50010UL });

} // namespace cura::utils

#endif // CHANNEL_H
