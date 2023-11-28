// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/channel.h"

#include "utils/resources/certificate.pem.h"

#include <fmt/format.h>
#include <spdlog/spdlog.h>

namespace cura::utils
{
namespace details
{
inline constexpr static bool ALLOW_REMOTE_CHANNELS = ENABLE_REMOTE_PLUGINS;
} // namespace details

std::shared_ptr<grpc::Channel> createChannel(const ChannelSetupConfiguration& config)
{
    constexpr auto create_credentials = [](const ChannelSetupConfiguration& config)
    {
        if (config.host == "localhost" || config.host == "127.0.0.1")
        {
            spdlog::info("Create local channel on port {}.", config.port);
            return grpc::InsecureChannelCredentials();
        }
        if (details::ALLOW_REMOTE_CHANNELS)
        {
            spdlog::info("Create local channel on port {}.", config.port);
            auto creds_config = grpc::SslCredentialsOptions();
            creds_config.pem_root_certs = resources::certificate;
            return grpc::SslCredentials(creds_config);
        }
        // Create empty credentials, so it'll make a dummy channel where all operations fail.
        // This is consistent with creating a channel with the wrong credentials as it where.
        spdlog::warn("Remote plugins where disabled, will not connect to {}:{}.", config.host, config.port);
        return std::shared_ptr<grpc::ChannelCredentials>();
    };
    grpc::ChannelArguments args;
    args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, 200 * 1000 /*200 sec*/);
    args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 100 * 1000 /*100 sec*/);
    args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);

    return grpc::CreateCustomChannel(fmt::format("{}:{}", config.host, config.port), create_credentials(config), args);
}

} // namespace cura::utils
