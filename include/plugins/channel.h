#ifndef CHANNEL_H
#define CHANNEL_H

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include "../resources/certificate.pem.h"
#include "plugins/slots.h"

#ifndef BUILD_ALLOW_REMOTE_PLUGINS
#define BUILD_ALLOW_REMOTE_PLUGINS true
#endif

namespace cura::plugins
{
    namespace details
    {
        constexpr bool ALLOW_REMOTE_PLUGINS = (BUILD_ALLOW_REMOTE_PLUGINS);
    } // namespace details

    struct ChannelSetupConfiguration
    {
    public:
        std::string host;
        uint64_t port;
        uint64_t shibolet = 0UL; //TODO: Either get from startup (plugin would receive this as well) or remove completely.
    };

    auto createChannel(const ChannelSetupConfiguration& plugins_config = {"localhost", 50010UL})
    {
        constexpr auto create_credentials =
            [](const ChannelSetupConfiguration& plugins_config)
            {
                if (plugins_config.host == "localhost" || plugins_config.host == "127.0.0.1")
                {
                    spdlog::info("Create local channel on port {}.", plugins_config.port);
                    return grpc::InsecureChannelCredentials();
                }
                else if (details::ALLOW_REMOTE_PLUGINS)
                {
                    spdlog::info("Create local channel on port {}.", plugins_config.port);
                    auto creds_config = grpc::SslCredentialsOptions();
                    creds_config.pem_root_certs = resources::certificate;
                    return grpc::SslCredentials(creds_config);
                }
                // Create empty credentials, so it'll make a dummy channel where all operations fail.
                // This is consitent with creating a channel with the wrong credentials as it where.
                spdlog::warn("Remote plugins where disabled, will not connect to {}:{}.", plugins_config.host, plugins_config.port);
                return std::shared_ptr<grpc::ChannelCredentials>();
            };
        return grpc::CreateChannel(fmt::format("{}:{}", plugins_config.host, plugins_config.port), create_credentials(plugins_config));
    }

} // namespace cura::plugins

#endif // CHANNEL_H
