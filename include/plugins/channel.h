#ifndef CHANNEL_H
#define CHANNEL_H

#define REMOTE_PLUGIN_BUILD
// ^^ TODO: remove later & move to build-system

#include "plugins/slots.h"

#ifdef REMOTE_PLUGIN_BUILD
#include "../secrets/plugins.crt.h"
#include "../secrets/plugins.key.h"
#endif

namespace cura::plugins
{
    struct ChannelSetupConfiguration
    {
    public:
        std::string host;
        uint64_t port;
        uint64_t shibolet = 0UL; //TODO: Either get from startup (plugin would receive this as well) or remove completely.
    };

    auto createChannel(const ChannelSetupConfiguration& plugins_config = {"localhost", 50010UL})
    {
#ifdef REMOTE_PLUGIN_BUILD
        auto creds_config = grpc::SslCredentialsOptions();
        creds_config.pem_root_certs = secrets::certificate;
        creds_config.pem_cert_chain = secrets::certificate;
        creds_config.pem_private_key = secrets::private_key;
        auto channel_creds = grpc::SslCredentials(creds_config);
#else // not REMOTE_PLUGIN_BUILD
        auto channel_creds = grpc::InsecureChannelCredentials();
#endif
        return grpc::CreateChannel(fmt::format("{}:{}", plugins_config.host, plugins_config.port), channel_creds);
    }

} // namespace cura::plugins

#endif // CHANNEL_H
