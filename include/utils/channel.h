#ifndef CHANNEL_H
#define CHANNEL_H

#include <grpcpp/create_channel.h>
#include <memory>
#include <string>

#ifndef BUILD_ALLOW_REMOTE_CHANNELS
#define BUILD_ALLOW_REMOTE_CHANNELS true
#endif

namespace cura::utils
{
    struct ChannelSetupConfiguration
    {
    public:
        std::string host;
        uint64_t port;
        uint64_t shibolet = 0UL; //TODO: Either get from startup (server would receive this as well) or remove completely.
    };

    std::shared_ptr<grpc::Channel> createChannel(const ChannelSetupConfiguration& config = { "localhost", 50010UL });

} // namespace cura::utils

#endif // CHANNEL_H
