// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_PLUGINPROXY_H
#define CURAENGINE_INCLUDE_PLUGINS_PLUGINPROXY_H

#include <functional>
#include <memory>

#include <Arcus/Socket.h>
#include <Arcus/SocketListener.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <semver.hpp>
#include <spdlog/spdlog.h>

#include "plugins/validator.h"

#include "plugin.pb.h"

namespace cura::plugins
{
namespace detail
{
template<class Validator>
class PluginListener : public Arcus::SocketListener
{
public:
    PluginListener(std::shared_ptr<Validator> validator) noexcept : validator_{ validator }
    {
    }

    void stateChanged(Arcus::SocketState) override{};

    void messageReceived() override
    {
        auto message = getSocket()->takeNextMessage();
        // Check if the message is a Version_ret and update the validator
        if (message->GetDescriptor() == &plugins::proto::Version_ret::default_instance())
        {
            auto* version_ret = dynamic_cast<plugins::proto::Version_ret*>(message.get());
            validator_->version = semver::from_string(version_ret->version());
            spdlog::info("Plugin version: {}", validator_->version.to_string());
        }

        // TODO: Handle Receive and Send messages
    };

    void error(const Arcus::Error& error) override{};

private:
    std::shared_ptr<Validator> validator_{};
};

} // namespace detail

template<VersionRangeLiteral VersionRange, class Receive, class Send, class Proj = std::identity>
class PluginProxy
{
    std::unique_ptr<Arcus::Socket> socket_{};
    std::shared_ptr<Validator<VersionRange>> validator_{};
    std::unique_ptr<detail::PluginListener<Validator<VersionRange>>> listener_{};
    ranges::semiregular_box<Proj> projection_{};

    PluginProxy(const std::string& ip, int port, Proj&& proj = Proj{}) noexcept
        : socket_{ std::make_unique<Arcus::Socket>() }
        , validator_{ std::make_shared<Validator<VersionRange>>() }
        , listener_{ std::make_unique<detail::PluginListener>(validator_) }
        , projection_{ std::forward<Proj>(proj) }
    {
        // Add the listener
        socket_->addListener(listener_.get());

        // Register all message types
        socket_->registerMessageType(&plugins::proto::Version_ret::default_instance());
        socket_->registerMessageType(&Receive::default_instance());
        socket_->registerMessageType(&Send::default_instance());

        // Connect to the plugin
        spdlog::info("Connecting to plugin at {}:{}", ip, port);
        socket_->connect(ip, port);
        while (socket_->getState() == Arcus::SocketState::Connecting || socket_->getState() == Arcus::SocketState::Opening)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{ 100 }); // TODO: Fix magic number
            // TODO: Add timeout??? Is this blocking even necessary?
        }

        // Send request for version
        auto version_args = std::make_shared<plugins::proto::Version_args>();
        version_args->set_version_range(VersionRange.value);
        socket_->sendMessage(version_args);
    }

    auto operator()(auto&&... args)
    {
        if (*validator_ && socket_->getState() == Arcus::SocketState::Connected)
        {
            auto send = std::make_shared<Send>(std::forward<decltype(args)>(args)...);
            socket_->sendMessage(send);
            // TODO: Block until message is received
            // TODO: Convert return message to actual return value
            return projection_(std::forward<decltype(args)>(args)...); // FIXME: This is not correct
        }
        return projection_(std::forward<decltype(args)>(args)...);
    }
};

} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_PLUGINPROXY_H
