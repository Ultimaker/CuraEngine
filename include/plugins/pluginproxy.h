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
using SlotID = proto::SlotID;

namespace detail
{
template<class Plugin>
class PluginListener : public Arcus::SocketListener
{
public:
    PluginListener(std::shared_ptr<Plugin> plugin) noexcept : plugin_{ plugin }
    {
    }

    void stateChanged(Arcus::SocketState) override{};

    void messageReceived() override
    {
        auto message = getSocket()->takeNextMessage();

        if (message->GetTypeName() == "cura.plugins.proto.Plugin_ret")
        {
            auto* plugin_ret = dynamic_cast<plugins::proto::Plugin_ret*>(message.get());
            plugin_->validator->version = semver::from_string( plugin_ret->version());
            plugin_->validator->plugin_hash = plugin_ret->plugin_hash();
            spdlog::info("Plugin version: {}", plugin_->validator->version.to_string());
        }
        else if (message->GetTypeName() == Plugin::receive_t::default_instance().GetTypeName())
        {
            // unblock plugin
        }
    };

    void error(const Arcus::Error& error) override{};

private:
    std::shared_ptr<Plugin> plugin_{};
};

} // namespace detail

template<SlotID Slot, class Validator, class Converter>
class PluginProxy
{
public:
    using receive_t = Converter::receive_t;
    using send_t = Converter::send_t;
    using validator_t = Validator;
    using plugin_t = PluginProxy<Slot, Validator, Converter>;
    static constexpr SlotID slot_id{ Slot };

    std::unique_ptr<Arcus::Socket> socket_{};
    validator_t validator{};
    std::unique_ptr<detail::PluginListener<plugin_t>> listener_{};
    Converter converter{ };

    PluginProxy(const std::string& ip, int port) noexcept
        : socket_{ std::make_unique<Arcus::Socket>() }
        , listener_{ std::make_unique<detail::PluginListener>(std::make_shared<plugin_t>(this)) }
    {
        // Add the listener
        socket_->addListener(listener_.get());

        // Register all receiving message types
        socket_->registerMessageType(&plugins::proto::Plugin_ret::default_instance());
        socket_->registerMessageType(&receive_t::default_instance());

        // Connect to the plugin
        spdlog::info("Connecting to plugin at {}:{}", ip, port);
        socket_->connect(ip, port);
        while (socket_->getState() == Arcus::SocketState::Connecting || socket_->getState() == Arcus::SocketState::Opening)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{ 100 }); // TODO: Fix magic number
            // TODO: Add timeout??? Is this blocking even necessary?
        }

        // TODO: Use Plugin Message instead of Version_args
        auto plugin_args = std::make_shared<plugins::proto::Plugin_args>();
        plugin_args->set_id(slot_id);
        socket_->sendMessage(plugin_args);
        // No need to wait for the response, since the listener will set the version
    }

    auto operator()(auto&&... args)
    {
        if (validator && socket_->getState() == Arcus::SocketState::Connected)
        {
            socket_->sendMessage(converter(std::forward<decltype(args)>(args)...));
            // TODO: Block until message is received
            // TODO: Convert return message to actual return value
            return 1; // FIXME: This is not correct
        }
        return 1;  // FIXME: handle plugin not connected
    }
};

} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_PLUGINPROXY_H
