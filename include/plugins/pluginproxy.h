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

#include "plugins/types.h"
#include "plugins/validator.h"

#include "plugin.pb.h"

namespace cura::plugins
{

namespace detail
{
template<class Validator, class Converter>
class PluginListener : public Arcus::SocketListener
{
public:
    using validator_t = Validator;
    using receive_t = Converter::receive_t;
    using converter_t = Converter;

    bool msgReceived() noexcept
    {
        bool msg_received = msg_received_;
        msg_received_ = false;
        return msg_received;
    }

private:
    std::shared_ptr<validator_t> validator_{};
    converter_t converter_{};
    bool msg_received_{ false };

public:
    explicit PluginListener(std::shared_ptr<validator_t> validator) noexcept : validator_{ validator_ }
    {
    }

    void stateChanged(Arcus::SocketState) override{};

    void messageReceived() override
    {
//        auto message = getSocket()->takeNextMessage();
//
//        if (message->GetTypeName() == "cura.plugins.proto.Plugin_ret")
//        {
//            auto* msg = dynamic_cast<proto::Plugin_ret*>(message.get());
//            cura::plugins::proto::Plugin_ret mm {};
//            auto x = converter_(mm);
////            validator_->version = semver::from_string(version);
////            validator_->plugin_hash = plugin_hash;
//            spdlog::info("Plugin version: {}", validator_->version.to_string());
//        }
//        else if (message->GetTypeName() == receive_t::default_instance().GetTypeName())
//        {
//            // unblock plugin
//        }
    };

    void error(const Arcus::Error& error) override{};
};

} // namespace detail

template<plugins::SlotID Slot, class Validator, class Converter>
class PluginProxy
{
public:
    // type aliases for easy use
    using receive_t = Converter::receive_t;
    using send_t = Converter::send_t;
    using validator_t = Validator;
    using converter_t = Converter;
    using listener_t = detail::PluginListener<validator_t, converter_t>;

    plugins::SlotID slot_id{ Slot };

    std::shared_ptr<validator_t> validator;

private:
    std::unique_ptr<Arcus::Socket> socket_;
    listener_t* listener_; // FIXME: in Arcus use smart_ptr for listeners otherwise we need to add a deconstructor in this class and that forces us to define the big 6
    converter_t converter_{};

public:
    PluginProxy(const std::string& ip, int port) : validator{ std::make_shared<validator_t>() }, socket_{ std::make_unique<Arcus::Socket>() }, listener_{ new listener_t(validator) }
    {
        // Add the listener
        socket_->addListener(listener_);

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

        converters::simplify_converter_fn<send_t, receive_t > convvvvv{};

        SlotID slotId { SlotID::SIMPLIFY };
        auto x = convvvvv(slotId);

        proto::Plugin_args args{};
        args.set_id(slot_id);

        socket_->sendMessage(std::make_shared<proto::Plugin_args>(args));
        if (listener_->msgReceived())
        {
            auto message = socket_->takeNextMessage();
            if (message->GetTypeName() == "cura.plugins.proto.Plugin_ret")
            {
                auto* msg = dynamic_cast<proto::Plugin_ret*>(message.get());
                validator->version = semver::from_string(msg->version());
                validator->plugin_hash = msg->plugin_hash();
                spdlog::info("Plugin version: {}", validator->version.to_string());
            }
        }
        //
        // No need to wait for the response, since the listener will set the version
    }

    auto operator()(auto&&... args)
    {
        if (validator && socket_->getState() == Arcus::SocketState::Connected)
        {
            socket_->sendMessage(converter_(std::forward<decltype(args)>(args)...));
            // TODO: Block until message is received
            // TODO: Convert return message to actual return value
            return 1; // FIXME: This is not correct
        }
        return 1; // FIXME: handle plugin not connected
    }
};

} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_PLUGINPROXY_H
