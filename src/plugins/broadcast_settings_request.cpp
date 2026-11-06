// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/broadcast_settings_request.h"

#include <string>
#include <unordered_map>

namespace curaengine::plugins
{
broadcast_settings_request::value_type broadcast_settings_request::operator()(const broadcast_settings_request::native_value_type& slice_message) const
{
    value_type message{};
    auto* global_settings = message.mutable_global_settings()->mutable_settings();
    for (const auto& setting : slice_message.global_settings().settings())
    {
        global_settings->emplace(setting.name(), setting.value());
    }

    auto* extruders_settings = message.mutable_extruder_settings();
    for (const auto& extruder : slice_message.extruders())
    {
        auto* settings = extruders_settings->Add()->mutable_settings();
        for (const auto& setting : extruder.settings().settings())
        {
            settings->emplace(setting.name(), setting.value());
        }
    }

    auto* object_settings = message.mutable_object_settings();
    for (const auto& mesh_group : slice_message.object_lists())
    {
        std::unordered_map<std::string, std::string> mesh_group_settings;
        for (const auto& setting : mesh_group.settings())
        {
            mesh_group_settings[setting.name()] = setting.value();
        }
        for (const auto& object : mesh_group.objects())
        {
            std::unordered_map<std::string, std::string> per_object_settings = mesh_group_settings;
            for (const auto& setting : object.settings())
            {
                per_object_settings[setting.name()] = setting.value();
            }

            per_object_settings["mesh_name"] = object.name();

            auto* settings = object_settings->Add()->mutable_settings();
            for (const auto& key_value_pair : per_object_settings)
            {
                settings->emplace(key_value_pair.first, key_value_pair.second);
            }
        }
    }

    auto* limit_to_extruder = message.mutable_limit_to_extruder();
    for (const auto& setting_extruder : slice_message.limit_to_extruder())
    {
        limit_to_extruder->emplace(setting_extruder.name(), setting_extruder.extruder());
    }
    return message;
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
