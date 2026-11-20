// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/broadcast_settings_request.h"

#include <string>
#include <unordered_map>

#include "Scene.h"

namespace curaengine::plugins
{
broadcast_settings_request::value_type broadcast_settings_request::operator()(const broadcast_settings_request::native_value_type& scene) const
{
    value_type message{};
    auto* global_settings = message.mutable_global_settings()->mutable_settings();
    for (const auto& [setting_name, setting_value] : scene.settings.getRawSettings())
    {
        global_settings->emplace(setting_name, setting_value);
    }

    auto* extruders_settings = message.mutable_extruder_settings();
    for (const auto& extruder : scene.extruders)
    {
        auto* settings = extruders_settings->Add()->mutable_settings();
        for (const auto& [setting_name, setting_value] : extruder.settings_.getRawSettings())
        {
            settings->emplace(setting_name, setting_value);
        }
    }

    auto* object_settings = message.mutable_object_settings();
    for (const auto& mesh_group : scene.getMeshGroups())
    {
        std::unordered_map<std::string, std::string> mesh_group_settings;
        for (const auto& [setting_name, setting_value] : mesh_group->settings.getRawSettings())
        {
            mesh_group_settings[setting_name] = setting_value;
        }
        for (const auto& object : mesh_group->meshes)
        {
            std::unordered_map<std::string, std::string> per_object_settings = mesh_group_settings;
            for (const auto& [setting_name, setting_value] : object.settings_.getRawSettings())
            {
                per_object_settings[setting_name] = setting_value;
            }

            per_object_settings["mesh_name"] = object.mesh_name_;

            auto* settings = object_settings->Add()->mutable_settings();
            for (const auto& key_value_pair : per_object_settings)
            {
                settings->emplace(key_value_pair.first, key_value_pair.second);
            }
        }
    }

    auto* limit_to_extruder = message.mutable_limit_to_extruder();
    for (const auto& [setting_name, extruder_train] : scene.limit_to_extruder)
    {
        limit_to_extruder->emplace(setting_name, extruder_train->extruder_nr_);
    }
    return message;
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
