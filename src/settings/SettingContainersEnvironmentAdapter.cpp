// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/SettingContainersEnvironmentAdapter.h"

#include <cura-formulae-engine/parser/parser.h>

#include "Application.h"
#include "Slice.h"
#include "settings/Settings.h"

namespace cfe = CuraFormulaeEngine;

namespace cura
{

SettingContainersEnvironmentAdapter::SettingContainersEnvironmentAdapter(const std::optional<int> target_extruder_nr)
    : settings_(
          target_extruder_nr.has_value() ? &Application::getInstance().current_slice_->scene.extruders.at(target_extruder_nr.value()).settings_
                                         : &Application::getInstance().current_slice_->scene.settings)
{
}

std::optional<cfe::eval::Value> SettingContainersEnvironmentAdapter::get(const std::string& setting_id) const
{
    if (! settings_->has(setting_id))
    {
        return std::nullopt;
    }

    const std::string setting_raw_value = settings_->get<std::string>(setting_id);

    zeus::expected<cfe::ast::ExprPtr, error_t> parse_result = cfe::parser::parse(setting_raw_value);
    if (! parse_result.has_value())
    {
        return cfe::eval::Value(setting_raw_value);
    }

    cfe::env::EnvironmentMap env;
    const cfe::ast::ExprPtr& expression = parse_result.value();
    cfe::eval::Result eval_result = expression.evaluate(&env);

    if (! eval_result.has_value())
    {
        return std::nullopt;
    }

    return eval_result.value();
}

bool SettingContainersEnvironmentAdapter::has(const std::string& key) const
{
    return settings_->has(key);
}

std::unordered_map<std::string, cfe::eval::Value> SettingContainersEnvironmentAdapter::getAll() const
{
    std::unordered_map<std::string, cfe::eval::Value> result;
    for (const std::string& key : settings_->getKeys())
    {
        std::optional<cfe::eval::Value> value = get(key);
        if (value.has_value())
        {
            result[key] = value.value();
        }
    }
    return result;
}

} // namespace cura