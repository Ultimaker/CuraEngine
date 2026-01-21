// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/SettingContainersEnvironmentAdapter.h"

#include <cura-formulae-engine/parser/parser.h>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/map.hpp>

#include "settings/Settings.h"

namespace cfe = CuraFormulaeEngine;

namespace cura
{

SettingContainersEnvironmentAdapter::SettingContainersEnvironmentAdapter(const Settings& settings)
    : settings_(settings)
{
}

std::optional<cfe::eval::Value> SettingContainersEnvironmentAdapter::get(const std::string& setting_id) const
{
    constexpr bool parent_lookup = true;
    if (! settings_.has(setting_id, parent_lookup))
    {
        return std::nullopt;
    }

    const std::string setting_raw_value = settings_.get<std::string>(setting_id);

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
        // Expressions are being evaluated for numbers, but strings are being recognized as variable identifiers, which then fail to evaluate, so treat them as raw strings instead
        return cfe::eval::Value(setting_raw_value);
    }

    return eval_result.value();
}

bool SettingContainersEnvironmentAdapter::has(const std::string& key) const
{
    constexpr bool parent_lookup = true;
    return settings_.has(key, parent_lookup);
}

std::unordered_map<std::string, cfe::eval::Value> SettingContainersEnvironmentAdapter::getAll() const
{
    std::unordered_map<std::string, cfe::eval::Value> result;
    for (const std::string& key : settings_.getKeys())
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