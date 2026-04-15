// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "gcode_export/FunctionsEnvironment.h"

#include <cura-formulae-engine/parser/parser.h>

#include <range/v3/view/map.hpp>

#include "settings/SettingContainersEnvironmentAdapter.h"


namespace cfe = CuraFormulaeEngine;

namespace cura
{

FunctionsEnvironment::FunctionsEnvironment(const cfe::env::Environment* shadow_environment)
    : cfe::env::LocalEnvironment(shadow_environment)
{
    set("extruderValues", cfe::eval::Value(std::bind(&FunctionsEnvironment::extruderValues, this, std::placeholders::_1)));
}

void FunctionsEnvironment::addExtruderAdapter(const size_t extruder_nr, const std::shared_ptr<SettingContainersEnvironmentAdapter>& extruder_adapter)
{
    extruder_adapters_[extruder_nr] = extruder_adapter;
}

cfe::eval::Result FunctionsEnvironment::extruderValues(const std::vector<cfe::eval::Value>& args) const
{
    if (args.size() != 1)
    {
        return zeus::unexpected(cfe::eval::Error::InvalidNumberOfArguments);
    }

    const cfe::eval::Value& arg_setting_name = args.front();
    if (! std::holds_alternative<std::string>(arg_setting_name.value))
    {
        return zeus::unexpected(cfe::eval::Error::TypeMismatch);
    }

    const auto& setting_name = std::get<std::string>(arg_setting_name.value);

    std::vector<cfe::eval::Value> result;
    for (const std::shared_ptr<SettingContainersEnvironmentAdapter>& extruder_adapter : extruder_adapters_ | ranges::views::values)
    {
        const std::optional<CuraFormulaeEngine::eval::Value> extruder_setting_value = extruder_adapter->get(setting_name);
        if (! extruder_setting_value.has_value())
        {
            return zeus::unexpected(cfe::eval::Error::ValueError);
        }

        result.emplace_back(*extruder_setting_value);
    }
    return result;
}

} // namespace cura
