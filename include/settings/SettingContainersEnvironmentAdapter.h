// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SETTINGS_SETTINGCONTAINERENVIRONMENTADAPTER_H
#define SETTINGS_SETTINGCONTAINERENVIRONMENTADAPTER_H

#include <cura-formulae-engine/ast/ast.h>

namespace cura
{

class Settings;

class SettingContainersEnvironmentAdapter : public CuraFormulaeEngine::env::Environment
{
public:
    explicit SettingContainersEnvironmentAdapter(const std::optional<int> target_extruder_nr = std::nullopt);

    [[nodiscard]] std::optional<CuraFormulaeEngine::eval::Value> get(const std::string& setting_id) const override;

    [[nodiscard]] bool has(const std::string& key) const override;

    [[nodiscard]] std::unordered_map<std::string, CuraFormulaeEngine::eval::Value> getAll() const override;

private:
    Settings* settings_{};
};

} // namespace cura

#endif