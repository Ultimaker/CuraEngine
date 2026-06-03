// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SETTINGS_SETTINGCONTAINERENVIRONMENTADAPTER_H
#define SETTINGS_SETTINGCONTAINERENVIRONMENTADAPTER_H

#include <cura-formulae-engine/ast/ast.h>

namespace cura
{

class Settings;

/*!
 * Adapter class used to expose the engine settings stack to the formulae engine resolving system
 */
class SettingContainersEnvironmentAdapter : public CuraFormulaeEngine::env::ChainableEnvironment
{
public:
    /*!
     * Base constructor
     * @param settings The settings to be used as the base contextual stack
     */
    explicit SettingContainersEnvironmentAdapter(const Settings& settings, const CuraFormulaeEngine::env::Environment* shadow_environment = nullptr);

protected:
    [[nodiscard]] std::optional<CuraFormulaeEngine::eval::Value> getImpl(const std::string& setting_id) const noexcept override;

    [[nodiscard]] bool hasImpl(const std::string& key) const noexcept override;

    [[nodiscard]] std::unordered_map<std::string, CuraFormulaeEngine::eval::Value> getAllImpl() const noexcept override;

private:
    const Settings& settings_;
};

} // namespace cura

#endif
