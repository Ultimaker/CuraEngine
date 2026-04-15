// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODEEXPORT_FUNCTIONSENVIRONMENT_H
#define GCODEEXPORT_FUNCTIONSENVIRONMENT_H

#include <cura-formulae-engine/ast/ast.h>
#include <map>

namespace cura
{

class SettingContainersEnvironmentAdapter;

class FunctionsEnvironment : public CuraFormulaeEngine::env::LocalEnvironment
{
public:
    explicit FunctionsEnvironment(const CuraFormulaeEngine::env::Environment* shadow_environment = nullptr);

    void addExtruderAdapter(const size_t extruder_nr, const std::shared_ptr<SettingContainersEnvironmentAdapter>& extruder_adapter);

private:
    CuraFormulaeEngine::eval::Result extruderValues(const std::vector<CuraFormulaeEngine::eval::Value>& args) const;

private:
    std::map<size_t, std::shared_ptr<SettingContainersEnvironmentAdapter>> extruder_adapters_;
};

} // namespace cura

#endif
