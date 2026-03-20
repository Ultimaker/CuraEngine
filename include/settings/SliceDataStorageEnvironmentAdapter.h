// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SETTINGS_SLICEDATASTORAGEENVIRONMENTADAPTER_H
#define SETTINGS_SLICEDATASTORAGEENVIRONMENTADAPTER_H

#include <cura-formulae-engine/ast/ast.h>
#include "utils/AABB.h"

namespace cura
{

class SliceDataStorage;

/*!
 * Adapter class used to expose the engine settings stack to the formulae engine resolving system
 */
class SliceDataStorageEnvironmentAdapter : public CuraFormulaeEngine::env::Environment
{
public:
    /*!
     * Base constructor
     * @param settings The settings to be used as the base contextual stack
     */
    explicit SliceDataStorageEnvironmentAdapter(const SliceDataStorage& storage, const CuraFormulaeEngine::env::Environment *next_environment = nullptr);

    [[nodiscard]] std::optional<CuraFormulaeEngine::eval::Value> get(const std::string& variable_id) const override;

    [[nodiscard]] bool has(const std::string& key) const override;

    [[nodiscard]] std::unordered_map<std::string, CuraFormulaeEngine::eval::Value> getAll() const override;

private:
    const SliceDataStorage& storage_;
    const CuraFormulaeEngine::env::Environment *const next_environment_;
    const std::vector<std::string> additional_variables_;
    std::optional<AABB> initial_layer_bb_;
};

} // namespace cura

#endif
