// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "prime_tower/PrimeTower.h"

namespace curaengine::prime_tower
{

/*!
 * Specific prime tower implementation that generates interleaved priming paths. It is optimized to waste as few
 * filament as possible, while ensuring that the prime tower is still robust even if it gets very high.
 * When there is no actual priming required for extruders, it will create a kind of circular zigzag pattern that acts as
 * a sparse support. Otherwise it will create priming annuli, stacked on top of each other.
 * This is very effective when doing multi-color printing, however it can be used only if all the filaments properly
 * adhere to each other. Otherwise there is a high risk that the tower will collapse during the print.
 */
class PrimeTowerInterleaved : public PrimeTower
{
public:
    PrimeTowerInterleaved();

    virtual ExtruderPrime getExtruderPrime(
        const std::vector<bool>& extruder_is_used_on_this_layer,
        size_t extruder_nr,
        size_t last_extruder,
        const SliceDataStorage& storage,
        const types::LayerIndex& layer_nr) const override;

protected:
    virtual void polishExtrudersUses(utils::LayerVector<std::vector<ExtruderUse>>& extruders_use, const size_t start_extruder) override;

    virtual std::map<types::LayerIndex, std::vector<ExtruderToolPaths>> generateToolPaths(const utils::LayerVector<std::vector<ExtruderUse>>& extruders_use) override;
};

} // namespace curaengine::prime_tower
