// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_INTERLEAVED_H
#define PRIME_TOWER_INTERLEAVED_H

#include "PrimeTower/PrimeTower.h"

namespace cura
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
        const LayerIndex& layer_nr) const override;

protected:
    virtual void polishExtrudersUses(LayerVector<std::vector<ExtruderUse>>& extruders_use, const size_t start_extruder) override;

    virtual std::map<LayerIndex, std::vector<ExtruderToolPaths>> generateToolPaths(const LayerVector<std::vector<ExtruderUse>>& extruders_use) override;
};

} // namespace cura

#endif // PRIME_TOWER_INTERLEAVED_H
