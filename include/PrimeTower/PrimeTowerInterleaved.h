// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_INTERLEAVED_H
#define PRIME_TOWER_INTERLEAVED_H

#include "PrimeTower/PrimeTower.h"

namespace cura
{

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

    virtual std::map<LayerIndex, std::vector<ExtruderMoves>> generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use) override;
};

} // namespace cura

#endif // PRIME_TOWER_INTERLEAVED_H
