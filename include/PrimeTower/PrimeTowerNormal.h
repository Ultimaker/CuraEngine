// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_NORMAL_H
#define PRIME_TOWER_NORMAL_H

#include "PrimeTower/PrimeTower.h"

namespace cura
{

class PrimeTowerNormal : public PrimeTower
{
public:
    PrimeTowerNormal();

    virtual ExtruderPrime getExtruderPrime(
        const std::vector<bool>& extruder_is_used_on_this_layer,
        size_t extruder_nr,
        size_t last_extruder,
        const SliceDataStorage& storage,
        const LayerIndex& layer_nr) const override;

protected:
    virtual std::map<LayerIndex, std::vector<ExtruderToolPaths>> generateToolPaths(const LayerVector<std::vector<ExtruderUse>>& extruders_use) override;
};

} // namespace cura

#endif // PRIME_TOWER_NORMAL_H
