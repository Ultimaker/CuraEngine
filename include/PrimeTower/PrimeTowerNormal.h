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
    PrimeTowerNormal(size_t extruder_count);

    virtual ExtruderPrime getExtruderPrime(
        const std::vector<bool>& extruder_is_used_on_this_layer,
        size_t extruder_nr,
        size_t last_extruder,
        const SliceDataStorage& storage,
        const LayerIndex& layer_nr) const override;

protected:
    virtual std::map<LayerIndex, std::vector<ExtruderMoves>>
        generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage) override;

private:
    /*
     * In which order, from outside to inside, will we be printing the prime
     * towers for maximum strength?
     *
     * This is the spatial order from outside to inside. This is NOT the actual
     * order in time in which they are printed.
     */
    std::vector<size_t> extruder_order_;
};

} // namespace cura

#endif // PRIME_TOWER_NORMAL_H
