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

    virtual std::map<LayerIndex, std::map<size_t, Shape>>
        generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage) override;

protected:
    virtual bool requiresBaseExtraPrint(size_t extruder_nr) const override;

    virtual bool requiresFirstLayerExtraInnerPrint(size_t extruder_nr) const override;

    virtual std::map<size_t, std::map<size_t, Shape>> generateSparseInfillImpl(const std::vector<coord_t>& rings_radii) const override;

    virtual std::vector<size_t> findExtrudersSparseInfill(
        const LayerPlan& gcode_layer,
        const std::vector<ExtruderUse>& required_extruder_prime,
        const std::vector<size_t>& initial_list_idx = {}) const override;

    virtual void processExtruderNoPrime(const size_t extruder_nr, LayerPlan& gcode_layer) const override;
};

} // namespace cura

#endif // PRIME_TOWER_NORMAL_H
