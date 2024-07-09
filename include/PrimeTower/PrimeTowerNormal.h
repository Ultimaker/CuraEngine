// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_NORMAL_H
#define PRIME_TOWER_NORMAL_H

#include "PrimeTower/PrimeTower.h"

namespace cura
{

/*!
 * Specific prime tower implementation that generates nested cylinders. Each layer, all the extruders will be used to
 * contribute to the prime tower, even if they don't actually need priming. In this case, a circular zigzag pattern will
 * be used to act as support for upper priming extrusions.
 * Although this method is not very efficient, it is required when using different materials that don't properly adhere
 * to each other. By nesting the cylinders, you make sure that the tower remains consistent and strong along the print.
 */
class PrimeTowerNormal : public PrimeTower
{
private:
    const std::vector<size_t> used_extruders_;

public:
    PrimeTowerNormal(const std::vector<size_t>& used_extruders);

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
