//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIGHTNING_GENERATOR_H
#define LIGHTNING_GENERATOR_H

#include "LightningLayer.h"

#include "../utils/polygonUtils.h"

#include <functional>
#include <memory>
#include <vector>

namespace cura 
{
class SliceMeshStorage;

class LightningGenerator  // "Just like Nicola used to make!"
{
public:
    /*
     * Generate lightning fill aka minfill aka 'Ribbed Support Vault Infill', see Tricard, Claux, Lefebvre / 'Ribbed Support Vaults for 3D Printing of Hollowed Objects'
     * see https ://hal.archives-ouvertes.fr/hal-02155929/document
     */
    LightningGenerator(const SliceMeshStorage& mesh);

    const LightningLayer& getTreesForLayer(const size_t& layer_id) const;

protected:
    // Necesary, since normally overhangs are only generated for the outside of the model, and only when support is generated.
    void generateInitialInternalOverhangs(const SliceMeshStorage& mesh);

    void generateTrees(const SliceMeshStorage& mesh);

    coord_t supporting_radius;
    coord_t wall_supporting_radius;
    coord_t prune_length;
    coord_t straightening_max_distance;

    std::vector<Polygons> overhang_per_layer;
    std::vector<LightningLayer> lightning_layers;
};

} // namespace cura

#endif // LIGHTNING_GENERATOR_H
