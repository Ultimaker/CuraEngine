//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIGHTNING_GENERATOR_H
#define LIGHTNING_GENERATOR_H

#include "LightningLayer.h"

#include "../utils/polygonUtils.h"

#include <functional>
#include <memory>
#include <vector>
#include <unordered_set>

namespace cura 
{
class SliceMeshStorage;

/*
 *            .---------.
 *     . :+*#%%@*=\\\*@@@#:
 *  .+*=+@%#%@@@@#%@@@@@@@@*
 * :@@%+=@##==%@@@@@@@@@@@@@
 * %@@%=-'     '"-+*#%@@@@@@@.
 * %@#'            ...*=@@@@@-
 * .-             ....*=@@@@@*
 *  .        ..:......:#@@@@@=
 *  : :-- .-*%@%%**=-:.-%@@#=.=.
 *   =##%: :==-.:::..:::=@+++. )
 *   :     ..       .::--:-#% /
 *    \    ...     ..:---==:_;
 *     :  :=w=:   ..:----+=
 *      :-#@@@%#*-.:::---==
 *      :*=--==--:.:----=-=.
 *       . .-=-...:--=+*+-+=:.
 *        \     .-=+:'       .:
 *         .':-==-"  .:-=+#%@@@*
 *       .'      :+#@@@@@@@@@@@@+
 *    .=#%#. :-+#%@@@@@@@@@@@@@@@:
 *  -+%##*+#***%@%####%@@@@@@@@@@@*.
 * 
 *                           <3 Nikolai
 */
class LightningGenerator
{
public:
    /*!
     * TODO: instead of radius we should pass around the overhang_angle
     * and compute the radius from the tangent of the angle and the local (adaptive) layer thickness
     */
    LightningGenerator(const coord_t& radius, const SliceMeshStorage& mesh);

    const LightningLayer& getTreesForLayer(const size_t& layer_id);

protected:
    // Necesary, since normally overhangs are only generated for the outside of the model, and only when support is generated.
    void generateInitialInternalOverhangs(const SliceMeshStorage& mesh, coord_t supporting_radius);

    void generateTrees(const SliceMeshStorage& mesh);

    coord_t supporting_radius;
    std::vector<Polygons> overhang_per_layer;
    std::vector<LightningLayer> lightning_layers;
};

} // namespace cura

#endif // LIGHTNING_GENERATOR_H
