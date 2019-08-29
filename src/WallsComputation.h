//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WALLS_COMPUTATION_H
#define WALLS_COMPUTATION_H

#include "settings/types/LayerIndex.h"

namespace cura 
{

class SliceLayer;
class SliceLayerPart;

/*!
 * Function container for computing the outer walls / insets / perimeters polygons of a layer
 */
class WallsComputation
{
public:
    /*!
     * \brief Basic constructor initialising the parameters with which to
     * perform the walls computation.
     *
     * \param settings The per-mesh settings object to get setting values from.
     * \param layer_nr The layer index that these walls are generated for.
     */
    WallsComputation(const Settings& settings, const LayerIndex layer_nr);

    /*!
     * Generates the insets / perimeters for all parts in a layer.
     * 
     * Note that the second inset gets offsetted by WallsComputation::line_width_0 instead of the first, 
     * which leads to better results for a smaller WallsComputation::line_width_0 than WallsComputation::line_width_x and when printing the outer wall last.
     *
     * \param layer The layer for which to generate the insets.
     */ 
    void generateInsets(SliceLayer* layer);

private:
    /*!
     * \brief Settings container to get my settings from.
     *
     * Normally this is a mesh's settings.
     */
    const Settings& settings;

    /*!
     * \brief The layer that these walls are generated for.
     */
    const LayerIndex layer_nr;

    /*!
     * Generates the insets / perimeters for a single layer part.
     *
     * \param part The part for which to generate the insets.
     */
    void generateInsets(SliceLayerPart* part);

};
}//namespace cura

#endif//WALLS_COMPUTATION_H
