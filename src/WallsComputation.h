//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WALLS_COMPUTATION_H
#define WALLS_COMPUTATION_H

#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"

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
     * \brief Generates the walls / inner area for all parts in a layer.
     *
     * Generates walls for all parts, by calling the generateWall for the individual parts.
     *
     * \param layer The layer for which to generate the walls and inner area.
     */ 
    void generateWalls(SliceLayer* layer);

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
     * Generates the walls / inner area for a single layer part.
     *
     * \param part The part for which to generate the insets.
     */
    void generateWalls(SliceLayerPart* part);

    /*!
     * Generates the outer inset / perimeter used in spiralize mode for a single layer part. The spiral inset is
     * generated using offsets.
     *
     * \param part The part for which to generate the spiral inset.
     * \param line_width_0 The width of the outer (spiralized) wall.
     * \param wall_0_inset The part for which to generate the spiral inset.
     * \param recompute_outline_based_on_outer_wall Whether we need to recompute the print outline according to the
     *        generated spiral inset.
     */
    void generateSpiralInsets(SliceLayerPart *part, coord_t line_width_0, coord_t wall_0_inset, bool recompute_outline_based_on_outer_wall);
};
}//namespace cura

#endif//WALLS_COMPUTATION_H
