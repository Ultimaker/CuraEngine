//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WALLS_COMPUTATION_H
#define WALLS_COMPUTATION_H

#include "sliceDataStorage.h"

namespace cura 
{

/*!
 * Function container for computing the outer walls / insets / perimeters polygons of a layer
 */
class WallsComputation
{
public:
    /*!
     * The offset applied to the outer wall
     */
    int wall_0_inset;
    /*!
     * line width of the outer wall
     */
    int line_width_0;
    /*!
     * line width of other walls
     */
    int line_width_x;
    /*!
     * The number of insets to to generate
     */
    size_t inset_count;
    /*!
     * Whether to compute a more accurate poly representation of the printed outlines, based on the outer wall
     */
    bool recompute_outline_based_on_outer_wall;

    /*!
     * Whether to remove parts which have no insets.
     */
    bool remove_parts_with_no_insets;

    /*!
     * Basic constructor initializing the parameters with which to perform the
     * walls computation.
     * 
     * \param wall_0_inset The offset applied to the outer wall.
     * \param line_width_0 Line width of the outer wall.
     * \param line_width_x Line width of other walls.
     * \param inset_count The number of insets to to generate.
     * \param recompute_outline_based_on_outer_wall Whether to compute a more
     * accurate poly representation of the printed outlines, based on the outer
     * wall.
     * \param remove_parts_with_no_insets Whether to remove parts if they get no
     * single inset.
     */
    WallsComputation(int wall_0_inset, int line_width_0, int line_width_x, size_t inset_count, bool recompute_outline_based_on_outer_wall, bool remove_parts_with_no_insets);

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
     * Generates the insets / perimeters for a single layer part.
     *
     * \param part The part for which to generate the insets.
     */
    void generateInsets(SliceLayerPart* part);

};
}//namespace cura

#endif//WALLS_COMPUTATION_H
