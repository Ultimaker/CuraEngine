/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INSET_H
#define INSET_H

#include "sliceDataStorage.h"

namespace cura 
{

/*!
 * Generates the insets / perimeters for a single layer part.
 * 
 * \param part The part for which to generate the insets.
 * \param wall_0_inset The offset applied to the outer wall
 * \param line_width_0 line width of the outer wall
 * \param line_width_x line width of other walls
 * \param insetCount The number of insets to to generate
 * \param avoidOverlappingPerimeters_0 Whether to remove the parts of the first perimeters where it have overlap with itself (and store the gaps thus created in the \p storage)
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p part)
 */
void generateInsets(SliceLayerPart* part, int wall_0_inset, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters);

/*!
 * Generates the insets / perimeters for all parts in a layer.
 * 
 * Note that the second inset gets offsetted by \p line_width_0 instead of the first, 
 * which leads to better results for a smaller \p line_width_0 than \p line_width_x and when printing the outer wall last.
 * 
 * \param layer The layer for which to generate the insets.
 * \param wall_0_inset The offset applied to the outer wall
 * \param line_width_0 line width of the outer wall
 * \param line_width_x line width of other walls
 * \param insetCount The number of insets to to generate
 * \param avoidOverlappingPerimeters_0 Whether to remove the parts of the first perimeters where it have overlap with itself (and store the gaps thus created in the \p storage)
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p part)
 */ 
void generateInsets(SliceLayer* layer, int wall_0_inset, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters);

}//namespace cura

#endif//INSET_H
