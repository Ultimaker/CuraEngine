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
 * \param extrusionWidth extrusionWidth
 * \param insetCount The number of insets to to generate
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p part)
 */
void generateInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters);

/*!
 * Generates the insets / perimeters for all parts in a layer.
 * 
 * \param layer The layer for which to generate the insets.
 * \param extrusionWidth extrusionWidth
 * \param insetCount The number of insets to to generate
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p part)
 */ 
void generateInsets(SliceLayer* layer, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters);

}//namespace cura

#endif//INSET_H
