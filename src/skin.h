/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIN_H
#define SKIN_H

#include "sliceDataStorage.h"

namespace cura 
{

/*!
 * Generate the gap areas which occur between consecutive insets.
 * 
 * \param layerNr The index of the layer for which to generate the gaps.
 * \param storage The storage where the layer outline information (input) is stored and where the gap areas (output) are stored.
 * \param extrusionWidth extrusionWidth
 * \param downSkinCount The number of layers of bottom gaps
 * \param upSkinCount The number of layers of top gaps
 */
void generatePerimeterGaps(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount);

/*!
 * Generate the skin areas and its insets.
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param storage The storage where the layer outline information (input) is stored and where the skin insets and fill areas (output) are stored.
 * \param extrusionWidth extrusionWidth
 * \param downSkinCount The number of layers of bottom skin
 * \param upSkinCount The number of layers of top skin
 * \param insetCount The number of perimeters to surround the skin
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p storage)
 */
void generateSkins(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int insetCount, int avoidOverlappingPerimeters);

/*!
 * Generate the skin areas (outlines)
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param storage The storage where the layer outline information (input) is stored and where the skin outline (output) is stored.
 * \param extrusionWidth extrusionWidth
 * \param downSkinCount The number of layers of bottom skin
 * \param upSkinCount The number of layers of top skin
 */
void generateSkinAreas(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount);

/*!
 * Generate the skin insets.
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param part The part where the skin outline information (input) is stored and where the skin insets (output) are stored.
 * \param extrusionWidth extrusionWidth
 * \param insetCount The number of perimeters to surround the skin
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p storage)
 */
void generateSkinInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters);


void generateSparse(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int avoidOverlappingPerimeters);
void combineSparseLayers(int layerNr, SliceMeshStorage& storage, int amount);

}//namespace cura

#endif//SKIN_H
