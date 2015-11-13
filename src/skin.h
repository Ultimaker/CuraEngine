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
 * \param innermost_wall_extrusion_width The line width of the inner most wall
 * \param insetCount The number of perimeters to surround the skin
 * \param no_small_gaps_heuristic A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
 * \param avoidOverlappingPerimeters_0 Whether to remove the parts of the first perimeters where it have overlap with itself (and store the gaps thus created in the \p storage)
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p storage)
 */
void generateSkins(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int innermost_wall_extrusion_width, int insetCount, bool no_small_gaps_heuristic, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters);

/*!
 * Generate the skin areas (outlines)
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param storage The storage where the layer outline information (input) is stored and where the skin outline (output) is stored.
 * \param extrusionWidth extrusionWidth
 * \param downSkinCount The number of layers of bottom skin
 * \param upSkinCount The number of layers of top skin
 * \param no_small_gaps_heuristic A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
 */
void generateSkinAreas(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, bool no_small_gaps_heuristic);

/*!
 * Generate the skin insets.
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param part The part where the skin outline information (input) is stored and where the skin insets (output) are stored.
 * \param extrusionWidth extrusionWidth
 * \param insetCount The number of perimeters to surround the skin
 * \param avoidOverlappingPerimeters_0 Whether to remove the parts of the first perimeters where it have overlap with itself (and store the gaps thus created in the \p storage)
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p storage)
 */
void generateSkinInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters);

/*!
 * Generate Infill
 * \param layerNr The index of the layer for which to generate the infill
 * \param part The part where the insets (input) are stored and where the infill (output) is stored.
 * \param extrusionWidth width of the wall lines
 * \param infill_skin_overlap overlap distance between infill and skin
 */
void generateInfill(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int infill_skin_overlap);

/*!
 * \brief Combines the infill of multiple layers for a specified mesh.
 * 
 * The infill layers are combined while the thickness of each layer is
 * multiplied such that the infill should fill up again to the full height of
 * all combined layers.
 * 
 * \param storage The mesh to combine the infill layers of.
 * \param amount The number of layers to combine.
 */
void combineInfillLayers(SliceMeshStorage& storage,unsigned int amount);

}//namespace cura

#endif//SKIN_H
