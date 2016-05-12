/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIN_H
#define SKIN_H

#include "sliceDataStorage.h"

namespace cura 
{

/*!
 * Limit the gap areas which occur between consecutive insets to skin only.
 * 
 * \param layerNr The index of the layer for which to generate the gaps.
 * \param mesh The storage where the layer outline information (input) is stored and where the gap areas (output) are stored.
 * \param extrusionWidth extrusionWidth
 * \param downSkinCount The number of layers of bottom gaps
 * \param upSkinCount The number of layers of top gaps
 */
void generatePerimeterGaps(int layerNr, SliceMeshStorage& mesh, int extrusionWidth, int downSkinCount, int upSkinCount);

/*!
 * Generate the skin areas and its insets.
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param mesh The storage where the layer outline information (input) is stored and where the skin insets and fill areas (output) are stored.
 * \param extrusionWidth extrusionWidth
 * \param downSkinCount The number of layers of bottom skin
 * \param upSkinCount The number of layers of top skin
 * \param wall_line_count The number of walls, i.e. the number of the wall from which to offset.
 * \param innermost_wall_extrusion_width The line width of the inner most wall
 * \param insetCount The number of perimeters to surround the skin
 * \param no_small_gaps_heuristic A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
 * \param avoidOverlappingPerimeters_0 Whether to remove the parts of the first perimeters where it have overlap with itself (and store the gaps thus created in the \p mesh)
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p mesh)
 */
void generateSkins(int layerNr, SliceMeshStorage& mesh, int extrusionWidth, int downSkinCount, int upSkinCount, int wall_line_count, int innermost_wall_extrusion_width, int insetCount, bool no_small_gaps_heuristic, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters);

/*!
 * Generate the skin areas (outlines)
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param mesh The storage where the layer outline information (input) is stored and where the skin outline (output) is stored.
 * \param extrusionWidth extrusionWidth
 * \param downSkinCount The number of layers of bottom skin
 * \param upSkinCount The number of layers of top skin
 * \param wall_line_count The number of walls, i.e. the number of the wall from which to offset.
 * \param no_small_gaps_heuristic A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
 */
void generateSkinAreas(int layerNr, SliceMeshStorage& mesh, int extrusionWidth, int downSkinCount, int upSkinCount, int wall_line_count, bool no_small_gaps_heuristic);

/*!
 * Generate the skin insets.
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param part The part where the skin outline information (input) is stored and where the skin insets (output) are stored.
 * \param extrusionWidth extrusionWidth
 * \param insetCount The number of perimeters to surround the skin
 * \param avoidOverlappingPerimeters_0 Whether to remove the parts of the first perimeters where it have overlap with itself (and store the gaps thus created in the \p mesh)
 * \param avoidOverlappingPerimeters Whether to remove the parts of two consecutive perimeters where they have overlap (and store the gaps thus created in the \p mesh)
 */
void generateSkinInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters);

/*!
 * Generate Infill by offsetting from the last wall.
 * 
 * The walls should already be generated.
 * 
 * After this function has been called on a layer of a mesh, each SliceLayerPart of that layer should have an infill_area consisting of exactly one Polygons : the normal uncombined infill area.
 * 
 * \param layerNr The index of the layer for which to generate the infill
 * \param mesh The storage where the layer outline information (input) is stored and where the skin outline (output) is stored.
 * \param part The part where the insets (input) are stored and where the infill (output) is stored.
 * \param innermost_wall_extrusion_width width of the innermost wall lines
 * \param infill_skin_overlap overlap distance between infill and skin
 * \param wall_line_count The number of walls, i.e. the number of the wall from which to offset.
 */
void generateInfill(int layerNr, SliceMeshStorage& mesh, int innermost_wall_extrusion_width, int infill_skin_overlap, int wall_line_count);

/*!
 * \brief Combines the infill of multiple layers for a specified mesh.
 * 
 * The infill layers are combined while the thickness of each layer is
 * multiplied such that the infill should fill up again to the full height of
 * all combined layers.
 * 
 * \param mesh The mesh to combine the infill layers of.
 * \param amount The number of layers to combine.
 */
void combineInfillLayers(SliceMeshStorage& mesh, unsigned int amount);

}//namespace cura

#endif//SKIN_H
