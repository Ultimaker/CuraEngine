/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIN_H
#define SKIN_H

#include "sliceDataStorage.h"

namespace cura 
{
/*!
 * Generate the skin areas and its insets.
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param mesh The storage where the layer outline information (input) is stored and where the skin insets and fill areas (output) are stored.
 * \param downSkinCount The number of layers of bottom skin
 * \param upSkinCount The number of layers of top skin
 * \param wall_line_count The number of walls, i.e. the number of the wall from which to offset.
 * \param wall_line_width_x The line width of the inner most wall
 * \param insetCount The number of perimeters to surround the skin
 * \param no_small_gaps_heuristic A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
 */
void generateSkins(int layerNr, SliceMeshStorage& mesh, int downSkinCount, int upSkinCount, int wall_line_count, int wall_line_width_x, int insetCount, bool no_small_gaps_heuristic);

/*!
 * Generate the skin areas (outlines)
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param mesh The storage where the layer outline information (input) is stored
 * and where the skin outline (output) is stored.
 * \param innermost_wall_line_width The line width of the walls around the skin, by which
 * we must inset for each wall.
 * \param downSkinCount The number of layers of bottom skin.
 * \param upSkinCount The number of layers of top skin.
 * \param wall_line_count The number of walls, i.e. the number of the wall from
 * which to offset.
 * \param no_small_gaps_heuristic A heuristic which assumes there will be no
 * small gaps between bottom and top skin with a z size smaller than the skin
 * size itself.
 */
void generateSkinAreas(int layerNr, SliceMeshStorage& mesh, const int innermost_wall_line_width, int downSkinCount, int upSkinCount, int wall_line_count, bool no_small_gaps_heuristic);

/*!
 * Generate the skin insets.
 * 
 * \param layerNr The index of the layer for which to generate the skins.
 * \param part The part where the skin outline information (input) is stored and
 * where the skin insets (output) are stored.
 * \param wall_line_width The width of the perimeters around the skin.
 * \param insetCount The number of perimeters to surround the skin.
 */
void generateSkinInsets(SliceLayerPart* part, const int wall_line_width, int insetCount);

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
 * \param innermost_wall_line_width width of the innermost wall lines
 * \param infill_skin_overlap overlap distance between infill and skin
 * \param wall_line_count The number of walls, i.e. the number of the wall from which to offset.
 */
void generateInfill(int layerNr, SliceMeshStorage& mesh, const int innermost_wall_line_width, int infill_skin_overlap, int wall_line_count);

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

/*!
 * Class containing all skin and infill area computation functions
 */
class SkinInfillAreaComputation
{
public:
    /*!
     * Generate infill areas which cause a gradually less dense infill structure from top to bottom.
     * 
     * The areas generated overlap, so that more dense infill adds on to less dense infill.
     * That way you don't have infill lines which are broken when they cross a border between separated infill areas - if they would be as such.
     * 
     * This function also guarantees that the SliceLayerPart::infill_area_per_combine_per_density is initialized with at least one item.
     * The last item in the list will be equal to the infill_area after this function.
     * 
     * \param gradual_infill_step_height // The height difference between consecutive density infill areas
     * \param max_infill_steps the maximum exponent of division of infill density. At 5 the least dense infill will be 2^4 * infill_line_distance i.e. one 16th as dense
     */
    static void generateGradualInfill(SliceMeshStorage& mesh, unsigned int gradual_infill_step_height, unsigned int max_infill_steps);
    
};

}//namespace cura

#endif//SKIN_H
