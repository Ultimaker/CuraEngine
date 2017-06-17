/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIN_H
#define SKIN_H

#include "sliceDataStorage.h"

namespace cura 
{

/*!
 * Class containing all skin and infill area computation functions
 */
class SkinInfillAreaComputation
{
public:

    /*!
     * Initialize the parameters for skin and infill area computation.
     * 
     * \param layer_nr The index of the layer for which to generate the skins and infill.
     * \param mesh The storage where the layer outline information (input) is stored and where the skin insets and fill areas (output) are stored.
     * \param bottom_layer_count The number of layers of bottom skin
     * \param top_layer_count The number of layers of top skin
     * \param wall_line_count The number of walls, i.e. the number of the wall from which to offset.
     * \param innermost_wall_line_width width of the innermost wall lines
     * \param infill_skin_overlap overlap distance between infill and skin
     * \param wall_line_width_x The line width of the inner most wall
     * \param skin_inset_count The number of perimeters to surround the skin
     * \param no_small_gaps_heuristic A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
     * \param process_infill Whether to process infill, i.e. whether there's a positive infill density or there are infill meshes modifying this mesh.
     */
    SkinInfillAreaComputation(int layer_nr, SliceMeshStorage& mesh, int bottom_layer_count, int top_layer_count, int wall_line_count, const int innermost_wall_line_width, int infill_skin_overlap, int wall_line_width_x, int skin_inset_count, bool no_small_gaps_heuristic, bool process_infill);

    /*!
     * Generate the skin areas and its insets.
     */
    void generateSkinsAndInfill();

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
    static void combineInfillLayers(SliceMeshStorage& mesh, unsigned int amount);

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

protected:
    /*!
     * Generate the skin areas (outlines)
     */
    void generateSkinAreas();

    /*!
     * Generate the skin areas (outlines) of one part in a layer
     * 
     * \param part The part for which to generate skins.
     */
    void generateSkinAndInfillAreas(SliceLayerPart& part);

    /*!
     * Calculate the basic areas which have air above
     * 
     * \param part The part for which to compute the top skin areas
     * \param min_infill_area The minimum area to fill with skin
     * \param[in,out] upskin The areas of top skin to be pdated by the layers above.
     */
    void calculateTopSkin(const SliceLayerPart& part, int min_infill_area, Polygons& upskin);

    /*!
     * Calculate the basic areas which have air below
     * 
     * \param part The part for which to compute the bottom skin areas
     * \param min_infill_area The minimum area to fill with skin
     * \param[in,out] downskin The areas of bottom skin to be pdated by the layers above.
     */
    void calculateBottomSkin(const SliceLayerPart& part, int min_infill_area, Polygons& downskin);

    /*!
     * Apply skin expansion:
     * expand skins into infill area
     * where the skin is broad enough
     * 
     * \param original_outline The outline within which skin and infill lie (inner bounds of innermost walls)
     * \param[in,out] upskin The top skin areas to grow
     * \param[in,out] downskin The bottom skin areas to grow
     */
    void applySkinExpansion(const Polygons& original_outline, Polygons& upskin, Polygons& downskin);

    /*!
     * Generate infill of a given part
     * \param[in,out] part The part where the wall information (input) is retrieved and
     * where the infill areas (output) are stored.
     * \param skin The skin areas on the layer of the \p part
     */
    void generateInfill(SliceLayerPart& part, const Polygons& skin);

    /*!
     * Generate the skin insets.
     * 
     * \param part The part where the skin outline information (input) is stored and
     * where the skin insets (output) are stored.
     */
    void generateSkinInsets(SliceLayerPart* part);

    /*!
     * Generate the skin insets of a skin part.
     * 
     * \param skin_part The part where the skin outline information (input) is stored and
     * where the skin insets (output) are stored.
     */
    void generateSkinInsets(SkinPart& skin_part);

protected:
    const int layer_nr; //!< The index of the layer for which to generate the skins and infill.
    SliceMeshStorage& mesh; //!< The storage where the layer outline information (input) is stored and where the skin insets and fill areas (output) are stored.
    const int bottom_layer_count; //!< The number of layers of bottom skin
    const int top_layer_count; //!< The number of layers of top skin
    const int wall_line_count; //!< The number of walls, i.e. the number of the wall from which to offset.
    const int innermost_wall_line_width; //!< width of the innermost wall lines
    const int infill_skin_overlap; //!< overlap distance between infill and skin
    const int wall_line_width_x; //!< The line width of the inner most wall
    const int skin_inset_count; //!< The number of perimeters to surround the skin
    const bool no_small_gaps_heuristic; //!< A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
    const bool process_infill; //!< Whether to process infill, i.e. whether there's a positive infill density or there are infill meshes modifying this mesh.

private:
    /*!
     * Helper function to get the innermost walls of each part which might intersect with \p part_here
     * 
     * \param part_here The part for which to check
     * \param layer2 The layer from which to gather the innermost walls
     */
    Polygons getInsidePolygons(const SliceLayerPart& part_here, const SliceLayer& layer2);
};

}//namespace cura

#endif//SKIN_H
