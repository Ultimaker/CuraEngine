//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SKIN_H
#define SKIN_H

#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"

namespace cura 
{

class Polygons;
class SkinPart;
class SliceLayerPart;
class SliceMeshStorage;

/*!
 * Class containing all skin and infill area computation functions
 */
class SkinInfillAreaComputation
{
public:

    /*!
     * \brief Initialize the parameters for skin and infill area computation.
     * 
     * \param layer_nr The index of the layer for which to generate the skins
     * and infill.
     * \param mesh The storage where the layer outline information (input) is
     * stored and where the skin insets and fill areas (output) are stored.
     * \param process_infill Whether to process infill, i.e. whether there's a
     * positive infill density or there are infill meshes modifying this mesh.
     */
    SkinInfillAreaComputation(const LayerIndex& layer_nr, SliceMeshStorage& mesh, bool process_infill);

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
     */
    static void combineInfillLayers(SliceMeshStorage& mesh);

    /*!
     * \brief Generate infill areas which cause a gradually less dense infill
     * structure from top to bottom.
     *
     * The areas generated overlap, so that more dense infill adds on to less
     * dense infill. That way you don't have infill lines which are broken when
     * they cross a border between separated infill areas, if they would be as
     * such.
     *
     * This function also guarantees that the
     * SliceLayerPart::infill_area_per_combine_per_density is initialised with
     * at least one item. The last item in the list will be equal to the
     * infill_area after this function.
     * \param mesh The mesh to generate the infill areas for.
     */
    static void generateGradualInfill(SliceMeshStorage& mesh);

    /*!
     * Limit the infill areas to places where they support internal overhangs.
     * 
     * This function uses the part.infill_area and part.infill_area_own
     * and computes a new part.infill_area_own
     * 
     * \param mesh The mesh for which to recalculate the infill areas
     */
    static void generateInfillSupport(SliceMeshStorage& mesh);
protected:
    /*!
     * Generate the skin areas (outlines) and the infill areas
     */
    void generateSkinAndInfillAreas();

    /*!
     * Generate the skin areas (outlines) of one part in a layer
     * 
     * \param part The part for which to generate skins.
     */
    void generateSkinAndInfillAreas(SliceLayerPart& part);

    /*!
     * \brief Calculate the basic areas which have air above.
     * \param part The part for which to compute the top skin areas.
     * \param[in,out] upskin The areas of top skin to be updated by the layers
     * above. The input is the area within the inner walls (or an empty Polygons
     * object).
     */
    void calculateTopSkin(const SliceLayerPart& part, Polygons& upskin);

    /*!
     * \brief Calculate the basic areas which have air below.
     * \param part The part for which to compute the bottom skin areas.
     * \param[in,out] downskin The areas of bottom skin to be updated by the
     * layers above. The input is the area within the inner walls (or an empty
     * Polygons object).
     */
    void calculateBottomSkin(const SliceLayerPart& part, Polygons& downskin);

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
     * Calculate the areas which are 'directly' under air,
     * remove them from the \ref SkinPart::inner_infill and save them in the \ref SkinPart::roofing_fill of the \p part
     * 
     * \param[in,out] part Where to get the sSkinParts to get the outline info from and to store the roofing areas
     */
    void generateRoofing(SliceLayerPart& part);

    /*!
     * Generate the skin insets and the inner infill area
     * 
     * \param part The part where the skin outline information (input) is stored and
     * where the skin insets (output) are stored.
     */
    void generateSkinInsetsAndInnerSkinInfill(SliceLayerPart* part);

    /*!
     * Generate the skin insets of a skin part
     * 
     * \param skin_part The part where the skin outline information (input) is stored and
     * where the skin insets (output) are stored.
     */
    void generateSkinInsets(SkinPart& skin_part);

    /*!
     * Generate the inner_infill_area of a skin part
     * 
     * \param skin_part The part where the skin outline information (input) is stored and
     * where the inner infill area (output) is stored.
     */
    void generateInnerSkinInfill(SkinPart& skin_part);

protected:
    const LayerIndex layer_nr; //!< The index of the layer for which to generate the skins and infill.
    SliceMeshStorage& mesh; //!< The storage where the layer outline information (input) is stored and where the skin insets and fill areas (output) are stored.
    const size_t bottom_layer_count; //!< The number of layers of bottom skin
    const size_t initial_bottom_layer_count; //!< Whether to make bottom skin for the initial layer
    const size_t top_layer_count; //!< The number of layers of top skin
    const size_t wall_line_count; //!< The number of walls, i.e. the number of the wall from which to offset.
    const coord_t skin_line_width; //!< The line width of the skin.
    const coord_t wall_line_width_0; //!< The line width of the outer wall
    const coord_t wall_line_width_x; //!< The line width of the inner most wall
    const coord_t innermost_wall_line_width; //!< width of the innermost wall lines
    const coord_t infill_skin_overlap; //!< overlap distance between infill and skin
    const size_t skin_inset_count; //!< The number of perimeters to surround the skin
    const bool no_small_gaps_heuristic; //!< A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
    const bool process_infill; //!< Whether to process infill, i.e. whether there's a positive infill density or there are infill meshes modifying this mesh.

    coord_t top_reference_wall_expansion; //!< The horizontal expansion to apply to the top reference wall in order to shrink the top skin
    coord_t bottom_reference_wall_expansion; //!< The horizontal expansion to apply to the bottom reference wall in order to shrink the bottom skin
    coord_t top_skin_expand_distance; //!< The distance by which the top skins should be larger than the original top skins.
    coord_t bottom_skin_expand_distance; //!< The distance by which the bottom skins should be larger than the original bottom skins.
    const size_t top_reference_wall_idx; //!< The wall of the layer above to consider as inside. Lower index means more skin.
    const size_t bottom_reference_wall_idx; //!< The wall of the layer below to consider as inside. Lower index means more skin.
private:
    static coord_t getSkinLineWidth(const SliceMeshStorage& mesh, const LayerIndex& layer_nr); //!< Compute the skin line width, which might be different for the first layer.
    static coord_t getWallLineWidth0(const SliceMeshStorage& mesh, const LayerIndex& layer_nr); //!< Compute the outer wall line width, which might be different for the first layer
    static coord_t getWallLineWidthX(const SliceMeshStorage& mesh, const LayerIndex& layer_nr); //!< Compute the inner wall line widths, which might be different for the first layer
    static coord_t getInfillSkinOverlap(const SliceMeshStorage& mesh, const LayerIndex& layer_nr, const coord_t& innermost_wall_line_width); //!< Compute the infill_skin_overlap

    /*!
     * Helper function to get the walls of each part which might intersect with \p part_here
     * 
     * \param part_here The part for which to check
     * \param layer2_nr The layer index from which to gather the outlines
     * \param wall_idx The 1-based wall index for the walls to grab. e.g. the outermost walls or the second walls. Zero means the outline.
     */
    Polygons getWalls(const SliceLayerPart& part_here, int layer2_nr, unsigned int wall_idx);

    /*!
     * Get the wall index of the reference wall for either the top or bottom skin.
     * With larger user specified preshrink come lower reference wall indices.
     * 
     * The \p preshrink is updated to be relative to be the offset from the resulting reference wall.
     * A preshrink distance close to an existing wall will snap to that wall so that no offset has to be computed.
     * 
     * \param[in,out] preshrink The expansion to be applied to the reference wall. The input is the expansion to be applied to the innermost wall, the output is the expansion applied to the returned reference wall.
     * \return The index of the reference wall to view as being inside the model for the skin area computation.
     */
    int getReferenceWallIdx(coord_t& preshrink) const;

};

}//namespace cura

#endif//SKIN_H
