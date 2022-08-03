//Copyright (c) 2021 Ultimaker B.V.
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
     * Remove the areas which are 'directly' under air from the \ref SkinPart::inner_infill and 
     * save them in the \ref SkinPart::roofing_fill of the \p part.
     * 
     * \param[in,out] part Where to get the SkinParts to get the outline info from and to store the roofing areas
     */
    void generateRoofing(SliceLayerPart& part);

    /*!
     * Remove the areas which are directly under air in the top-most surface and directly above air in bottom-most
     * surfaces from the \ref SkinPart::inner_infill and save them in the \ref SkinPart::bottom_most_surface_fill and
     * \ref SkinPart::top_most_surface_fill (respectively) of the \p part.
     *
     * \param[in,out] part Where to get the SkinParts to get the outline info from and to store the top and bottom-most
     * infill areas
     */
    void generateTopAndBottomMostSkinSurfaces(SliceLayerPart& part);

    /*!
     * Helper function to calculate and return the areas which are 'directly' under air.
     *
     * \param part Where to get the SkinParts to get the outline info from
     * \param roofing_layer_count The number of layers above the layer which we are looking into
     */
    Polygons generateNoAirAbove(SliceLayerPart& part, size_t roofing_layer_count);

    /*!
     * Helper function to calculate and return the areas which are 'directly' above air.
     *
     * \param part Where to get the SkinParts to get the outline info from
     * \param flooring_layer_count The number of layers below the layer which we are looking into
     */
    Polygons generateNoAirBelow(SliceLayerPart& part, size_t flooring_layer_count);

    /*!
     * Helper function to recalculate the roofing fill and inner infill in roofing layers where the 
     * insets have to be changed.
     *
     * \param part Where to get the SkinParts to get the outline info from
     * \param skin_part The part where the skin outline information (input) is stored and
     * where the inner infill and roofing infill areas (output) is stored.
     */
    void regenerateRoofingFillAndInnerInfill(SliceLayerPart& part, SkinPart& skin_part);

protected:
    LayerIndex layer_nr; //!< The index of the layer for which to generate the skins and infill.
    SliceMeshStorage& mesh; //!< The storage where the layer outline information (input) is stored and where the skin insets and fill areas (output) are stored.
    size_t bottom_layer_count; //!< The number of layers of bottom skin
    size_t initial_bottom_layer_count; //!< Whether to make bottom skin for the initial layer
    size_t top_layer_count; //!< The number of layers of top skin
    size_t wall_line_count; //!< The number of walls, i.e. the number of the wall from which to offset.
    coord_t skin_line_width; //!< The line width of the skin.
    size_t skin_inset_count; //!< The number of perimeters to surround the skin
    bool no_small_gaps_heuristic; //!< A heuristic which assumes there will be no small gaps between bottom and top skin with a z size smaller than the skin size itself
    bool process_infill; //!< Whether to process infill, i.e. whether there's a positive infill density or there are infill meshes modifying this mesh.

    coord_t top_skin_preshrink; //!< The top skin removal width, to remove thin strips of skin along nearly-vertical walls.
    coord_t bottom_skin_preshrink; //!< The bottom skin removal width, to remove thin strips of skin along nearly-vertical walls.
    coord_t top_skin_expand_distance; //!< The distance by which the top skins should be larger than the original top skins.
    coord_t bottom_skin_expand_distance; //!< The distance by which the bottom skins should be larger than the original bottom skins.
private:
    static coord_t getSkinLineWidth(const SliceMeshStorage& mesh, const LayerIndex& layer_nr); //!< Compute the skin line width, which might be different for the first layer.

    /*!
     * Helper function to get the outline of each part which might intersect
     * with \p part_here in a different layer.
     *
     * This is used to determine where the air is on a different layer. Inner
     * area that is close to air on a different layer (below air for top skin,
     * above air for bottom skin) will be come skin. The rest will become
     * infill.
     * \param part_here The part for which to check.
     * \param layer2_nr The layer index from which to gather the outlines.
     */
    Polygons getOutlineOnLayer(const SliceLayerPart& part_here, const LayerIndex layer2_nr);
};

}//namespace cura

#endif//SKIN_H
