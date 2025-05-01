// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include "SupportCradle.h"
#include "TreeModelVolumes.h"
#include "TreeSupportBaseCircle.h"
#include "TreeSupportElement.h"
#include "TreeSupportEnums.h"
#include "TreeSupportSettings.h"
#include "boost/functional/hash.hpp" // For combining hashes
#include "geometry/Polygon.h"
#include "polyclipping/clipper.hpp"
#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"
#include "utils/Coord_t.h"

namespace cura
{
using PropertyAreasUnordered = std::unordered_map<TreeSupportElement, Shape>;
using PropertyAreas = std::map<TreeSupportElement, Shape>;

struct FakeRoofArea
{
    FakeRoofArea(Shape area, coord_t line_distance, bool fractional)
        : area_(area)
        , line_distance_(line_distance)
        , fractional_(fractional)
    {
    }
    /*!
     * \brief Area that should be a fake roof.
     */
    Shape area_;

    /*!
     * \brief Distance between support lines
     */
    coord_t line_distance_;

    /*!
     * \brief If the area should be added as a fractional support area.
     */
    bool fractional_;
};


/*!
 * \brief Generates a tree structure to support your models.
 */
class TreeSupport
{
public:
    /*!
     * \brief Creates an instance of the tree support generator.
     *
     * \param storage The data storage to get global settings from.
     */
    TreeSupport(const SliceDataStorage& storage);

    /*!
     * \brief Create the areas that need support.
     *
     * These areas are stored inside the given SliceDataStorage object.
     * \param storage The data storage where the mesh data is gotten from and
     * where the resulting support areas are stored.
     */
    void generateSupportAreas(SliceDataStorage& storage);


private:
    /*!
     * \brief Precalculates all avoidances, that could be required.
     *
     * \param storage[in] Background storage to access meshes.
     * \param currently_processing_meshes[in] Indexes of all meshes that are processed in this iteration
     * \param top_most_cradle_layer_idx[in] Layer_idx of the top-most cradle.
     * \return Uppermost layer precalculated. -1 if no layer were precalculated as no overhang is present.
     */
    LayerIndex precalculate(const SliceDataStorage& storage, std::vector<size_t> currently_processing_meshes, LayerIndex top_most_cradle_layer_idx);


    /*!
     * \brief Creates the initial influence areas (that can later be propagated down) by placing them below the overhang.
     *
     * Generates Points where the Model should be supported and creates the areas where these points have to be placed.
     *
     * \param mesh[in] The mesh that is currently processed.
     * \param move_bounds[out] Storage for the influence areas.
     * \param storage[in] Background storage, required for adding roofs.
     * \param cradle_data_model[out] All generated cradles, with its corresponding cradle lines.

     */
    void generateInitialAreas(
        const SliceMeshStorage& mesh,
        std::vector<std::set<TreeSupportElement*>>& move_bounds,
        SliceDataStorage& storage,
        std::vector<std::vector<SupportCradle*>>& cradle_data_model);


    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This helper merges all elements in input with the elements into reduced_new_layer.
     * Elements in input_aabb are merged together if possible, while elements reduced_new_layer_aabb are not checked against each other.
     *
     * \param reduced_aabb[in,out] The already processed elements.
     * \param input_aabb[in] Not yet processed elements
     * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate. Value is the influence area where the center of a circle of support may be placed.
     * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is
     * not forced to. Value is the influence area where the center of a circle of support may be placed. \param influence_areas[in] The influence areas without avoidance removed.
     * \param insert_bp_areas[out] Elements to be inserted into the main dictionary after the Helper terminates.
     * \param insert_model_areas[out] Elements to be inserted into the secondary dictionary after the Helper terminates.
     * \param insert_influence[out] Elements to be inserted into the dictionary containing the largest possibly valid influence area (ignoring if the area may not be there because
     * of avoidance) \param erase[out] Elements that should be deleted from the above dictionaries. \param layer_idx[in] The Index of the current Layer.
     */
    void mergeHelper(
        std::map<TreeSupportElement, AABB>& reduced_aabb,
        std::map<TreeSupportElement, AABB>& input_aabb,
        const PropertyAreasUnordered& to_bp_areas,
        const PropertyAreas& to_model_areas,
        const PropertyAreas& influence_areas,
        PropertyAreasUnordered& insert_bp_areas,
        PropertyAreasUnordered& insert_model_areas,
        PropertyAreasUnordered& insert_influence,
        std::vector<TreeSupportElement>& erase,
        const LayerIndex layer_idx);

    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This manages the helper and uses a divide and conquer approach to parallelize this problem.
     * This parallelization can at most accelerate the merging by a factor of 2.
     *
     * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is
     * not forced to. Value is the influence area where the center of a circle of support may be placed.
     * \param influence_areas[in] The Elements of the current Layer without
     * avoidances removed. This is the largest possible influence area for this layer. Value is the influence area where the center of a circle of support may be placed.
     * \param layer_idx[in] The current layer.
     */
    void mergeInfluenceAreas(PropertyAreasUnordered& to_bp_areas, PropertyAreas& to_model_areas, PropertyAreas& influence_areas, LayerIndex layer_idx);

    /*!
     * \brief Checks if an influence area contains a valid subsection and returns the corresponding metadata and the new Influence area.
     *
     * Calculates an influence areas of the layer below, based on the influence area of one element on the current layer.
     * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in we would change our gracious or to_buildplate status the influence areas are
     * instead increased by maximum_move_distance_slow. Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this
     * were the case, the radius is not increased instead.
     *
     * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was
     * done to avoid not needed heap allocations.
     *
     * \param settings[in] Which settings have to be used to check validity.
     * \param layer_idx[in] Number of the current layer.
     * \param parent[in] The metadata of the parents influence area.
     * \param relevant_offset[in] The maximal possible influence area. No guarantee regarding validity with current layer collision required, as it is ensured in-function!
     * \param to_bp_data[out] The part of the Influence area that can reach the buildplate.
     * \param to_model_data[out] The part of the Influence area that do not have to reach the buildplate. This has overlap with new_layer_data.
     * \param increased[out]  Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the
     * user-supplied settings. \param overspeed[in] How much should the already offset area be offset again. Usually this is 0. \param mergelayer[in] Will the merge method be
     * called on this layer. This information is required as some calculation can be avoided if they are not required for merging. \return A valid support element for the next
     * layer regarding the calculated influence areas. Empty if no influence are can be created using the supplied influence area and settings.
     */
    std::optional<TreeSupportElement> increaseSingleArea(
        AreaIncreaseSettings settings,
        LayerIndex layer_idx,
        TreeSupportElement* parent,
        const Shape& relevant_offset,
        Shape& to_bp_data,
        Shape& to_model_data,
        Shape& increased,
        const coord_t overspeed,
        const bool mergelayer);

    /*!
     * \brief Increases influence areas as far as required.
     *
     * Calculates influence areas of the layer below, based on the influence areas of the current layer.
     * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in it would change the gracious or to_buildplate status, the influence areas are
     * instead increased by maximum_move_distance. Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were
     * the case, the radius is not increased instead.
     *
     * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was
     * done to avoid not needed heap allocations.
     *
     * \param to_bp_areas[out] Influence areas that can reach the buildplate
     * \param to_model_areas[out] Influence areas that do not have to reach the buildplate. This has overlap with new_layer_data, as areas that can reach the buildplate are also
     * considered valid areas to the model. This redundancy is required if a to_buildplate influence area is allowed to merge with a to model influence area.
     * \param influence_areas[out] Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the
     * user-supplied settings.
     * \param bypass_merge_areas[out] Influence areas ready to be added to the layer below that do not need merging.
     * \param last_layer[in] Influence areas of the current layer.
     * \param layer_idx[in] Number of the current layer.
     * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
     */
    void increaseAreas(
        PropertyAreasUnordered& to_bp_areas,
        PropertyAreas& to_model_areas,
        PropertyAreas& influence_areas,
        PropertyAreas& bypass_merge_areas,
        const std::vector<TreeSupportElement*>& last_layer,
        const LayerIndex layer_idx,
        const bool mergelayer);

    /*!
     * \brief Evaluate which cradle lines will have to be removed, remove them and remove tips that support said lines
     *

     * \param to_bp_areas[in,out] The Elements of the current Layer that will reach the buildplate.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param to_model_areas[in,out] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate
     is
     * not forced to. Value is the influence area where the center of a circle of support may be placed.
     * \param influence_areas[in,out] The Elements of the current Layer without
     * avoidances removed. This is the largest possible influence area for this layer. Value is the influence area where the center of a circle of support may be placed.
     * \param layer_idx[in] The current layer.
     * \param move_bounds[in,out] All currently existing influence areas
     * \param cradle_data[in] Information about all cradle lines on this layer.
     */
    void handleCradleLineValidity(
        PropertyAreasUnordered& to_bp_areas,
        PropertyAreas& to_model_areas,
        PropertyAreas& influence_areas,
        PropertyAreas& bypass_merge_areas,
        LayerIndex layer_idx,
        std::vector<std::set<TreeSupportElement*>>& move_bounds,
        std::vector<std::vector<CradlePresenceInformation>>& cradle_data);

    /*!
     * \brief Propagates influence downwards, and merges overlapping ones.
     *
     * \param move_bounds[in,out] All currently existing influence areas
     * \param cradle_data[in,out] All currently existing cradles, with its corresponding cradle lines.
     */
    void createLayerPathing(std::vector<std::set<TreeSupportElement*>>& move_bounds, std::vector<std::vector<SupportCradle*>>& cradle_data);


    /*!
     * \brief Sets the result_on_layer for all parents based on the SupportElement supplied.
     *
     * \param elem[in] The SupportElements, which parent's position should be determined.
     */
    void setPointsOnAreas(const TreeSupportElement* elem);

    /*!
     * \brief Get the best point to connect to the model and set the result_on_layer of the relevant SupportElement accordingly.
     *
     * \param move_bounds[in,out] All currently existing influence areas
     * \param first_elem[in,out] SupportElement that did not have its result_on_layer set meaning that it does not have a child element.
     * \param layer_idx[in] The current layer.
     * \return Should elem be deleted.
     */
    bool setToModelContact(std::vector<std::set<TreeSupportElement*>>& move_bounds, TreeSupportElement* first_elem, const LayerIndex layer_idx);

    /*!
     * \brief Set the result_on_layer point for all influence areas
     *
     * \param move_bounds[in,out] All currently existing influence areas
     */
    void createNodesFromArea(std::vector<std::set<TreeSupportElement*>>& move_bounds);

    /*!
     * \brief Draws circles around result_on_layer points of the influence areas
     *
     * \param linear_data[in] All currently existing influence areas with the layer they are on
     * \param layer_tree_polygons[out] Resulting branch areas with the layerindex they appear on.
     *    layer_tree_polygons.size() has to be at least linear_data.size() as each Influence area in linear_data will save have at least one (that's why it's a vector<vector>)
     * corresponding branch area in layer_tree_polygons. \param inverse_tree_order[in] A mapping that returns the child of every influence area.
     */
    void generateBranchAreas(
        std::vector<std::pair<LayerIndex, TreeSupportElement*>>& linear_data,
        std::vector<std::unordered_map<TreeSupportElement*, Shape>>& layer_tree_polygons,
        const std::map<TreeSupportElement*, TreeSupportElement*>& inverse_tree_order);

    /*!
     * \brief Applies some smoothing to the outer wall, intended to smooth out sudden jumps as they can happen when a branch moves though a hole.
     *
     * \param layer_tree_polygons[in,out] Resulting branch areas with the layerindex they appear on.
     */
    void smoothBranchAreas(std::vector<std::unordered_map<TreeSupportElement*, Shape>>& layer_tree_polygons);

    /*!
     * \brief Drop down areas that do rest non-gracefully on the model to ensure the branch actually rests on something.
     *
     * \param layer_tree_polygons[in] Resulting branch areas with the layerindex they appear on.
     * \param linear_data[in] All currently existing influence areas with the layer they are on
     * \param dropped_down_areas[out] Areas that have to be added to support all non-graceful areas.
     * \param inverse_tree_order[in] A mapping that returns the child of every influence area.
     */
    void dropNonGraciousAreas(
        std::vector<std::unordered_map<TreeSupportElement*, Shape>>& layer_tree_polygons,
        const std::vector<std::pair<LayerIndex, TreeSupportElement*>>& linear_data,
        std::vector<std::vector<std::pair<LayerIndex, Shape>>>& dropped_down_areas,
        const std::map<TreeSupportElement*, TreeSupportElement*>& inverse_tree_order);


     /*!
     * \brief Accumulate areas needed later, union all import and add all roof to storage.
     * \param support_layer_storage[in,out] Areas where support should be generated.
     * \param support_layer_storage_fractional[in,out] Areas where support has to be, projected up for fractional height.
     * \param support_roof_storage[in] Areas where support was replaced with roof.
     * \param support_roof_extra_wall_storage[in] Areas where support was replaced with roof, but roofs need to have a wall to print correctly.
     * \param support_roof_storage_fractional[in] Areas of roof that were projected one layer up.
     * \param support_roof_extra_wall_storage_fractional[in] Areas of roof that were projected one layer up, but roofs need to have a wall to print correctly.
     * \param fake_roof_areas_combined[out] All areas that contain the fake roofs.
     * \param cradle_base_areas[out] Copy of all cradle base areas. Already added to correct storage.
     * \param cradle_support_line_areas[out] All cradle lines consisting of regular support. Already added as support.
     * \param storage[in,out] The storage where the support should be stored.
     * \param cradle_data[in] All currently existing cradles, with its corresponding cradle lines.
     */
    void prepareSupportAreas(
        std::vector<Shape>& support_layer_storage,
        std::vector<Shape>& support_layer_storage_fractional,
        std::vector<Shape>& support_roof_storage,
        std::vector<Shape>& support_roof_extra_wall_storage,
        std::vector<Shape>& support_roof_storage_fractional,
        std::vector<Shape>& support_roof_extra_wall_storage_fractional,
        std::vector<Shape>& fake_roof_areas_combined,
        std::vector<Shape>& cradle_base_areas,
        std::vector<Shape>& cradle_support_line_areas,
        SliceDataStorage& storage,
        std::vector<std::vector<SupportCradle*>>& cradle_data);

    /*!
     * \brief Calculates which holes are valid (rest on walls) and which holes rest on which other holes
     * \param support_layer_storage[in] Areas where support should be generated.
     * \param hole_parts[out] Parts of holes, ordered by layer.
     * \param valid_holes[out] Indices of holes that rest on outer wall, by layer.
     * \param non_removable_holes[out] Indices of holes that can not be removed, by layer.
     * \param hole_rest_map[out] Ordered by layer, information on which hole index on the layer below a given hole rests on
     */
    void calculateSupportHoles(std::vector<Shape>& support_layer_storage,
                               std::vector<std::vector<Shape>>& hole_parts,
                               std::vector<std::set<size_t>>& valid_holes,
                               std::vector<std::set<size_t>>& non_removable_holes,
                               std::vector<std::map<size_t, std::vector<size_t>>>& hole_rest_map);

    /*!
     * \brief Generates support areas with high density infill to support interface above. Has to be called even if no support skin will generate,
     * as cradle lines are added to the support_layer_storage here.

     * \param support_layer_storage[in,out] Areas where support should be generated.
     * \param support_skin_storage[out] Areas where high density support should be generated.
     * \param fake_roof_areas_combined[in] All areas that contain the fake roofs.
     * \param cradle_base_areas[in] Copy of all cradle base areas. Already added to correct storage.
     * \param cradle_support_line_areas[in] All cradle lines consisting of regular support. Already added as support.
     * \param hole_parts[in] Parts of holes, ordered by layer.
     * \param valid_holes[in] Indices of holes that rest on outer wall, by layer.
     * \param non_removable_holes[in] Indices of holes that can not be removed, by layer.
     * \param hole_rest_map[in] Ordered by layer, information on which hole index on the layer below a given hole rests on
     * \param storage[in] The storage where the support should be stored.
     * \param layer_tree_polygons[in] Resulting branch areas with the layerindex they appear on.
     */
    void generateSupportSkin(
        std::vector<Shape>& support_layer_storage,
        std::vector<Shape>& support_skin_storage,
        std::vector<Shape>& fake_roof_areas_combined,
        std::vector<Shape>& cradle_base_areas,
        std::vector<Shape>& cradle_support_line_areas,
        std::vector<std::vector<Shape>>& hole_parts,
        std::vector<std::set<size_t>>& valid_holes,
        std::vector<std::set<size_t>>& non_removable_holes,
        std::vector<std::map<size_t, std::vector<size_t>>>& hole_rest_map,
        SliceDataStorage& storage,
        std::vector<std::unordered_map<TreeSupportElement*, Shape>>& layer_tree_polygons);

    /*!
     * \brief Filters out holes that would cause support to be printed mid-air.
     * \param support_layer_storage[in,out] Areas where support should be generated.
     * \param hole_parts[in] Parts of holes, ordered by layer.
     * \param valid_holes[in] Indices of holes that rest on outer wall, by layer.
     * \param non_removable_holes[in] Indices of holes that can not be removed, by layer.
     * \param hole_rest_map[in] Ordered by layer, information on which hole index on the layer below a given hole rests on
     */
    void removeFloatingHoles(std::vector<Shape>& support_layer_storage,
                             std::vector<std::vector<Shape>>& hole_parts,
                             std::vector<std::set<size_t>>& valid_holes,
                             std::vector<std::set<size_t>>& non_removable_holes,
                             std::vector<std::map<size_t, std::vector<size_t>>>& hole_rest_map);

    /*!
     * \brief Generates Support Floor, ensures Support Roof can not cut of branches, and saves the branches as support to storage
     *
     * \param support_layer_storage[in] Areas where support should be generated.
     * \param support_skin_storage[in] Areas where high density support should be generated.
     * \param support_layer_storage_fractional[out] Areas where support has to be, projected up for fractional height.
     * \param storage[in,out] The storage where the support should be stored.
     */
    void finalizeInterfaceAndSupportAreas(
        std::vector<Shape>& support_layer_storage,
        std::vector<Shape>& support_skin_storage,
        std::vector<Shape>& support_layer_storage_fractional,
        SliceDataStorage& storage);

    /*!
     * \brief Draws circles around result_on_layer points of the influence areas and applies some post processing.
     *
     * \param move_bounds[in] All currently existing influence areas
     * \param storage[in,out] The storage where the support should be stored.
     * \param cradle_data[in] All currently existing cradles, with its corresponding cradle lines.
     */
    void drawAreas(std::vector<std::set<TreeSupportElement*>>& move_bounds, SliceDataStorage& storage, std::vector<std::vector<SupportCradle*>>& cradle_data);

    /*!
     * \brief Settings with the indexes of meshes that use these settings.
     */
    std::vector<std::pair<TreeSupportSettings, std::vector<size_t>>> grouped_meshes;

    /*!
     * \brief Areas that use a higher density pattern of regular support to support the model (fake_roof).
     */
    std::vector<std::vector<FakeRoofArea>> fake_roof_areas;

    /*!
     * \brief Areas where no support may be. Areas will be subtracted from support areas.
     */
    std::vector<Shape> support_free_areas;

    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes.
     */
    TreeModelVolumes volumes_;

    /*!
     * \brief Contains config settings to avoid loading them in every function. This was done to improve readability of the code.
     */
    TreeSupportSettings config;

    /*!
     * \brief The progress multiplier of all values added progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_multiplier = 1;

    /*!
     * \brief The progress offset added to all values communicated to the progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_offset = 0;
};


} // namespace cura


#endif /* TREESUPPORT_H */
