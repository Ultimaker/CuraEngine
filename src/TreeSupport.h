// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include "TreeModelVolumes.h"
#include "TreeSupportBaseCircle.h"
#include "TreeSupportElement.h"
#include "TreeSupportEnums.h"
#include "TreeSupportSettings.h"
#include "boost/functional/hash.hpp" // For combining hashes
#include "polyclipping/clipper.hpp"
#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"
#include "utils/polygon.h"

// The various stages of the process can be weighted differently in the progress bar.
// These weights are obtained experimentally using a small sample size. Sensible weights can differ drastically based on the assumed default settings and model.
#define TREE_PROGRESS_TOTAL 10000
#define TREE_PROGRESS_PRECALC_COLL TREE_PROGRESS_TOTAL * 0.1
#define TREE_PROGRESS_PRECALC_AVO TREE_PROGRESS_TOTAL * 0.4
#define TREE_PROGRESS_GENERATE_NODES TREE_PROGRESS_TOTAL * 0.1
#define TREE_PROGRESS_AREA_CALC TREE_PROGRESS_TOTAL * 0.3
#define TREE_PROGRESS_DRAW_AREAS TREE_PROGRESS_TOTAL * 0.1

#define TREE_PROGRESS_GENERATE_BRANCH_AREAS TREE_PROGRESS_DRAW_AREAS / 3
#define TREE_PROGRESS_SMOOTH_BRANCH_AREAS TREE_PROGRESS_DRAW_AREAS / 3
#define TREE_PROGRESS_FINALIZE_BRANCH_AREAS TREE_PROGRESS_DRAW_AREAS / 3

#define SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL false
#define SUPPORT_TREE_AVOID_SUPPORT_BLOCKER true
#define SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION true
#define SUPPORT_TREE_EXPONENTIAL_THRESHOLD 1000
#define SUPPORT_TREE_EXPONENTIAL_FACTOR 1.5
#define SUPPORT_TREE_PRE_EXPONENTIAL_STEPS 1
#define SUPPORT_TREE_COLLISION_RESOLUTION 500 // Only has an effect if SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION is false

#define SUPPORT_TREE_MAX_DEVIATION 0

namespace cura
{


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
    enum class LineStatus
    {
        INVALID,
        TO_MODEL,
        TO_MODEL_GRACIOUS,
        TO_MODEL_GRACIOUS_SAFE,
        TO_BP,
        TO_BP_SAFE
    };

    using LineInformation = std::vector<std::pair<Point, TreeSupport::LineStatus>>;


    /*!
     * \brief Precalculates all avoidances, that could be required.
     *
     * \param storage[in] Background storage to access meshes.
     * \param currently_processing_meshes[in] Indexes of all meshes that are processed in this iteration
     */
    void precalculate(const SliceDataStorage& storage, std::vector<size_t> currently_processing_meshes);
    /*!
     * \brief Converts a Polygons object representing a line into the internal format.
     *
     * \param polylines[in] The Polyline that will be converted.
     * \param layer_idx[in] The current layer.
     * \return All lines of the \p polylines object, with information for each point regarding in which avoidance it is currently valid in.
     */
    std::vector<LineInformation> convertLinesToInternal(Polygons polylines, LayerIndex layer_idx);
    /*!
     * \brief Converts lines in internal format into a Polygons object representing these lines.
     *
     * \param lines[in] The lines that will be converted.
     * \return All lines of the \p lines object as a Polygons object.
     */
    Polygons convertInternalToLines(std::vector<TreeSupport::LineInformation> lines);
    /*!
     * \brief Returns a function, evaluating if a point has to be added now. Required for a splitLines call in generateInitialAreas.
     *
     * \param current_layer[in] The layer on which the point lies
     * \return A function that can be called to evaluate a point.
     */
    std::function<bool(std::pair<Point, TreeSupport::LineStatus>)> getEvaluatePointForNextLayerFunction(size_t current_layer);
    /*!
     * \brief Evaluates which points of some lines are not valid one layer below and which are. Assumes all points are valid on the current layer. Validity is evaluated using supplied lambda.
     *
     * \param lines[in] The lines that have to be evaluated.
     * \param evaluatePoint[in] The function used to evaluate the points.
     * \return A pair with which points are still valid in the first slot and which are not in the second slot.
     */
    std::pair<std::vector<LineInformation>, std::vector<LineInformation>> splitLines(std::vector<LineInformation> lines, std::function<bool(std::pair<Point, TreeSupport::LineStatus>)> evaluatePoint); // assumes all Points on the current line are valid

    /*!
     * \brief Ensures that every line segment is about distance in length. The resulting lines may differ from the original but all points are on the original
     *
     * \param input[in] The lines on which evenly spaced points should be placed.
     * \param distance[in] The distance the points should be from each other.
     * \param min_points[in] The amount of points that have to be placed. If not enough can be placed the distance will be reduced to place this many points.
     * \return A Polygons object containing the evenly spaced points. Does not represent an area, more a collection of points on lines.
     */
    Polygons ensureMaximumDistancePolyline(const Polygons& input, coord_t distance, size_t min_points) const;

    /*!
     * \brief Adds the implicit line from the last vertex of a Polygon to the first one.
     *
     * \param poly[in] The Polygons object, of which its lines should be extended.
     * \return A Polygons object with implicit line from the last vertex of a Polygon to the first one added.
     */
    Polygons toPolylines(const Polygons& poly) const;


    /*!
     * \brief Converts toolpaths to a Polygons object, containing the implicit line from first to last vertex
     *
     * \param toolpaths[in] The toolpaths.
     * \return A Polygons object with implicit line from the last vertex of a Polygon to the first one added.
     */
    Polygons toPolylines(const std::vector<VariableWidthLines> toolpaths) const;

    /*!
     * \brief Returns Polylines representing the (infill) lines that will result in slicing the given area
     *
     * \param area[in] The area that has to be filled with infill.
     * \param roof[in] Whether the roofing or regular support settings should be used.
     * \param layer_idx[in] The current layer index.
     * \param support_infill_distance[in] The distance that should be between the infill lines.
     * \param cross_fill_provider[in] A SierpinskiFillProvider required for cross infill.
     *
     * \return A Polygons object that represents the resulting infill lines.
     */
    Polygons generateSupportInfillLines(const Polygons& area, bool roof, LayerIndex layer_idx, coord_t support_infill_distance, SierpinskiFillProvider* cross_fill_provider = nullptr, bool include_walls = false);

    /*!
     * \brief Unions two Polygons. Ensures that if the input is non empty that the output also will be non empty.
     * \param first[in] The first Polygon.
     * \param second[in] The second Polygon.
     * \return The union of both Polygons
     */
    [[nodiscard]] Polygons safeUnion(const Polygons first, const Polygons second = Polygons()) const;

    /*!
     * \brief Creates a valid CrossInfillProvider
     * Based on AreaSupport::precomputeCrossInfillTree, but calculates for each mesh separately
     * \param mesh[in] The mesh that is currently processed.
     * \param line_distance[in] The distance between the infill lines of the resulting infill
     * \param line_width[in] What is the width of a line used in the infill.
     * \return A valid CrossInfillProvider. Has to be freed manually to avoid a memory leak.
     */
    SierpinskiFillProvider* generateCrossFillProvider(const SliceMeshStorage& mesh, coord_t line_distance, coord_t line_width);


    /*!
     * \brief Creates the initial influence areas (that can later be propagated down) by placing them below the overhang.
     *
     * Generates Points where the Model should be supported and creates the areas where these points have to be placed.
     *
     * \param mesh[in] The mesh that is currently processed.
     * \param move_bounds[out] Storage for the influence areas.
     * \param storage[in] Background storage, required for adding roofs.
     */
    void generateInitialAreas(const SliceMeshStorage& mesh, std::vector<std::set<TreeSupportElement*>>& move_bounds, SliceDataStorage& storage);

    /*!
     * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
     * \param me[in] Polygons object that has to be offset.
     * \param distance[in] The distance by which me should be offset. Expects values >=0.
     * \param collision[in] The area representing obstacles.
     * \param last_step_offset_without_check[in] The most it is allowed to offset in one step.
     * \param min_amount_offset[in] How many steps have to be done at least. As this uses round offset this increases the amount of vertices, which may be required if Polygons get very small. Required as arcTolerance is not exposed in offset, which should result with a similar result.
     * \return The resulting Polygons object.
     */
    [[nodiscard]] Polygons safeOffsetInc(const Polygons& me, coord_t distance, const Polygons& collision, coord_t safe_step_size, coord_t last_step_offset_without_check, size_t min_amount_offset) const;


    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This helper merges all elements in input with the elements into reduced_new_layer.
     * Elements in input_aabb are merged together if possible, while elements reduced_new_layer_aabb are not checked against each other.
     *
     * \param reduced_aabb[in,out] The already processed elements.
     * \param input_aabb[in] Not yet processed elements
     * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate. Value is the influence area where the center of a circle of support may be placed.
     * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
     * Value is the influence area where the center of a circle of support may be placed.
     * \param influence_areas[in] The influence areas without avoidance removed.
     * \param insert_bp_areas[out] Elements to be inserted into the main dictionary after the Helper terminates.
     * \param insert_model_areas[out] Elements to be inserted into the secondary dictionary after the Helper terminates.
     * \param insert_influence[out] Elements to be inserted into the dictionary containing the largest possibly valid influence area (ignoring if the area may not be there because of avoidance)
     * \param erase[out] Elements that should be deleted from the above dictionaries.
     * \param layer_idx[in] The Index of the current Layer.
     */
    void mergeHelper(std::map<TreeSupportElement, AABB>& reduced_aabb, std::map<TreeSupportElement, AABB>& input_aabb, const std::unordered_map<TreeSupportElement, Polygons>& to_bp_areas, const std::unordered_map<TreeSupportElement, Polygons>& to_model_areas, const std::map<TreeSupportElement, Polygons>& influence_areas, std::unordered_map<TreeSupportElement, Polygons>& insert_bp_areas, std::unordered_map<TreeSupportElement, Polygons>& insert_model_areas, std::unordered_map<TreeSupportElement, Polygons>& insert_influence, std::vector<TreeSupportElement>& erase, const LayerIndex layer_idx);
    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This manages the helper and uses a divide and conquer approach to parallelize this problem. This parallelization can at most accelerate the merging by a factor of 2.
     *
     * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param influence_areas[in] The Elements of the current Layer without avoidances removed. This is the largest possible influence area for this layer.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param layer_idx[in] The current layer.
     */
    void mergeInfluenceAreas(std::unordered_map<TreeSupportElement, Polygons>& to_bp_areas, std::unordered_map<TreeSupportElement, Polygons>& to_model_areas, std::map<TreeSupportElement, Polygons>& influence_areas, LayerIndex layer_idx);

    /*!
     * \brief Checks if an influence area contains a valid subsection and returns the corresponding metadata and the new Influence area.
     *
     * Calculates an influence areas of the layer below, based on the influence area of one element on the current layer.
     * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in we would change our gracious or to_buildplate status the influence areas are instead increased by maximum_move_distance_slow.
     * Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were the case, the radius is not increased instead.
     *
     * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was done to avoid not needed heap allocations.
     *
     * \param settings[in] Which settings have to be used to check validity.
     * \param layer_idx[in] Number of the current layer.
     * \param parent[in] The metadata of the parents influence area.
     * \param relevant_offset[in] The maximal possible influence area. No guarantee regarding validity with current layer collision required, as it is ensured in-function!
     * \param to_bp_data[out] The part of the Influence area that can reach the buildplate.
     * \param to_model_data[out] The part of the Influence area that do not have to reach the buildplate. This has overlap with new_layer_data.
     * \param increased[out]  Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
     * \param overspeed[in] How much should the already offset area be offset again. Usually this is 0.
     * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
     * \return A valid support element for the next layer regarding the calculated influence areas. Empty if no influence are can be created using the supplied influence area and settings.
     */
    std::optional<TreeSupportElement> increaseSingleArea(AreaIncreaseSettings settings, LayerIndex layer_idx, TreeSupportElement* parent, const Polygons& relevant_offset, Polygons& to_bp_data, Polygons& to_model_data, Polygons& increased, const coord_t overspeed, const bool mergelayer);
    /*!
     * \brief Increases influence areas as far as required.
     *
     * Calculates influence areas of the layer below, based on the influence areas of the current layer.
     * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in it would change the gracious or to_buildplate status, the influence areas are instead increased by maximum_move_distance.
     * Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were the case, the radius is not increased instead.
     *
     * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was done to avoid not needed heap allocations.
     *
     * \param to_bp_areas[out] Influence areas that can reach the buildplate
     * \param to_model_areas[out] Influence areas that do not have to reach the buildplate. This has overlap with new_layer_data, as areas that can reach the buildplate are also considered valid areas to the model.
     * This redundancy is required if a to_buildplate influence area is allowed to merge with a to model influence area.
     * \param influence_areas[out] Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
     * \param bypass_merge_areas[out] Influence areas ready to be added to the layer below that do not need merging.
     * \param last_layer[in] Influence areas of the current layer.
     * \param layer_idx[in] Number of the current layer.
     * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
     */
    void increaseAreas(std::unordered_map<TreeSupportElement, Polygons>& to_bp_areas, std::unordered_map<TreeSupportElement, Polygons>& to_model_areas, std::map<TreeSupportElement, Polygons>& influence_areas, std::vector<TreeSupportElement*>& bypass_merge_areas, const std::vector<TreeSupportElement*>& last_layer, const LayerIndex layer_idx, const bool mergelayer);

    /*!
     * \brief Propagates influence downwards, and merges overlapping ones.
     *
     * \param move_bounds[in,out] All currently existing influence areas
     */
    void createLayerPathing(std::vector<std::set<TreeSupportElement*>>& move_bounds);


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
     * \param layer_tree_polygons[out] Resulting branch areas with the layerindex they appear on. layer_tree_polygons.size() has to be at least linear_data.size() as each Influence area in linear_data will save have at least one (that's why it's a vector<vector>) corresponding branch area in layer_tree_polygons.
     * \param inverse_tree_order[in] A mapping that returns the child of every influence area.
     */
    void generateBranchAreas(std::vector<std::pair<LayerIndex, TreeSupportElement*>>& linear_data, std::vector<std::unordered_map<TreeSupportElement*, Polygons>>& layer_tree_polygons, const std::map<TreeSupportElement*, TreeSupportElement*>& inverse_tree_order);

    /*!
     * \brief Applies some smoothing to the outer wall, intended to smooth out sudden jumps as they can happen when a branch moves though a hole.
     *
     * \param layer_tree_polygons[in,out] Resulting branch areas with the layerindex they appear on.
     */
    void smoothBranchAreas(std::vector<std::unordered_map<TreeSupportElement*, Polygons>>& layer_tree_polygons);

    /*!
     * \brief Drop down areas that do rest non-gracefully on the model to ensure the branch actually rests on something.
     *
     * \param layer_tree_polygons[in] Resulting branch areas with the layerindex they appear on.
     * \param linear_data[in] All currently existing influence areas with the layer they are on
     * \param dropped_down_areas[out] Areas that have to be added to support all non-graceful areas.
     * \param inverse_tree_order[in] A mapping that returns the child of every influence area.
     */
    void dropNonGraciousAreas(std::vector<std::unordered_map<TreeSupportElement*, Polygons>>& layer_tree_polygons, const std::vector<std::pair<LayerIndex, TreeSupportElement*>>& linear_data, std::vector<std::vector<std::pair<LayerIndex, Polygons>>>& dropped_down_areas, const std::map<TreeSupportElement*, TreeSupportElement*>& inverse_tree_order);


    /*!
     * \brief Generates Support Floor, ensures Support Roof can not cut of branches, and saves the branches as support to storage
     *
     * \param support_layer_storage[in] Areas where support should be generated.
     * \param support_roof_storage[in] Areas where support was replaced with roof.
     * \param storage[in,out] The storage where the support should be stored.
     */
    void finalizeInterfaceAndSupportAreas(std::vector<Polygons>& support_layer_storage, std::vector<Polygons>& support_roof_storage, SliceDataStorage& storage);

    /*!
     * \brief Draws circles around result_on_layer points of the influence areas and applies some post processing.
     *
     * \param move_bounds[in] All currently existing influence areas
     * \param storage[in,out] The storage where the support should be stored.
     */
    void drawAreas(std::vector<std::set<TreeSupportElement*>>& move_bounds, SliceDataStorage& storage);

    /*!
     * \brief Settings with the indexes of meshes that use these settings.
     *
     */
    std::vector<std::pair<TreeSupportSettings, std::vector<size_t>>> grouped_meshes;

    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes.
     *
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
