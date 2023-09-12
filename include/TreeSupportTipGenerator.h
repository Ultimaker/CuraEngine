// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef TREESUPPORTTIPGENERATOR_H
#define TREESUPPORTTIPGENERATOR_H

#include "TreeModelVolumes.h"
#include "TreeSupport.h"
#include "TreeSupportBaseCircle.h"
#include "TreeSupportElement.h"
#include "TreeSupportEnums.h"
#include "TreeSupportSettings.h"
#include "boost/functional/hash.hpp" // For combining hashes
#include "polyclipping/clipper.hpp"
#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"
#include "utils/Coord_t.h"
#include "utils/polygon.h"

namespace cura
{


class TreeSupportTipGenerator
{
public:
    TreeSupportTipGenerator(const SliceDataStorage& storage, const SliceMeshStorage& mesh, TreeModelVolumes& volumes_);

    /*!
     * \brief Generate tips, that will later form branches
     *
     * \param storage[in] Background storage, required for adding roofs.
     * \param mesh[in] The mesh that is currently processed. Contains the overhangs.
     * \param move_bounds[out] The storage for the tips.
     * \param additional_support_areas[out] Areas that should have been roofs, but are now support, as they would not generate any lines as roof. Should already be initialised.

     * \return All lines of the \p polylines object, with information for each point regarding in which avoidance it is currently valid in.
     */
    void generateTips(
        SliceDataStorage& storage,
        const SliceMeshStorage& mesh,
        std::vector<std::set<TreeSupportElement*>>& move_bounds,
        std::vector<Polygons>& additional_support_areas,
        std::vector<Polygons>& placed_support_lines_support_areas);

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

    using LineInformation = std::vector<std::pair<Point, TreeSupportTipGenerator::LineStatus>>;

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
    Polygons convertInternalToLines(std::vector<TreeSupportTipGenerator::LineInformation> lines);

    /*!
     * \brief Returns a function, evaluating if a point has to be added now. Required for a splitLines call in generateInitialAreas.
     *
     * \param current_layer[in] The layer on which the point lies
     * \return A function that can be called to evaluate a point.
     */
    std::function<bool(std::pair<Point, TreeSupportTipGenerator::LineStatus>)> getEvaluatePointForNextLayerFunction(size_t current_layer);

    /*!
     * \brief Evaluates which points of some lines are not valid one layer below and which are. Assumes all points are valid on the current layer. Validity is evaluated using
     * supplied lambda.
     *
     * \param lines[in] The lines that have to be evaluated.
     * \param evaluatePoint[in] The function used to evaluate the points.
     * \return A pair with which points are still valid in the first slot and which are not in the second slot.
     */
    std::pair<std::vector<LineInformation>, std::vector<LineInformation>> splitLines(
        std::vector<LineInformation> lines,
        std::function<bool(std::pair<Point, TreeSupportTipGenerator::LineStatus>)> evaluatePoint); // assumes all Points on the current line are valid

    /*!
     * \brief Ensures that every line segment is about distance in length. The resulting lines may differ from the original but all points are on the original
     *
     * \param input[in] The lines on which evenly spaced points should be placed.
     * \param distance[in] The distance the points should be from each other.
     * \param min_points[in] The amount of points that have to be placed. If not enough can be placed the distance will be reduced to place this many points.
     * \param enforce_distance[in] If points should not be added if they are closer than distance to other points.
     * \return A Polygons object containing the evenly spaced points. Does not represent an area, more a collection of points on lines.
     */
    Polygons ensureMaximumDistancePolyline(const Polygons& input, coord_t distance, size_t min_points, bool enforce_distance) const;

    /*!
     * \brief Creates a valid CrossInfillProvider
     * Based on AreaSupport::precomputeCrossInfillTree, but calculates for each mesh separately
     * \param mesh[in] The mesh that is currently processed.
     * \param line_distance[in] The distance between the infill lines of the resulting infill
     * \param line_width[in] What is the width of a line used in the infill.
     * \return A valid CrossInfillProvider. Has to be freed manually to avoid a memory leak.
     */
    std::shared_ptr<SierpinskiFillProvider> generateCrossFillProvider(const SliceMeshStorage& mesh, coord_t line_distance, coord_t line_width) const;


    /*!
     * \brief Drops overhang areas further down until they are valid (at most max_overhang_insert_lag layers)
     * \param mesh[in] The mesh that is currently processed.
     * \param result[out] The dropped overhang ares
     * \param roof[in] Whether the result is for roof generation.
     */
    void dropOverhangAreas(const SliceMeshStorage& mesh, std::vector<Polygons>& result, bool roof);

    /*!
     * \brief Calculates which areas should be supported with roof, and saves these in roof support_roof_drawn
     * \param mesh[in] The mesh that is currently processed.
     */
    void calculateRoofAreas(const SliceMeshStorage& mesh);

    /*!
     * \brief Add a point as a tip
     * \param move_bounds[out] The storage for the tips.
     * \param p[in] The point that will be added and its LineStatus.
     * \param dtt[in] The distance to top the added tip will have.
     * \param insert_layer[in] The layer the tip will be on.
     * \param dont_move_until[in] Until which dtt the branch should not move if possible.
     * \param roof[in] Whether the tip supports a roof.
     * \param skip_ovalisation[in] Whether the tip may be ovalized when drawn later.
     */
    void addPointAsInfluenceArea(
        std::vector<std::set<TreeSupportElement*>>& move_bounds,
        std::pair<Point, LineStatus> p,
        size_t dtt,
        LayerIndex insert_layer,
        size_t dont_move_until,
        bool roof,
        bool skip_ovalisation,
        std::vector<Point> additional_ovalization_targets = std::vector<Point>());


    /*!
     * \brief Add all points of a line as a tip
     * \param move_bounds[out] The storage for the tips.
     * \param lines[in] The lines of which points will be added.
     * \param roof_tip_layers[in] Amount of layers the tip should be drawn as roof.
     * \param insert_layer_idx[in] The layer the tip will be on.
     * \param supports_roof[in] Whether the tip supports a roof.
     * \param dont_move_until[in] Until which dtt the branch should not move if possible.
     */
    void addLinesAsInfluenceAreas(
        std::vector<std::set<TreeSupportElement*>>& move_bounds,
        std::vector<TreeSupportTipGenerator::LineInformation> lines,
        size_t roof_tip_layers,
        LayerIndex insert_layer_idx,
        bool supports_roof,
        size_t dont_move_until,
        bool connect_points);

    /*!
     * \brief Remove tips that should not have been added in the first place.
     * \param move_bounds[in,out] The already added tips
     * \param storage[in] Background storage, required for adding roofs.
     * \param additional_support_areas[in] Areas that should have been roofs, but are now support, as they would not generate any lines as roof.
     */
    void removeUselessAddedPoints(std::vector<std::set<TreeSupportElement*>>& move_bounds, SliceDataStorage& storage, std::vector<Polygons>& additional_support_areas);


    /*!
     * \brief If large areas should be supported by a roof out of regular support lines.
     */
    bool use_fake_roof;

    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes.
     */
    TreeModelVolumes& volumes_;

    /*!
     * \brief Contains config settings to avoid loading them in every function. This was done to improve readability of the code.
     */
    TreeSupportSettings config;


    /*!
     * \brief Minimum area an overhang has to have to be supported.
     */
    const double minimum_support_area;

    /*!
     * \brief Minimum area an overhang has to have to become a roof.
     */
    const double minimum_roof_area;

    /*!
     * \brief Amount of layers of roof. Zero if roof is disabled
     */
    const size_t support_roof_layers;

    /*!
     * \brief Distance between tips, so that the tips form a lime. Is smaller than Tip Diameter.
     */
    const coord_t connect_length;

    /*!
     * \brief Distance between tips, if the tips support an overhang.
     */
    const coord_t support_tree_branch_distance;

    /*!
     * \brief Distance between support roof lines. Is required for generating roof patterns.
     */
    const coord_t support_roof_line_distance;


    /*!
     * \brief Amount of offset to each overhang for support with regular branches (as opposed to roof).
     */
    const coord_t support_outset;

    /*!
     * \brief Amount of offset to each overhang for support with roof (as opposed to regular branches).
     */
    const coord_t roof_outset;

    /*!
     * \brief Whether tips should be printed as roof
     */
    const bool force_tip_to_roof;

    /*!
     * \brief Whether the maximum distance a branch should from a point they support should be limited. Can be violated if required.
     */
    const bool support_tree_limit_branch_reach;

    /*!
     * \brief Maximum distance a branch should from a point they support (in the xy plane). Can be violated if required.
     */
    const coord_t support_tree_branch_reach_limit;

    /*!
     * \brief Distance in layers from the overhang to the first layer with support. This is the z distance in layers+1
     */
    const size_t z_distance_delta;

    /*!
     * \brief Whether the Support Distance Priority is X/Y Overrides Z
     */
    const bool xy_overrides;

    /*!
     * \brief Amount of layers further down than required an overhang can be supported, when Support Distance Priority is X/Y Overrides Z
     */
    size_t max_overhang_insert_lag;

    /*!
     * \brief Area of a tip.
     */
    const double tip_roof_size;

    /*!
     * \brief Whether only support that can rest on a flat surface should be supported.
     */
    const bool only_gracious = SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL;

    /*!
     * \brief Whether minimum_roof_area is a hard limit. If false the roof will be combined with roof above and below, to see if a part of this roof may be part of a valid roof
     * further up/down.
     */
    const bool force_minimum_roof_area = SUPPORT_TREE_MINIMUM_ROOF_AREA_HARD_LIMIT;

    /*!
     * \brief Distance between branches when the branches support a support pattern
     */
    coord_t support_supporting_branch_distance;

    /*!
     * \brief Required to generate cross infill patterns
     */
    std::shared_ptr<SierpinskiFillProvider> cross_fill_provider;

    /*!
     * \brief Map that saves locations of already inserted tips. Used to prevent tips far to close together from being added.
     */
    std::vector<std::unordered_set<Point>> already_inserted;

    /*!
     * \brief Areas that will be saved as support roof
     */
    std::vector<Polygons> support_roof_drawn;

    /*!
     * \brief Areas that will be saved as support roof, originating from tips being replaced with roof areas.
     */
    std::vector<Polygons> roof_tips_drawn;


    std::mutex critical_move_bounds;
    std::mutex critical_roof_tips;
};

} // namespace cura

#endif /* TREESUPPORT_H */