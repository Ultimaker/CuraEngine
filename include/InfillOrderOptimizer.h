// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <optional>
#include <unordered_map>

#include "types/EFillMethod.h"
#include "utils/ExtrusionLine.h"

namespace curaengine::types
{
enum class EFillMethod;
struct MeshPathConfigs;
}

namespace curaengine::geometry
{
class OpenLinesSet;
class Shape;
}

namespace curaengine
{




class LayerPlan;
class Settings;
class SliceDataStorage;
class SliceMeshStorage;


/*!
 * Optimizer for infill lines ordering. When an end position is given, infill extrusions are optimized in a way that ending printing the infill
 * will always end as close as possible to the seam of the wall that is printed after.
 * We also try to consider the following rules, if applicable and possible:
 *  - It is better to print the skin support after the rest, so that bridging lines have more printed material
 *    to stick to. This is ensured by setting the initial ordering of the infill_parts_.
 *  - We consider that infill walls are printed on the contour, thus they will always contain the closest location
 *    to the nearest incoming seam., and will always be printed at last.
 */
class InfillOrderOptimizer
{
public:
    /*! Types of infill areas. The order determines the preferred printing order. */
    enum class InfillPartArea
    {
        Infill,
        SkinSupport,
    };

    /*! Constructor for infill ordering optimizer. */
    InfillOrderOptimizer();

    /*! Add a set of open polylines to be printed consecutively. */
    void addPart(InfillPartArea part_area, geometry::OpenLinesSet& lines);

    /*! Add a set of closed polylines to be printed consecutively. */
    void addPart(InfillPartArea part_area, const geometry::Shape& polygons);

    /*! Add a set of extrusion lines to be printed consecutively. */
    void addPart(InfillPartArea part_area, const std::vector<std::vector<utils::VariableWidthLines>>& walls);

    /*! Process the paths ordering optimization. */
    void optimize(const bool skin_support_interlace_lines, const std::optional<geometry::Point2LL>& near_end_location = std::nullopt);

    /*!
     * Adds the ordered infill parts to the given layer plan.
     * \return Whether anything was added to the layer plan.
     */
    bool addToLayer(
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<geometry::Point2LL>& near_end_location,
        const types::EFillMethod infill_pattern,
        const types::MeshPathConfigs& mesh_config,
        const SliceDataStorage& storage,
        const SliceMeshStorage& mesh,
        const size_t extruder_nr,
        const utils::coord_t start_move_inwards_length,
        const utils::coord_t end_move_inwards_length,
        const geometry::Shape& infill_inner_contour,
        const utils::coord_t skin_support_line_distance,
        const geometry::Shape& infill_below_skin,
        const types::AngleDegrees& skin_support_angle) const;

private:
    /*! Types of infill print types. The order determines the preferred printing order. */
    enum class InfillPartType
    {
        Lines,
        Polygons,
        ExtrusionLines,
    };

    /*! Helper structure that holds an infill part to be printed as a whole. */
    struct InfillPart
    {
        InfillPartArea area;
        InfillPartType type;
        union
        {
            const geometry::Shape* polygons;
            geometry::OpenLinesSet* lines;
            const std::vector<std::vector<utils::VariableWidthLines>>* extrusion_lines;
        } paths;
    };

private:
    /*!
     * Calculates whether the given lines set has a vertex closer to the given location
     * @param line_set The lines set to be printed
     * @param location The target location that we should be close to
     * @param[in,out] closest_point The index of the line and vertex that is closest to the given location, which will be updated if we found a closer vertex
     * @param[in,out] closest_distance_squared The global closest distance to the given location, which will be updated if we found a closer vertex
     * @return Whether we have found a vertex in the lines set closer to the given location than the actual closest location
     */
    template<class LinesSetType>
    bool isCloserTo(const LinesSetType& line_set, const geometry::Point2LL& location, std::optional<std::pair<size_t, size_t>>& closest_point, utils::coord_t& closest_distance_squared);

    /*! Add the given part to the layer plan using the given settings */
    void addToLayer(
        const InfillPart& part,
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<geometry::Point2LL>& near_start_location,
        const bool reverse_print_direction,
        const types::EFillMethod infill_pattern,
        const types::MeshPathConfigs& mesh_config,
        const SliceDataStorage& storage,
        const SliceMeshStorage& mesh,
        const size_t extruder_nr,
        const utils::coord_t start_move_inwards_length,
        const utils::coord_t end_move_inwards_length,
        const geometry::Shape& infill_inner_contour,
        const utils::coord_t skin_support_line_distance,
        const geometry::Shape& infill_below_skin,
        const types::AngleDegrees& skin_support_angle) const;

    /*! Add the given infill lines to the layer plan using the given settings */
    void addInfillLinesToLayer(
        const geometry::OpenLinesSet& lines,
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<geometry::Point2LL>& near_start_location,
        const bool reverse_print_direction,
        const types::EFillMethod infill_pattern,
        const types::MeshPathConfigs& mesh_config,
        const utils::coord_t start_move_inwards_length,
        const utils::coord_t end_move_inwards_length,
        const geometry::Shape& infill_inner_contour,
        const bool enable_travel_optimization,
        const types::Ratio& flow_ratio,
        const double fan_speed,
        const std::unordered_multimap<const geometry::Polyline*, const geometry::Polyline*>& order_requirements) const;

    /*! Add the given skin support lines to the layer plan using the given settings */
    void addSkinSupportLinesToLayer(
        const geometry::OpenLinesSet& lines,
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<geometry::Point2LL>& near_start_location,
        const bool reverse_print_direction,
        const types::MeshPathConfigs& mesh_config,
        const utils::coord_t skin_support_line_distance,
        const geometry::Shape& infill_below_skin,
        const types::AngleDegrees& skin_support_angle,
        const bool enable_travel_optimization,
        const types::Ratio& flow_ratio) const;

    /*! Indicates whether part1 should be printed by default before part2 */
    static bool shouldPrintBefore(const InfillPart& part1, const InfillPart& part2);

private:
    std::vector<InfillPart> infill_parts_;
};
} // namespace curaengine
