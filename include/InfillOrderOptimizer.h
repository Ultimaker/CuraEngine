// Copyright (c) 2026 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INFILL_ORDER_OPTIMIZER_H
#define INFILL_ORDER_OPTIMIZER_H

#include <optional>

#include "utils/ExtrusionLine.h"

namespace cura
{

enum class EFillMethod;
class OpenLinesSet;
class Shape;
class LayerPlan;
class Settings;
class SliceDataStorage;
class SliceMeshStorage;
struct MeshPathConfigs;

/*!
 * Optimizer for infill lines ordering. Infill extrusions are optimized in a way that ending printing the infill
 * should always end as close as possible to the seam of the wall that is printed after. In some cases that may * not possible though, e.g. when wall are printed before.
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

    /*!
     * Constructor for infill ordering optimizer.
     */
    InfillOrderOptimizer();

    void addPart(InfillPartArea part_area, OpenLinesSet& lines);

    void addPart(InfillPartArea part_area, const Shape& polygons);

    void addPart(InfillPartArea part_area, const std::vector<std::vector<VariableWidthLines>>& walls);

    /*! Process the paths ordering optimization. The result can be retrieved in the path_optimizer_ variable. */
    void optimize(const bool skin_support_interlace_lines, const std::optional<Point2LL>& near_end_location = std::nullopt);

    /*!
     * Adds the insets to the given layer plan.
     *
     * The insets and the layer plan are passed to the constructor of this
     * class, so this optimize function needs no additional information.
     * \return Whether anything was added to the layer plan.
     */
    bool addToLayer(
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<Point2LL>& near_end_location,
        const EFillMethod infill_pattern,
        const MeshPathConfigs& mesh_config,
        const SliceDataStorage& storage,
        const SliceMeshStorage& mesh,
        const size_t extruder_nr,
        const coord_t start_move_inwards_length,
        const coord_t end_move_inwards_length,
        const Shape& infill_inner_contour,
        const coord_t skin_support_line_distance,
        const Shape& infill_below_skin,
        const AngleDegrees& skin_support_angle) const;

private:
    /*! Types of infill print types. The order determines the preferred printing order. */
    enum class InfillPartType
    {
        Lines,
        Polygons,
        ExtrusionLines,
    };

    struct InfillPart
    {
        InfillPartArea area;
        InfillPartType type;
        union
        {
            const Shape* polygons;
            OpenLinesSet* lines;
            const std::vector<std::vector<VariableWidthLines>>* extrusion_lines;
        } paths;
    };

private:
    template<class LinesSetType>
    bool isCloserTo(const LinesSetType& line_set, const Point2LL& location, std::optional<std::pair<size_t, size_t>>& closest_point, coord_t& closest_distance_squared);

    void addToLayer(
        const InfillPart& part,
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<Point2LL>& near_start_location,
        const bool reverse_print_direction,
        const EFillMethod infill_pattern,
        const MeshPathConfigs& mesh_config,
        const SliceDataStorage& storage,
        const SliceMeshStorage& mesh,
        const size_t extruder_nr,
        const coord_t start_move_inwards_length,
        const coord_t end_move_inwards_length,
        const Shape& infill_inner_contour,
        const coord_t skin_support_line_distance,
        const Shape& infill_below_skin,
        const AngleDegrees& skin_support_angle) const;

    void addInfillLinesToLayer(
        const OpenLinesSet& lines,
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<Point2LL>& near_start_location,
        const bool reverse_print_direction,
        const EFillMethod infill_pattern,
        const MeshPathConfigs& mesh_config,
        const coord_t start_move_inwards_length,
        const coord_t end_move_inwards_length,
        const Shape& infill_inner_contour,
        const bool enable_travel_optimization,
        const Ratio& flow_ratio,
        const double fan_speed,
        const std::unordered_multimap<const Polyline*, const Polyline*>& order_requirements) const;

    void addSkinSupportLinesToLayer(
        const OpenLinesSet& lines,
        LayerPlan& layer_plan,
        const Settings& settings,
        const std::optional<Point2LL>& near_start_location,
        const bool reverse_print_direction,
        const MeshPathConfigs& mesh_config,
        const coord_t skin_support_line_distance,
        const Shape& infill_below_skin,
        const AngleDegrees& skin_support_angle,
        const bool enable_travel_optimization,
        const Ratio& flow_ratio) const;

    static bool shouldPrintBefore(const InfillPart& part1, const InfillPart& part2);

private:
    std::vector<InfillPart> infill_parts_;
};
} // namespace cura

#endif // INSET_ORDER_OPTIMIZER_H
