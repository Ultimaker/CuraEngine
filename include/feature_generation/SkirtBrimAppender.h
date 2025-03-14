// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <settings/EnumSettings.h>
#include <utils/ExtrudersSet.h>

#include "ExtruderNumber.h"
#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/ContinuousExtruderMoveSequencePtr.h"
#include "print_operation/ExtruderPlanPtr.h"
#include "print_operation/LayerPlanPtr.h"
#include "print_operation/PrintPlan.h"

namespace cura
{

enum class EPlatformAdhesion;
class Shape;
class ExtrudersSet;

class SkirtBrimAppender : public PrintOperationTransformer<PrintPlan>
{
public:
    explicit SkirtBrimAppender(const SliceDataStorage& storage);

    void process(PrintPlan* print_plan) override;

private:
    /*!
     * Container to store the pre-extracted settings of an extruder
     */
    struct ExtruderConfig
    {
        bool outside_polys_; //!< Whether to generate brim on the outside
        bool inside_polys_; //!< Whether to generate brim on the inside
        coord_t line_width_; //!< The skirt/brim line width
        coord_t skirt_brim_minimal_length_; //!< The minimal brim length
        size_t line_count_; //!< The (minimal) number of brim lines to generate
        coord_t gap_; //!< The gap between the part and the first brim/skirt line
        coord_t brim_inside_margin_;
        size_t skirt_height_;
    };

    static constexpr coord_t min_brim_line_length_ = 3000u; //!< open polyline brim lines smaller than this will be removed
    const SliceDataStorage& storage_;

private:
    static std::tuple<std::vector<ConstExtruderPlanPtr>, ExtrudersSet> generateUsedExtruders(const PrintPlan* print_plan);

    static size_t calculateMaxHeight(const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs, const EPlatformAdhesion adhesion_type);

    static std::map<ExtruderNumber, ExtruderConfig> generateExtrudersConfigs(const ExtrudersSet& used_extruders, const EPlatformAdhesion adhesion_type);

    static std::map<ExtruderNumber, Shape> generateStartingOutlines(
        const PrintPlan* print_plan,
        const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
        const size_t height,
        const EPlatformAdhesion adhesion_type,
        const ExtrudersSet& used_extruders);

    std::map<ExtruderNumber, Shape> generateAllowedAreas(
        const std::map<ExtruderNumber, Shape>& starting_outlines,
        const EPlatformAdhesion adhesion_type,
        const ExtrudersSet& used_extruders,
        const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
        const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs) const;

    static void generateSkirtBrim(
        const EPlatformAdhesion adhesion_type,
        const ExtrudersSet& used_extruders,
        const std::vector<ConstExtruderPlanPtr> first_extruder_plans,
        const std::map<ExtruderNumber, Shape>& starting_outlines,
        std::map<ExtruderNumber, Shape> allowed_areas_per_extruder,
        const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
        const LayerPlanPtr& layer_plan);

    static std::vector<ContinuousExtruderMoveSequencePtr> generateOffset(
        const ExtruderNumber extruder_nr,
        const coord_t total_offset,
        Shape& covered_area,
        std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
        const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
        const LayerPlanPtr& layer_plan);
};

} // namespace cura