// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <settings/EnumSettings.h>

#include "ExtruderNumber.h"
#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/ExtruderPlanPtr.h"
#include "print_operation/FeatureExtrusionPtr.h"
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
     * A helper class to store an offset yet to be performed on either an outline polygon, or based on an earlier generated brim line.
     */
    struct Offset
    {
        Offset(
            const std::variant<const Shape*, int>& reference_outline_or_index,
            const bool outside,
            const bool inside,
            const coord_t offset_value,
            const coord_t total_offset,
            const size_t inset_idx,
            const size_t extruder_nr,
            const bool is_last)
            : reference_outline_or_index_(reference_outline_or_index)
            , outside_(outside)
            , inside_(inside)
            , offset_value_(offset_value)
            , total_offset_(total_offset)
            , inset_idx_(inset_idx)
            , extruder_nr_(extruder_nr)
            , is_last_(is_last)
        {
        }

        std::variant<const Shape*, int> reference_outline_or_index_;
        bool outside_; //!< Wether to offset outward from the reference polygons
        bool inside_; //!< Wether to offset inward from the reference polygons
        coord_t offset_value_; //!< Distance by which to offset from the reference
        coord_t total_offset_; //!< Total distance from the model
        int inset_idx_; //!< The outset index of this brimline
        ExtruderNumber extruder_nr_; //!< The extruder by which to print this brim line
        bool is_last_; //!< Whether this is the last planned offset for this extruder.
    };

    /*!
     * Container to store the pre-extracted settings of an extruder
     */
    struct ExtruderConfig
    {
        bool outside_polys_; //!< Whether to generate brim on the outside
        bool inside_polys_; //!< Whether to generate brim on the inside
        coord_t line_width_; //!< The skirt/brim line width
        coord_t skirt_brim_minimal_length_; //!< The minimal brim length
        int line_count_; //!< The (minimal) number of brim lines to generate
        coord_t gap_; //!< The gap between the part and the first brim/skirt line
        coord_t brim_inside_margin_;
    };

    static constexpr coord_t min_brim_line_length_ = 3000u; //!< open polyline brim lines smaller than this will be removed
    std::map<ExtruderNumber, ExtruderConfig> extruders_configs_;
    const SliceDataStorage& storage_;

private:
    static bool sortOffsets(const Offset& a, const Offset& b)
    {
        // Use extruder_nr in case both extruders have the same offset settings.
        return a.total_offset_ != b.total_offset_ ? a.total_offset_ < b.total_offset_ : a.extruder_nr_ < b.extruder_nr_;
    };

    static std::map<ExtruderNumber, Shape> generateStartingOutlines(const PrintPlan* print_plan,
        const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
        const size_t height,
        const EPlatformAdhesion adhesion_type,
        const ExtrudersSet& used_extruders);

    std::vector<Offset> generateBrimOffsetPlan(const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
        const EPlatformAdhesion adhesion_type,
        const std::map<ExtruderNumber, Shape>& starting_outlines,
        const ExtrudersSet &used_extruders) const;

    std::map<ExtruderNumber, Shape> generateAllowedAreas(const std::map<ExtruderNumber, Shape>& starting_outlines,
        const EPlatformAdhesion adhesion_type,
        const ExtrudersSet &used_extruders,
        const std::optional<ExtruderNumber> skirt_brim_extruder_nr,
        const SliceDataStorage& storage) const;

    void generateSkirtBrim(const SliceDataStorage& storage,
        const EPlatformAdhesion adhesion_type,
        const std::map<ExtruderNumber, Shape>& starting_outlines,
        std::vector<Offset>& offset_plan,
        std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
        PrintPlan* print_plan) const;

    void generateSkirtBrimV2(const SliceDataStorage& storage,
        const EPlatformAdhesion adhesion_type,
        const ExtrudersSet &used_extruders,
        const std::vector<ConstExtruderPlanPtr> first_extruder_plans,
        const std::map<ExtruderNumber,
        Shape>& starting_outlines,
        std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
        PrintPlan* print_plan) const;

    FeatureExtrusionPtr generateOffset(const Offset& offset,
        const std::map<ExtruderNumber, Shape>& starting_outlines,
        Shape& covered_area,
        std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
        const LayerPlanPtr &layer_plan) const;

    FeatureExtrusionPtr generateOffsetV2(const ExtruderNumber extruder_nr,
        const coord_t total_offset,
        Shape& covered_area,
        std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
        const LayerPlanPtr &layer_plan) const;
};

} // namespace cura