// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <settings/EnumSettings.h>
#include <settings/types/LayerIndex.h>

#include "ExtruderNumber.h"
#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/ContinuousExtruderMoveSequencePtr.h"
#include "print_operation/LayerPlanPtr.h"
#include "print_operation/PrintPlan.h"

namespace cura
{

enum class PrintFeatureType : unsigned char;
enum class EPlatformAdhesion;
class Shape;
class SVG;

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
        coord_t line_width_0_; //!< The skirt/brim line width on the first layer
        coord_t line_width_X_; //!< The skirt/brim line width on above layers
        coord_t skirt_brim_minimal_length_; //!< The minimal brim length
        size_t line_count_; //!< The (minimal) number of brim lines to generate
        coord_t gap_; //!< The gap between the part and the first brim/skirt line
        coord_t brim_inside_margin_;
        size_t skirt_height_;

        coord_t getLineWidth(const LayerIndex& layer_nr) const
        {
            return layer_nr > 0 ? line_width_X_ : line_width_0_;
        }

        coord_t getLineWidth(const ConstLayerPlanPtr& layer_plan) const;
    };

    enum class FirstExtruderOutlineAction
    {
        None, // Do nothing specific
        Save, // Save the first processed outline for each extruder
        Use, // Use the first processed outline for each extruder, instead of the given starting outlines
    };

    using FeatureFootprints = std::map<PrintFeatureType, std::vector<Shape>>;
    using FeatureFootprint = std::map<PrintFeatureType, Shape>;

    static constexpr coord_t min_brim_line_length_ = 3000u; //!< open polyline brim lines smaller than this will be removed
    const SliceDataStorage& storage_;
    SVG* svg_;

private:
    static std::vector<ExtruderNumber> generateUsedExtruders(const PrintPlan* print_plan);

    static size_t calculateMaxHeight(const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs, const EPlatformAdhesion adhesion_type);

    static std::map<ExtruderNumber, ExtruderConfig> generateExtrudersConfigs(std::vector<ExtruderNumber>& used_extruders, const EPlatformAdhesion adhesion_type);

    static std::map<ExtruderNumber, FeatureFootprint> generateFeatureFootprints(const PrintPlan* print_plan, const size_t height);

    /*!
     * Generates the outline for the first offset of each used extruder
     * @param print_plan The print plan, used to retrieve the extrusion feature on the first layers
     * @param brim_extruder_nr The brim specific extruder number, or nullopt if there is none
     * @param height The maximum height to be reached while generating the skirt/brim
     * @param adhesion_type The actual adhesion type
     * @param used_extruders The used extruders, in order of processing
     * @return In case the adhesion type is brim, a map containing the outlines of all the features per extruder
     *         In case the adhesion type is skirt, a map containing a single element with fixed 0, which is a union of
     *         all the outlines of all extruders on the first layers
     */
    void generateBaseAreas(
        const PrintPlan* print_plan,
        const std::optional<ExtruderNumber> brim_extruder_nr,
        const size_t height,
        const EPlatformAdhesion adhesion_type,
        const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
        const std::vector<ExtruderNumber>& used_extruders,
        std::map<ExtruderNumber, Shape>& starting_outlines,
        std::map<ExtruderNumber, Shape>& allowed_areas) const;

    void generateSkirtBrim(
        const EPlatformAdhesion adhesion_type,
        const std::optional<ExtruderNumber> brim_extruder_nr,
        std::vector<ExtruderNumber>& used_extruders,
        const std::map<ExtruderNumber, Shape>& starting_outlines,
        std::map<ExtruderNumber, Shape>& specific_starting_outlines,
        std::map<ExtruderNumber, Shape> allowed_areas_per_extruder,
        const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
        const LayerPlanPtr& layer_plan,
        const FirstExtruderOutlineAction first_extruder_outline_action) const;

    std::vector<ContinuousExtruderMoveSequencePtr> generateOffset(
        const ExtruderNumber extruder_nr,
        const coord_t total_offset,
        const Shape& outline,
        Shape& covered_area,
        std::map<ExtruderNumber, Shape>& allowed_areas_per_extruder,
        const std::map<ExtruderNumber, ExtruderConfig>& extruders_configs,
        const LayerPlanPtr& layer_plan,
        const bool update_allowed_areas) const;
};

} // namespace cura