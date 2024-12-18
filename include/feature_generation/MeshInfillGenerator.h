// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <utils/Coord_t.h>
#include <utils/ExtrusionLine.h>

#include "feature_generation/MeshFeatureGenerator.h"

namespace cura
{

enum class EFillMethod;
class Settings;
class LightningGenerator;
class Shape;

class MeshInfillGenerator : public MeshFeatureGenerator
{
public:
    explicit MeshInfillGenerator(const std::shared_ptr<SliceMeshStorage>& mesh);

    void preCalculateData() override;

    bool isActive() const override;

protected:
    void generateFeatures(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const std::vector<ExtruderPlanPtr>& extruder_plans, const SliceLayerPart& part)
        const override;

private:
    /*!
     * \brief Add thicker (multiple layers) sparse infill for a given part in a layer plan.
     */
    void processMultiLayerInfill(
        const SliceLayerPart& part,
        const Settings& settings,
        const LayerPlanPtr& layer_plan,
        const size_t last_idx,
        const EFillMethod infill_pattern,
        const coord_t infill_line_width,
        const coord_t infill_overlap,
        const bool zig_zaggify_infill,
        const auto generate_infill,
        std::vector<std::vector<VariableWidthLines>>& wall_tool_paths,
        OpenLinesSet& infill_lines,
        Shape& infill_polygons) const;

    /*!
     * \brief Add normal sparse infill for a given part in a layer.
     */
    void processSingleLayerInfill(
        const SliceLayerPart& part,
        const size_t last_idx,
        const EFillMethod infill_pattern,
        const size_t combine_idx,
        const coord_t infill_overlap,
        const auto generate_infill) const;

    bool partitionInfillBySkinAbove(Shape& infill_below_skin, Shape& infill_not_below_skin, const LayerPlanPtr& layer_plan, const SliceLayerPart& part, coord_t infill_line_width)
        const;

private:
    const coord_t infill_line_distance_;
    std::shared_ptr<LightningGenerator> lightning_generator_; //!< Pre-computed structure for Lightning type infill
};

} // namespace cura