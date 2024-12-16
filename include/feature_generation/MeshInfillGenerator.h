// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <utils/Coord_t.h>

#include "feature_generation/MeshFeatureGenerator.h"

namespace cura
{

class Shape;

class MeshInfillGenerator : public MeshFeatureGenerator
{
public:
    explicit MeshInfillGenerator(const std::shared_ptr<SliceMeshStorage>& mesh);

    bool isActive() const override;

protected:
    void generateFeatures(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const std::vector<ExtruderPlanPtr>& extruder_plans, const SliceLayerPart& part)
        const override;

private:
    /*!
     * \brief Add thicker (multiple layers) sparse infill for a given part in a
     * layer plan.
     *
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param extruder_nr The extruder for which to print all features of the
     * mesh which should be printed with this extruder.
     * \param mesh_config The line config with which to print a print feature.
     * \param part The part for which to create gcode.
     * \return Whether this function added anything to the layer plan.
     */
    void processMultiLayerInfill(const LayerPlanPtr& layer_plan, const ExtruderPlanPtr& extruder_plan, const SliceLayerPart& part) const;

    /*!
     * \brief Add normal sparse infill for a given part in a layer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param extruder_nr The extruder for which to print all features of the
     * mesh which should be printed with this extruder
     * \param mesh_config The line config with which to print a print feature.
     * \param part The part for which to create gcode.
     * \return Whether this function added anything to the layer plan.
     */
    void processSingleLayerInfill(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const ExtruderPlanPtr& extruder_plan, const SliceLayerPart& part) const;

    bool
        partitionInfillBySkinAbove(Shape& infill_below_skin, Shape& infill_not_below_skin, const LayerPlanPtr& layer_plan, const SliceLayerPart& part, coord_t infill_line_width) const;

private:
    const coord_t infill_line_distance_;
};

} // namespace cura