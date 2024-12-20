// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <utils/Coord_t.h>

#include "feature_generation/MeshFeatureGenerator.h"

namespace cura
{

enum class EFillMethod;
enum class PrintFeatureType : unsigned char;
struct GCodePathConfig;
struct Ratio;
class AngleDegrees;
class SkinPart;
class Shape;

class MeshSkinGenerator : public MeshFeatureGenerator
{
public:
    explicit MeshSkinGenerator(const std::shared_ptr<SliceMeshStorage>& mesh);

protected:
    void generateFeatures(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const std::vector<ExtruderPlanPtr>& extruder_plans, const SliceLayerPart& part)
        const override;

private:
    void processRoofing(const LayerPlanPtr& layer_plan, const SkinPart& skin_part, const ExtruderPlanPtr& extruder_plan) const;

    void processTopBottom(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const SkinPart& skin_part, const ExtruderPlanPtr& extruder_plan) const;

    void processSkinPrintFeature(
        const LayerPlanPtr& layer_plan,
        const Shape& area,
        const GCodePathConfig& config,
        EFillMethod pattern,
        const AngleDegrees skin_angle,
        const Ratio skin_density,
        const PrintFeatureType feature_type,
        const ExtruderPlanPtr& extruder_plan) const;

    static AngleDegrees getInfillAngle(const LayerPlanPtr& layer_plan, const std::vector<AngleDegrees>& angles);
};

} // namespace cura