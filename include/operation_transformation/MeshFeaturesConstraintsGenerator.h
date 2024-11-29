// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_MESHFEATURESCONSTRAINTSGENERATOR_H
#define PATHPROCESSING_MESHFEATURESCONSTRAINTSGENERATOR_H

#include "operation_transformation/FeatureExtrusionsConstraintsGenerator.h"

namespace cura
{

class MeshFeaturesConstraintsGenerator : public FeatureExtrusionsConstraintsGenerator
{
public:
    void appendConstraints(
        const FeatureExtrusionPtr& feature_extrusion,
        const std::vector<FeatureExtrusionPtr>& all_feature_extrusions,
        std::vector<FeatureExtrusionPtr>& extrusions_after) const override;
};

} // namespace cura

#endif // PATHPROCESSING_MESHFEATURESCONSTRAINTSGENERATOR_H
