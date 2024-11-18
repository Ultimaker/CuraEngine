// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONSCONSTRAINTSGENERATOR_H
#define PATHPROCESSING_FEATUREEXTRUSIONSCONSTRAINTSGENERATOR_H

#include <vector>

#include "path_planning/FeatureExtrusionPtr.h"

namespace cura
{

class FeatureExtrusionsConstraintsGenerator
{
public:
    virtual void appendConstraints(
        const FeatureExtrusionPtr& feature_extrusion,
        const std::vector<FeatureExtrusionPtr>& all_feature_extrusions,
        std::vector<FeatureExtrusionPtr>& extrusions_after) const = 0;
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONSCONSTRAINTSGENERATOR_H
