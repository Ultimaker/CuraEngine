// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONSORDERINGCONSTRAINT_H
#define PATHPROCESSING_FEATUREEXTRUSIONSORDERINGCONSTRAINT_H

#include <memory>

namespace cura
{

class FeatureExtrusion;

struct FeatureExtrusionOrderingConstraint
{
    std::shared_ptr<FeatureExtrusion> feature_before;
    std::shared_ptr<FeatureExtrusion> feature_after;
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONSORDERINGCONSTRAINT_H
