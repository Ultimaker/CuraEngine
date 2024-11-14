// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_FEATUREEXTRUSIONPTR_H
#define PATHPLANNING_FEATUREEXTRUSIONPTR_H

#include <memory>

namespace cura
{

class FeatureExtrusion;

using FeatureExtrusionPtr = std::shared_ptr<FeatureExtrusion>;
using ConstFeatureExtrusionPtr = std::shared_ptr<const FeatureExtrusion>;

} // namespace cura

#endif // PATHPLANNING_FEATUREEXTRUSIONPTR_H
