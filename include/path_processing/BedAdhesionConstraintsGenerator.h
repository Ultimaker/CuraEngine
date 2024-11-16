// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_BEDADHESIONCONSTRAINTSGENERATOR_H
#define PATHPROCESSING_BEDADHESIONCONSTRAINTSGENERATOR_H

#include "path_processing/FeatureExtrusionsConstraintsGenerator.h"

namespace cura
{

class BedAdhesionConstraintsGenerator : public FeatureExtrusionsConstraintsGenerator
{
public:
    void appendConstraints(
        const FeatureExtrusionPtr& feature_extrusion,
        const std::vector<FeatureExtrusionPtr>& all_feature_extrusions,
        std::vector<FeatureExtrusionPtr>& extrusions_after) const override;
};

} // namespace cura

#endif // PATHPROCESSING_BEDADHESIONCONSTRAINTSGENERATOR_H
