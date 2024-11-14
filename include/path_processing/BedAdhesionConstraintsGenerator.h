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
    void appendConstraints(const std::vector<FeatureExtrusionPtr>& feature_extrusions, std::vector<FeatureExtrusionOrderingConstraint>& constraints) const override;
};

} // namespace cura

#endif // PATHPROCESSING_BEDADHESIONCONSTRAINTSGENERATOR_H
