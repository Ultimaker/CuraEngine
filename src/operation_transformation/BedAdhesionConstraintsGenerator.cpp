// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/BedAdhesionConstraintsGenerator.h"

#include <range/v3/view/filter.hpp>

#include "PrintFeatureType.h"
#include "print_operation/FeatureExtrusion.h"

namespace cura
{

void BedAdhesionConstraintsGenerator::appendConstraints(
    const FeatureExtrusionPtr& feature_extrusion,
    const std::vector<FeatureExtrusionPtr>& all_feature_extrusions,
    std::vector<FeatureExtrusionPtr>& extrusions_after) const
{
    for (const std::shared_ptr<FeatureExtrusion>& other_feature_extrusion : all_feature_extrusions)
    {
        if (other_feature_extrusion != feature_extrusion && other_feature_extrusion->getPrintFeatureType() != PrintFeatureType::SkirtBrim)
        {
            extrusions_after.push_back(other_feature_extrusion);
        }
    }
}

} // namespace cura
