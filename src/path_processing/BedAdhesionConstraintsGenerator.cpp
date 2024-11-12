// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/BedAdhesionConstraintsGenerator.h"

#include <range/v3/view/filter.hpp>

#include "PrintFeatureType.h"
#include "path_planning/FeatureExtrusion.h"
#include "path_processing/FeatureExtrusionOrderingConstraint.h"

namespace cura
{

void BedAdhesionConstraintsGenerator::appendConstraints(
    const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions,
    std::vector<FeatureExtrusionOrderingConstraint>& constraints) const
{
    // Helper functions to iterate on features by type
    auto type_is = [](PrintFeatureType feature_type)
    {
        return [&feature_type](const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
        {
            return feature_extrusion->getPrintFeatureType() == feature_type;
        };
    };

    auto type_is_not = [](PrintFeatureType feature_type)
    {
        return [&feature_type](const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
        {
            return feature_extrusion->getPrintFeatureType() != feature_type;
        };
    };

    // First process the bed adhesion features
    for (const std::shared_ptr<FeatureExtrusion>& adhesion_feature_extrusion : feature_extrusions | ranges::views::filter(type_is(PrintFeatureType::SkirtBrim)))
    {
        for (const std::shared_ptr<FeatureExtrusion>& non_adhesion_feature_extrusion : feature_extrusions | ranges::views::filter(type_is_not(PrintFeatureType::SkirtBrim)))
        {
            constraints.emplace_back(adhesion_feature_extrusion, non_adhesion_feature_extrusion);
        }
    }
}

} // namespace cura
