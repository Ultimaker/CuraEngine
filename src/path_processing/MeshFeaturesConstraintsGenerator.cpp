// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/MeshFeaturesConstraintsGenerator.h"

#include <map>

#include "path_planning/MeshFeatureExtrusion.h"
#include "path_processing/FeatureExtrusionOrderingConstraint.h"

namespace cura
{

void MeshFeaturesConstraintsGenerator::appendConstraints(const std::vector<FeatureExtrusionPtr>& feature_extrusions, std::vector<FeatureExtrusionOrderingConstraint>& constraints)
    const
{
    using FeatureByType = std::map<PrintFeatureType, std::vector<std::shared_ptr<FeatureExtrusion>>>;
    std::map<std::shared_ptr<const SliceMeshStorage>, FeatureByType> features_by_mesh;
    struct FeatureTypeOrderingConstraint
    {
        PrintFeatureType type_before;
        PrintFeatureType type_after;
    };

    std::vector<FeatureTypeOrderingConstraint> types_constraints;
    types_constraints.push_back(FeatureTypeOrderingConstraint{ .type_before = PrintFeatureType::OuterWall, .type_after = PrintFeatureType::InnerWall });
    types_constraints.push_back(FeatureTypeOrderingConstraint{ .type_before = PrintFeatureType::Infill, .type_after = PrintFeatureType::OuterWall });
    types_constraints.push_back(FeatureTypeOrderingConstraint{ .type_before = PrintFeatureType::Skin, .type_after = PrintFeatureType::OuterWall });

    for (const std::shared_ptr<FeatureExtrusion>& feature_extrusion : feature_extrusions)
    {
        if (const auto mesh_feature = std::dynamic_pointer_cast<MeshFeatureExtrusion>(feature_extrusion))
        {
            features_by_mesh[mesh_feature->getMesh()][mesh_feature->getPrintFeatureType()].push_back(feature_extrusion);
        }
    }

    for (auto& [mesh, mesh_feature_extrusions] : features_by_mesh)
    {
        for (const FeatureTypeOrderingConstraint& constraint : types_constraints)
        {
            for (const std::shared_ptr<FeatureExtrusion>& mesh_feature_extrusion_before : mesh_feature_extrusions[constraint.type_before])
            {
                for (const std::shared_ptr<FeatureExtrusion>& mesh_feature_extrusion_after : mesh_feature_extrusions[constraint.type_after])
                {
                    constraints.emplace_back(mesh_feature_extrusion_before, mesh_feature_extrusion_after);
                }
            }
        }
    }
}

} // namespace cura
