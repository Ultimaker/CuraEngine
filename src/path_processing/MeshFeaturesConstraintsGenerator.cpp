// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/MeshFeaturesConstraintsGenerator.h"

#include <map>

#include <range/v3/algorithm/contains.hpp>

#include "path_planning/MeshFeatureExtrusion.h"

namespace cura
{

void MeshFeaturesConstraintsGenerator::appendConstraints(
    const FeatureExtrusionPtr& feature_extrusion,
    const std::vector<FeatureExtrusionPtr>& all_feature_extrusions,
    std::vector<FeatureExtrusionPtr>& extrusions_after) const
{
    auto mesh_feature_extrusion = std::dynamic_pointer_cast<MeshFeatureExtrusion>(feature_extrusion);
    if (! mesh_feature_extrusion)
    {
        spdlog::error("The given feature extrusion is not a mesh feature extrusion");
        return;
    }

    std::vector<PrintFeatureType> types_after;
    switch (feature_extrusion->getPrintFeatureType())
    {
    case PrintFeatureType::OuterWall:
        types_after.push_back(PrintFeatureType::InnerWall);
        break;
    case PrintFeatureType::InnerWall:
        break;
    case PrintFeatureType::Infill:
        types_after.push_back(PrintFeatureType::OuterWall);
        break;
    case PrintFeatureType::Skin:
        types_after.push_back(PrintFeatureType::OuterWall);
        break;
    case PrintFeatureType::SkirtBrim:
    case PrintFeatureType::Support:
    case PrintFeatureType::MoveCombing:
    case PrintFeatureType::MoveRetraction:
    case PrintFeatureType::NoneType:
    case PrintFeatureType::PrimeTower:
    case PrintFeatureType::SupportInfill:
    case PrintFeatureType::SupportInterface:
    case PrintFeatureType::NumPrintFeatureTypes:
        break;
    }

    if (types_after.empty())
    {
        return;
    }

    for (const std::shared_ptr<FeatureExtrusion>& other_feature_extrusion : all_feature_extrusions)
    {
        if (other_feature_extrusion == feature_extrusion)
        {
            continue;
        }

        const auto other_mesh_feature = std::dynamic_pointer_cast<MeshFeatureExtrusion>(other_feature_extrusion);
        if (other_mesh_feature && other_mesh_feature->getMesh() == mesh_feature_extrusion->getMesh() && ranges::contains(types_after, other_mesh_feature->getPrintFeatureType()))
        {
            extrusions_after.push_back(other_feature_extrusion);
        }
    }
}

} // namespace cura
