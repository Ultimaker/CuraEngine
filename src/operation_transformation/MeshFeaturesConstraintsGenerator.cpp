// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/MeshFeaturesConstraintsGenerator.h"

#include <range/v3/algorithm/contains.hpp>

#include "print_operation/WallFeatureExtrusion.h"
#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"

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

    const PrintFeatureType feature_type = feature_extrusion->getPrintFeatureType();
    const Settings& mesh_settings = mesh_feature_extrusion->getMesh()->settings;
    const bool infill_before_walls = mesh_settings.get<bool>("infill_before_walls");
    const auto inset_direction = mesh_settings.get<InsetDirection>("inset_direction");
#warning Handle pack_by_inset
    const auto pack_by_inset = ! mesh_settings.get<bool>("optimize_wall_printing_order");

    std::optional<size_t> inset_index_after;
    if (const auto wall_feature_extrusion = std::dynamic_pointer_cast<WallFeatureExtrusion>(feature_extrusion))
    {
        if (inset_direction == InsetDirection::INSIDE_OUT)
        {
            if (wall_feature_extrusion->getInsetIndex() > 0)
            {
                inset_index_after = wall_feature_extrusion->getInsetIndex() - 1;
            }
        }
        else if (inset_direction == InsetDirection::OUTSIDE_IN)
        {
            inset_index_after = wall_feature_extrusion->getInsetIndex() + 1;
        }
    }

    std::vector<PrintFeatureType> types_after;

    if (infill_before_walls)
    {
        if (feature_type == PrintFeatureType::Infill)
        {
            types_after.push_back(PrintFeatureType::OuterWall);
            types_after.push_back(PrintFeatureType::InnerWall);
        }
    }
    else
    {
        if (feature_type == PrintFeatureType::OuterWall || feature_type == PrintFeatureType::InnerWall)
        {
            types_after.push_back(PrintFeatureType::Infill);
        }
    }

    if (types_after.empty() && ! inset_index_after.has_value())
    {
        // Early out, we obviously have no constraint to generate
        return;
    }

    for (const FeatureExtrusionPtr& other_feature_extrusion : all_feature_extrusions)
    {
        if (other_feature_extrusion == feature_extrusion)
        {
            continue;
        }

        const auto other_mesh_feature = std::dynamic_pointer_cast<MeshFeatureExtrusion>(other_feature_extrusion);
        if (other_mesh_feature && other_mesh_feature->getMesh() == mesh_feature_extrusion->getMesh())
        {
            const auto other_wall_feature = std::dynamic_pointer_cast<WallFeatureExtrusion>(other_feature_extrusion);
            bool process_after = inset_index_after.has_value() && other_wall_feature && other_wall_feature->getInsetIndex() == inset_index_after.value();
            process_after |= ranges::contains(types_after, other_mesh_feature->getPrintFeatureType());
            if (process_after)
            {
                extrusions_after.push_back(other_feature_extrusion);
            }
        }
    }
}

} // namespace cura
