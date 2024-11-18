// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/MeshFeatureExtrusion.h"

namespace cura
{

MeshFeatureExtrusion::MeshFeatureExtrusion(const GCodePathConfig& config, const std::shared_ptr<const SliceMeshStorage>& mesh)
    : FeatureExtrusion(config)
    , mesh_(mesh)
{
}

const std::shared_ptr<const SliceMeshStorage>& MeshFeatureExtrusion::getMesh() const
{
    return mesh_;
}

} // namespace cura
