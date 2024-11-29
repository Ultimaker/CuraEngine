// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/WallFeatureExtrusion.h"

namespace cura
{

WallFeatureExtrusion::WallFeatureExtrusion(const GCodePathConfig& config, const std::shared_ptr<const SliceMeshStorage>& mesh, const size_t inset_index)
    : MeshFeatureExtrusion(config, mesh)
    , inset_index_(inset_index)
{
}

size_t WallFeatureExtrusion::getInsetIndex() const
{
    return inset_index_;
}

} // namespace cura
