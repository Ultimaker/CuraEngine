// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/MeshFeatureExtrusion.h"

namespace cura
{

MeshFeatureExtrusion::MeshFeatureExtrusion(const PrintFeatureType type, const coord_t nominal_line_width, const std::shared_ptr<const SliceMeshStorage>& mesh)
    : FeatureExtrusion(type, nominal_line_width)
    , mesh_(mesh)
{
}

const std::shared_ptr<const SliceMeshStorage>& MeshFeatureExtrusion::getMesh() const
{
    return mesh_;
}

} // namespace cura
