// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_MESHFEATUREEXTRUSION_H
#define PATHPLANNING_MESHFEATUREEXTRUSION_H

#include "print_operation/FeatureExtrusion.h"

namespace cura
{

class MeshFeatureExtrusion : public FeatureExtrusion
{
public:
    explicit MeshFeatureExtrusion(const GCodePathConfig& config, const std::shared_ptr<const SliceMeshStorage>& mesh);

    const std::shared_ptr<const SliceMeshStorage>& getMesh() const;

private:
    std::shared_ptr<const SliceMeshStorage> mesh_;
};

} // namespace cura

#endif // PATHPLANNING_MESHFEATUREEXTRUSION_H
