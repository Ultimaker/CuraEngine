// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_WALLFEATUREEXTRUSION_H
#define PATHPLANNING_WALLFEATUREEXTRUSION_H

#include "print_operation/MeshFeatureExtrusion.h"

namespace cura
{

class WallFeatureExtrusion : public MeshFeatureExtrusion
{
public:
    explicit WallFeatureExtrusion(const GCodePathConfig& config, const std::shared_ptr<const SliceMeshStorage>& mesh, const size_t inset_index);

    size_t getInsetIndex() const;

private:
    const size_t inset_index_;
};

} // namespace cura

#endif // PATHPLANNING_WALLFEATUREEXTRUSION_H
