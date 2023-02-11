// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_SUPPORT_SUPPORT_AREA_H
#define CURAENGINE_INCLUDE_SUPPORT_SUPPORT_AREA_H

#include <memory>

#include "sliceDataStorage.h"
#include "utils/polygon.h"

namespace cura::support
{
enum class SupportAreaType : int
{
    OVERHANG = 1,
    FOUNDATION = 2,
    SUPPORT = 3
};


struct SupportArea
{
    std::shared_ptr<SliceMeshStorage> mesh;
    size_t layer_idx;
    std::shared_ptr<Polygons> outline;
    SupportAreaType area_type;
    double area;
};

} // namespace cura::support

#endif // CURAENGINE_INCLUDE_SUPPORT_SUPPORT_AREA_H
