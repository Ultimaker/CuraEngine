// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_SUPPORT_SUPPORT_AREA_H
#define CURAENGINE_INCLUDE_SUPPORT_SUPPORT_AREA_H

#include <memory>

#include "sliceDataStorage.h"
#include "utils/AABB.h"
#include "utils/polygon.h"

namespace cura::support
{
enum class SupportAreaType : int
{
    NONE = -1,
    OVERHANG = 1,
    OVERHANG_INTERFACE = 2,
    FOUNDATION = 3,
    FOUNDATION_INTERFACE = 4,
    MERGED = 5,
    SUPPORT = 6
};


struct SupportArea
{
    std::shared_ptr<SliceMeshStorage> mesh;
    size_t layer_idx { 0 };
    std::shared_ptr<Polygons> outline;
    SupportAreaType area_type { SupportAreaType::NONE };
    std::shared_ptr<double> area { 0 };
    std::shared_ptr<AABB> bounding_box;
};

} // namespace cura::support

#endif // CURAENGINE_INCLUDE_SUPPORT_SUPPORT_AREA_H
