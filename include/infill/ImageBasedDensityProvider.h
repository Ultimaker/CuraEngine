// Copyright (c) 2026 Tim Kuipers
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "../utils/AABB.h"
#include "DensityProvider.h"
#include "geometry/Point3LL.h"

namespace cura
{

struct AABB3D;

class ImageBasedDensityProvider : public DensityProvider
{
public:
    ImageBasedDensityProvider(const std::string filename, const AABB aabb);

    virtual ~ImageBasedDensityProvider();

    virtual double operator()(const AABB3D& aabb) const;

protected:
    Point3LL image_size; //!< dimensions of the image. Third dimension is the amount of channels.
    unsigned char* image = nullptr; //!< image data: rows of channel data per pixel.

    AABB print_aabb; //!< bounding box of print coordinates in which to apply the image
};

} // namespace cura
