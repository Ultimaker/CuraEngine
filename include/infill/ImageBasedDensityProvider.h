// Copyright (c) 2026 Tim Kuipers
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "../utils/AABB.h"
#include "DensityProvider.h"
#include "geometry/Point3LL.h"

namespace curaengine::utils
{
struct AABB3D;
}

namespace curaengine::infill
{


class ImageBasedDensityProvider : public DensityProvider
{
public:
    ImageBasedDensityProvider(const std::string filename, const utils::AABB aabb);

    virtual ~ImageBasedDensityProvider();

    virtual double operator()(const utils::AABB3D& aabb) const;

protected:
    geometry::Point3LL image_size; //!< dimensions of the image. Third dimension is the amount of channels.
    unsigned char* image = nullptr; //!< image data: rows of channel data per pixel.

    utils::AABB print_aabb; //!< bounding box of print coordinates in which to apply the image
};

} // namespace curaengine::infill
