// Copyright (c) 2026 UltiMaker
// Copyright (c) 2026 Tim Kuipers
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "DensityProvider.h"
#include "types/geometry/AABB.h"
#include "types/geometry/Point3LL.h"

namespace curaengine::types::geometry
{
struct AABB3D;
}

namespace curaengine::infill
{


class ImageBasedDensityProvider : public DensityProvider
{
public:
    ImageBasedDensityProvider(const std::string filename, const types::geometry::AABB aabb);

    virtual ~ImageBasedDensityProvider();

    virtual double operator()(const types::geometry::AABB3D& aabb) const;

protected:
    types::geometry::Point3LL image_size; //!< dimensions of the image. Third dimension is the amount of channels.
    unsigned char* image = nullptr; //!< image data: rows of channel data per pixel.

    types::geometry::AABB print_aabb; //!< bounding box of print coordinates in which to apply the image
};

} // namespace curaengine::infill
