//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_IMAGE_BASED_DENSITY_PROVIDER_H
#define INFILL_IMAGE_BASED_DENSITY_PROVIDER_H

#include "../utils/AABB.h"

#include "DensityProvider.h"

namespace cura
{

struct AABB3D;

class ImageBasedDensityProvider : public DensityProvider
{
public:
    ImageBasedDensityProvider(const std::string filename, const AABB aabb);

    virtual ~ImageBasedDensityProvider();

    virtual float operator()(const AABB3D& aabb) const;

protected:
    Point3 image_size; //!< dimensions of the image. Third dimension is the amount of channels.
    unsigned char* image = nullptr; //!< image data: rows of channel data per pixel.

    AABB print_aabb; //!< bounding box of print coordinates in which to apply the image
};

} // namespace cura


#endif // INFILL_IMAGE_BASED_DENSITY_PROVIDER_H
