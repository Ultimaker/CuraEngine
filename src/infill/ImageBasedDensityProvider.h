/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_IMAGE_BASED_DENSITY_PROVIDER_H
#define INFILL_IMAGE_BASED_DENSITY_PROVIDER_H

#include "../utils/intpoint.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"
#include "SierpinskiFillEdge.h"

namespace cura
{

class ImageBasedDensityProvider : public DensityProvider
{
public:
    ImageBasedDensityProvider(const std::string filename, const AABB aabb);

    virtual ~ImageBasedDensityProvider();

    virtual float operator()(const Point& a, const Point& b, const Point& c, const Point& d) const;

protected:
    Point3 image_size; //!< dimensions of the image. Third dimension is the amount of channels.
    unsigned char* image = nullptr; //!< image data: rows of channel data per pixel.

    AABB aabb; //!< bounding box of print coordinates in which to apply the image
};

} // namespace cura


#endif // INFILL_IMAGE_BASED_DENSITY_PROVIDER_H
