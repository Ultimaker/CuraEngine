/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_IMAGE_BASED_SUBDIVIDER_H
#define INFILL_IMAGE_BASED_SUBDIVIDER_H

#include "../utils/intpoint.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "SierpinskiFillEdge.h"

namespace cura
{

class ImageBasedSubdivider
{
public:
    ImageBasedSubdivider(const std::string filename, const AABB aabb, const coord_t line_width);

    ~ImageBasedSubdivider();

    bool operator()(const SierpinskiFillEdge& e1, const SierpinskiFillEdge& e2) const;

protected:
    Point3 image_size; //!< dimensions of the image. Third dimension is the amount of channels.
    unsigned char* image = nullptr; //!< image data: rows of channel data per pixel.

    AABB aabb; //!< bounding box of print coordinates in which to apply the image
    coord_t line_width; //!< The line width of the fill lines
};

} // namespace cura


#endif // INFILL_IMAGE_BASED_SUBDIVIDER_H
