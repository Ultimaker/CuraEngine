//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_IMAGE_BASED_DENSITY_PROVIDER_H
#define INFILL_IMAGE_BASED_DENSITY_PROVIDER_H

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"

namespace cura
{

class ImageBasedDensityProvider : public DensityProvider
{
public:
    ImageBasedDensityProvider(const std::string filename, const AABB3D aabb);

    virtual ~ImageBasedDensityProvider();

    virtual float operator()(const AABB3D& aabb, const int_fast8_t averaging_statistic) const;

protected:
    Point3 image_size; //!< dimensions of the image. Third dimension is the amount of channels.
    uint_fast8_t channels_used; //!< number of channels from the input image we use (we disregard the alpha channel)
    Point3 grid_size; //!< dimensions of the 3D grid of voxels. Same as image_size, but third channel is the number of images
    std::vector<unsigned char*> images; //!< voxel data as layers of images on top of each other

    AABB3D print_aabb; //!< bounding box of print coordinates in which to apply the image

    /*!
     * Load an image into the vector of images.
     * 
     * \param filename The file name of the image file
     * \param set_size Whether to set the \ref VoxelDensityProvider::image_size rather than checking the size
     */
    void loadImage(const std::string filename, const bool set_size);

    /*!
     * Advance the file name.
     * Make the number in which the file name ends one higher.
     */
    std::string advanceFilename(const std::string& filename);
};

} // namespace cura


#endif // INFILL_IMAGE_BASED_DENSITY_PROVIDER_H
