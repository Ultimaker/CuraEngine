//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_DENSITY_PROVIDER_H
#define INFILL_DENSITY_PROVIDER_H

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"

namespace cura
{

/*!
 * Parent class of function objects which return the density required for a given region.
 * 
 * This density requirement can be based on user input, distance to the 3d model shell, Z distance to top skin, etc.
 */
class DensityProvider
{
public:
    /*!
     * Get the density over a cubic region
     * 
     * \param aabb the cube over which to ge tthe density
     * \param averaging_statistic The statistic to use: -1 to get the minimum pixel value, 0 to get the average pixel value, 1 to get the maximum pixel value
     * \return the approximate min/average.max density over a cube
     */
    virtual float operator()(const AABB3D& aabb, const int_fast8_t averaging_statistic) const = 0;
    virtual ~DensityProvider()
    {
    };
};

} // namespace cura


#endif // INFILL_DENSITY_PROVIDER_H
