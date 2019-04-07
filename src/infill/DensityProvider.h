//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_DENSITY_PROVIDER_H
#define INFILL_DENSITY_PROVIDER_H

namespace cura
{

struct AABB3D;

/*!
 * Parent class of function objects which return the density required for a given region.
 * 
 * This density requirement can be based on user input, distance to the 3d model shell, Z distance to top skin, etc.
 */
class DensityProvider
{
public:
    /*!
     * \return the approximate required density of a cube
     */
    virtual float operator()(const AABB3D& aabb) const = 0;
    virtual ~DensityProvider()
    {
    };
};

} // namespace cura


#endif // INFILL_DENSITY_PROVIDER_H
