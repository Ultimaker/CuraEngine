/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_POLYGON_BASED_DENSITY_PROVIDER_H
#define INFILL_POLYGON_BASED_DENSITY_PROVIDER_H

#include <functional>

#include "../utils/IntPoint.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"


namespace cura
{

class SliceLayer; // forward decl

class PolygonBasedDensityProvider : public DensityProvider
{
public:
    /*!
     * density provider based on gradual infill regions
     */
    PolygonBasedDensityProvider(const SliceLayer& layer, float most_dense_density);

    virtual ~PolygonBasedDensityProvider()
    {
    };

    virtual float operator()(const AABB& aabb) const;

protected:
    //! Helper class for coupling regions to their density
    struct PolygonDensity
    {
        const Polygons* area;
        float density;
        PolygonDensity(const Polygons* area, float density)
        : area(area)
        , density(density)
        {
        }
    };

    std::list<PolygonDensity> polygon_densities; //!< Ordered from most priority to least: only if a point is not in the first 5 polygons will it consider the density of the 6th.
    float fallback_density; //!< The density to return if a point is in no polygon.

};

} // namespace cura


#endif // INFILL_POLYGON_BASED_DENSITY_PROVIDER_H
