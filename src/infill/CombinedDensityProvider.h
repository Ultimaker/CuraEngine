//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_COMBINED_DENSITY_PROVIDER_H
#define INFILL_COMBINED_DENSITY_PROVIDER_H

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"

namespace cura
{

class CombinedDensityProvider : public DensityProvider
{
public:
    CombinedDensityProvider(const DensityProvider& average_density_provider, const DensityProvider& extremal_density_provider);

    virtual ~CombinedDensityProvider();

    virtual float operator()(const AABB3D& aabb, const int_fast8_t averaging_statistic) const;

protected:
    const DensityProvider& average_density_provider;
    const DensityProvider& extremal_density_provider;

};

} // namespace cura


#endif // INFILL_COMBINED_DENSITY_PROVIDER_H
