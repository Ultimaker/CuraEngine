/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "CombinedDensityProvider.h"

namespace cura {

CombinedDensityProvider::CombinedDensityProvider(const DensityProvider& average_density_provider, const DensityProvider& extremal_density_provider)
: average_density_provider(average_density_provider)
, extremal_density_provider(extremal_density_provider)
{
}

CombinedDensityProvider::~CombinedDensityProvider()
{
}

float CombinedDensityProvider::operator()(const AABB3D& aabb, const int_fast8_t averaging_statistic) const
{
    if (averaging_statistic == 0)
    {
        return average_density_provider(aabb, averaging_statistic);
    }
    else
    {
        return extremal_density_provider(aabb, averaging_statistic);
    }
}

}; // namespace cura
