/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "CompensatedDensityProvider.h"

namespace cura {

CompensatedDensityProvider::CompensatedDensityProvider(const DensityProvider& density_provider, const std::function<float (float)>& mapping_function)
: density_provider(density_provider)
, mapping_function(mapping_function)
{
}

CompensatedDensityProvider::~CompensatedDensityProvider()
{
}

float CompensatedDensityProvider::operator()(const AABB3D& aabb, const int_fast8_t averaging_statistic) const
{
    mapping_function(density_provider(aabb, averaging_statistic));
}

}; // namespace cura
