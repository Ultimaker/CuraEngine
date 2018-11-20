//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_COMPENSATED_DENSITY_PROVIDER_H
#define INFILL_COMPENSATED_DENSITY_PROVIDER_H

#include <functional>

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"

namespace cura
{

class CompensatedDensityProvider : public DensityProvider
{
public:
    CompensatedDensityProvider(const DensityProvider& density_provider, const std::function<float (float)>& mapping_function);

    virtual ~CompensatedDensityProvider();

    virtual float operator()(const AABB3D& aabb, const int_fast8_t averaging_statistic) const;

protected:
    const DensityProvider& density_provider;
    const std::function<float (float)>& mapping_function;

};

} // namespace cura


#endif // INFILL_COMPENSATED_DENSITY_PROVIDER_H
