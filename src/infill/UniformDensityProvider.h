//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_UNIFORM_DENSITY_PROVIDER_H
#define INFILL_UNIFORM_DENSITY_PROVIDER_H

#include "DensityProvider.h"

namespace cura
{

struct AABB3D;

class UniformDensityProvider : public DensityProvider
{
public:
    UniformDensityProvider(float density)
    : density(density)
    {
    };

    virtual ~UniformDensityProvider()
    {
    };

    virtual float operator()(const AABB3D&) const
    {
        return density;
    };
protected:
    float density;
};

} // namespace cura


#endif // INFILL_UNIFORM_DENSITY_PROVIDER_H
