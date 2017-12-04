/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_UNIFORM_DENSITY_PROVIDER_H
#define INFILL_UNIFORM_DENSITY_PROVIDER_H

#include "../utils/intpoint.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"
#include "SierpinskiFillEdge.h"

namespace cura
{

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

    virtual float operator()(const SierpinskiFillEdge&, const SierpinskiFillEdge&) const
    {
        return density;
    };
protected:
    float density;
};

} // namespace cura


#endif // INFILL_UNIFORM_DENSITY_PROVIDER_H
