/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_DENSITY_PROVIDER_H
#define INFILL_DENSITY_PROVIDER_H


#include "SierpinskiFillEdge.h"

namespace cura
{

class DensityProvider
{
public:
    virtual float operator()(const SierpinskiFillEdge& e1, const SierpinskiFillEdge& e2) const = 0;
    virtual ~DensityProvider()
    {
    };
};

} // namespace cura


#endif // INFILL_DENSITY_PROVIDER_H
