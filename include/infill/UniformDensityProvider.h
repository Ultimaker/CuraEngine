// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "DensityProvider.h"

namespace cura
{

struct AABB3D;

class UniformDensityProvider : public DensityProvider
{
public:
    UniformDensityProvider(double density)
        : density_(density)
    {
    }

    virtual ~UniformDensityProvider()
    {
    }

    virtual double operator()(const AABB3D&) const
    {
        return density_;
    }

protected:
    double density_;
};

} // namespace cura
