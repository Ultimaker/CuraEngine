// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "DensityProvider.h"

namespace curaengine::types::geometry
{
struct AABB3D;
}

namespace curaengine::infill
{


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

    virtual double operator()(const types::geometry::AABB3D&) const
    {
        return density_;
    }

protected:
    double density_;
};

} // namespace curaengine::infill
