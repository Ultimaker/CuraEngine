// Copyright (c) 2026 Tim Kuipers
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine::utils
{
struct AABB3D;
}

namespace curaengine::infill
{



/*!
 * Parent class of function objects which return the density required for a given region.
 *
 * This density requirement can be based on user input, distance to the 3d model shell, Z distance to top skin, etc.
 */
class DensityProvider
{
public:
    /*!
     * \return the approximate required density of a cube
     */
    virtual double operator()(const utils::AABB3D& aabb) const = 0;
    virtual ~DensityProvider()
    {
    }
};

} // namespace curaengine::infill
