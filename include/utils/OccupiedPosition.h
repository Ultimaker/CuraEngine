// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include "geometry/Point3D.h"

namespace curaengine::utils
{

/*! Utility structure to allow storing an occupation in a boost::geometry::index::rtree. */
struct OccupiedPosition
{
    ::curaengine::geometry::Point3D position;
    uint8_t occupation;
};

} // namespace curaengine::utils

BOOST_GEOMETRY_REGISTER_POINT_3D_CONST(::curaengine::utils::OccupiedPosition, double, boost::geometry::cs::cartesian, position.x_, position.y_, position.z_)
