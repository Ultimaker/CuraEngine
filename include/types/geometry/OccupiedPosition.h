// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include "types/geometry/Point3D.h"

namespace curaengine::types::geometry
{

/*! Utility structure to allow storing an occupation in a boost::geometry::index::rtree. */
struct OccupiedPosition
{
    types::geometry::Point3D position;
    uint8_t occupation;
};

} // namespace curaengine::types::geometry

BOOST_GEOMETRY_REGISTER_POINT_3D_CONST(::curaengine::types::geometry::OccupiedPosition, double, boost::geometry::cs::cartesian, position.x_, position.y_, position.z_)
