// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include "geometry/Point3D.h"

/*! Utility structure to allow storing an occupation in a boost::geometry::index::rtree. It has to be defined outside any scope to be properly registered. */
struct OccupiedPosition
{
    cura::Point3D position;
    uint8_t occupation;
};

BOOST_GEOMETRY_REGISTER_POINT_3D_CONST(OccupiedPosition, double, boost::geometry::cs::cartesian, position.x_, position.y_, position.z_)
