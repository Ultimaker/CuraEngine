// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MESH_UTILS_H
#define UTILS_MESH_UTILS_H

#include <optional>

#include "geometry/Triangle2F.h"
#include "geometry/Triangle3D.h"

namespace cura
{

class Point3D;

namespace MeshUtils
{

std::optional<Point3D> getBarycentricCoordinates(const Point3D& point, const Triangle3D& triangle);

Point2F getUVCoordinates(const Point3D& barycentric_coordinates, const Triangle2F& uv_coordinates);

} // namespace MeshUtils

} // namespace cura

#endif
