// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/MeshUtils.h"

#include <optional>

#include "utils/Point2F.h"
#include "utils/Point3D.h"


namespace cura::MeshUtils
{

std::optional<Point3D> getBarycentricCoordinates(const Point3D& point, const Triangle3D& triangle)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const Point3D v0(triangle[1] - triangle[0]);
    const Point3D v1(triangle[2] - triangle[0]);
    const Point3D v2(point - triangle[0]);

    // Compute dot products
    const double d00 = v0 * v0;
    const double d01 = v0 * v1;
    const double d11 = v1 * v1;
    const double d20 = v2 * v0;
    const double d21 = v2 * v1;

    // Calculate denominator for barycentric coordinates
    const double denom = d00 * d11 - d01 * d01;

    // Check if triangle is degenerate
    constexpr double epsilon_triangle_cross_products = 0.000001;
    if (std::abs(denom) < epsilon_triangle_cross_products)
    {
        return std::nullopt;
    }

    // Calculate barycentric coordinates
    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;

    // Return as a Point3D where x/y/z represent the barycentric coordinates u/v/w
    return Point3D(u, v, w);
}

Point2F getUVCoordinates(const Point3D& barycentric_coordinates, const Triangle2F& uv_coordinates)
{
    return Point2F(
        (uv_coordinates[0].x_ * barycentric_coordinates.x_) + (uv_coordinates[1].x_ * barycentric_coordinates.y_) + (uv_coordinates[2].x_ * barycentric_coordinates.z_),
        (uv_coordinates[0].y_ * barycentric_coordinates.x_) + (uv_coordinates[1].y_ * barycentric_coordinates.y_) + (uv_coordinates[2].y_ * barycentric_coordinates.z_));
}

} // namespace cura::MeshUtils
