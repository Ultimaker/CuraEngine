// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "NoCopy.h"
#include "SVG.h"
#include "geometry/Point2F.h"
#include "geometry/Point3D.h"

namespace curaengine
{
class Mesh;
}

namespace curaengine::utils
{


class VoxelGrid;

class OBJ : NoCopy
{
public:
    OBJ(const std::string& filename, const double scale = 1.0);

    ~OBJ();

    void writeSphere(
        const geometry::Point3D& position,
        const double radius = 1.0,
        const SVG::Color color = SVG::Color::BLACK,
        const size_t latitude_segments = 4,
        const size_t longitude_segments = 8);

    void writeTriangle(
        const geometry::Point3D& p0,
        const geometry::Point3D& p1,
        const geometry::Point3D& p2,
        const SVG::Color color = SVG::Color::BLACK,
        const std::optional<geometry::Point2F>& uv0 = std::nullopt,
        const std::optional<geometry::Point2F>& uv1 = std::nullopt,
        const std::optional<geometry::Point2F>& uv2 = std::nullopt);

    void write(const Mesh& mesh, const SVG::Color color = SVG::Color::BLACK);

    void write(const VoxelGrid& voxel_grid);

    void write(const geometry::Polyline& polyline, const coord_t z, const coord_t height, const SVG::Color color = SVG::Color::BLACK);

    void write(const geometry::Shape& shape, const coord_t z, const coord_t height, const SVG::Color color = SVG::Color::BLACK);

private:
    struct Triangle
    {
        geometry::Point3D p0, p1, p2;
        SVG::Color color;
        std::optional<geometry::Point2F> uv0, uv1, uv2;
    };

    const std::string filename_;
    const double scale_;
    std::vector<Triangle> triangles_;

    geometry::Point3D scalePosition(const geometry::Point3D& p) const;

    size_t insertVertex(const geometry::Point3D& p);

    std::optional<size_t> insertUVCoordinate(const std::optional<geometry::Point2F>& p);

    std::string materialName(const SVG::Color color) const;
};

} // namespace curaengine::utils
