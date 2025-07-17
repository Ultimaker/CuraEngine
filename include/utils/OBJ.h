// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef OBJ_H
#define OBJ_H

#include <optional>

#include "NoCopy.h"
#include "SVG.h"
#include "utils/Point2F.h"

namespace cura
{

class Mesh;

class OBJ : NoCopy
{
public:
    OBJ(std::string filename, const double scale = 1.0);

    ~OBJ();

    void writeSphere(const Point3D& position, const double radius = 1.0, const SVG::Color color = SVG::Color::BLACK);

    void writeTriangle(
        const Point3D& p0,
        const Point3D& p1,
        const Point3D& p2,
        const SVG::Color color = SVG::Color::BLACK,
        const std::optional<Point2F>& uv0 = std::nullopt,
        const std::optional<Point2F>& uv1 = std::nullopt,
        const std::optional<Point2F>& uv2 = std::nullopt);

    void writeMesh(const Mesh& mesh, const SVG::Color color = SVG::Color::BLACK);

private:
    struct Triangle
    {
        size_t p0, p1, p2;
        SVG::Color color;
        std::optional<size_t> uv0, uv1, uv2;
    };

    const std::string filename_;
    const double scale_;
    std::vector<Point3D> vertices_;
    std::vector<Triangle> triangles_;
    std::vector<Point2F> uv_coordinates_;

    Point3D scalePosition(const Point3D& p) const;

    size_t insertVertex(const Point3D& p);

    std::optional<size_t> insertUVCoordinate(const std::optional<Point2F>& p);

    std::string materialName(const SVG::Color color) const;
};

} // namespace cura
#endif // SVG_H
