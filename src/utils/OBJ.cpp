// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/OBJ.h"

#include <fstream>

#include <range/v3/algorithm/find.hpp>
#include <spdlog/spdlog.h>

#include "mesh.h"
#include "utils/Point3D.h"

namespace cura
{

OBJ::OBJ(std::string filename, const double scale)
    : filename_(filename)
    , scale_(scale)
{
}

OBJ::~OBJ()
{
    std::string material_filename = fmt::format("{}.mtl", filename_);
    std::ofstream out(filename_);
    std::set<SVG::Color> used_colors;
    out << "mltlib " << material_filename << std::endl;
    out << std::fixed << std::setprecision(9);

    for (const Point3D& vertex : vertices_)
    {
        out << "v " << vertex.x_ << " " << vertex.y_ << " " << vertex.z_ << std::endl;
    }

    for (const Point2F& uv_coordinate : uv_coordinates_)
    {
        out << "vt " << uv_coordinate.x_ << " " << uv_coordinate.y_ << std::endl;
    }

    const auto export_vertex = [](const size_t vertex_id, const std::optional<size_t>& uv_coordinate)
    {
        return uv_coordinate.has_value() ? fmt::format("{}/{}", vertex_id + 1, uv_coordinate.value() + 1) : std::to_string(vertex_id + 1);
    };

    std::optional<SVG::Color> current_color;
    for (const Triangle& tri : triangles_)
    {
        if (tri.color != current_color)
        {
            out << "usemtl " << materialName(tri.color) << std::endl;
            current_color = tri.color;
            used_colors.insert(tri.color);
        }

        // OBJ indices are 1-based
        out << "f " << export_vertex(tri.p0, tri.uv0) << " " << export_vertex(tri.p1, tri.uv1) << " " << export_vertex(tri.p2, tri.uv2) << std::endl;
    }

    std::ofstream out_material(material_filename);
    for (SVG::Color color : used_colors)
    {
        out_material << "newmtl " << materialName(color) << std::endl;

        SVG::ColorObject color_rgb = SVG::ColorObject::toRgb(color);
        out_material << "Kd " << color_rgb.r_ << color_rgb.g_ << color_rgb.b_ << std::endl << std::endl;
    }
}

void OBJ::writeSphere(const Point3D& position, const double radius, const SVG::Color color)
{
    constexpr size_t latitude_segments = 4; // Number of latitude segments
    constexpr size_t longitude_segments = 8; // Number of longitude segments

    std::vector<std::vector<size_t>> vertex_indices(latitude_segments + 1);
    for (size_t i = 0; i <= latitude_segments; ++i)
    {
        const double theta = std::numbers::pi * i / latitude_segments;
        const double sin_theta = std::sin(theta);
        const double cos_theta = std::cos(theta);
        vertex_indices[i].resize(longitude_segments + 1);
        for (size_t j = 0; j <= longitude_segments; ++j)
        {
            const double phi = 2.0 * std::numbers::pi * j / longitude_segments;
            const double sin_phi = std::sin(phi);
            const double cos_phi = std::cos(phi);
            const double x = radius * sin_theta * cos_phi;
            const double y = radius * sin_theta * sin_phi;
            const double z = radius * cos_theta;
            vertex_indices[i][j] = insertVertex(Point3D(x, y, z) + position);
        }
    }

    for (size_t i = 0; i < latitude_segments; ++i)
    {
        for (size_t j = 0; j < longitude_segments; ++j)
        {
            const size_t v00 = vertex_indices[i][j];
            const size_t v01 = vertex_indices[i][j + 1];
            const size_t v10 = vertex_indices[i + 1][j];
            const size_t v11 = vertex_indices[i + 1][j + 1];

            if (i != 0)
            {
                triangles_.push_back(Triangle{ v00, v11, v01, color });
            }
            if (i != latitude_segments - 1)
            {
                triangles_.push_back(Triangle{ v00, v10, v11, color });
            }
        }
    }
}

void OBJ::writeTriangle(
    const Point3D& p0,
    const Point3D& p1,
    const Point3D& p2,
    const SVG::Color color,
    const std::optional<Point2F>& uv0,
    const std::optional<Point2F>& uv1,
    const std::optional<Point2F>& uv2)
{
    triangles_.push_back(Triangle{ insertVertex(p0), insertVertex(p1), insertVertex(p2), color, insertUVCoordinate(uv0), insertUVCoordinate(uv1), insertUVCoordinate(uv2) });
}

void OBJ::writeMesh(const Mesh& mesh, const SVG::Color color)
{
    for (const MeshFace& face : mesh.faces_)
    {
        writeTriangle(
            Point3D(mesh.vertices_[face.vertex_index_[0]].p_),
            Point3D(mesh.vertices_[face.vertex_index_[1]].p_),
            Point3D(mesh.vertices_[face.vertex_index_[2]].p_),
            color,
            face.uv_coordinates_[0],
            face.uv_coordinates_[1],
            face.uv_coordinates_[2]);
    }
}

Point3D OBJ::scalePosition(const Point3D& p) const
{
    return p * scale_;
}

size_t OBJ::insertVertex(const Point3D& p)
{
    const Point3D scaled_p = scalePosition(p);

    const auto iterator = ranges::find(vertices_, scaled_p);
    if (iterator == vertices_.end())
    {
        vertices_.push_back(scaled_p);
        return vertices_.size() - 1;
    }

    return std::distance(vertices_.begin(), iterator);
}

std::optional<size_t> OBJ::insertUVCoordinate(const std::optional<Point2F>& p)
{
    if (! p.has_value())
    {
        return std::nullopt;
    }

    const auto iterator = ranges::find(uv_coordinates_, p.value());
    if (iterator == uv_coordinates_.end())
    {
        uv_coordinates_.push_back(p.value());
        return uv_coordinates_.size() - 1;
    }

    return std::distance(uv_coordinates_.begin(), iterator);
}

std::string OBJ::materialName(const SVG::Color color) const
{
    return fmt::format("material_{}", SVG::toString(color));
}

} // namespace cura
