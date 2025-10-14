// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/OBJ.h"

#include <fstream>
#include <unordered_set>

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

    std::vector<Point3D> ordered_vertices;
    std::map<Point3D, size_t> vertices_ids;
    std::vector<Point2F> ordered_uv_coordinates;
    std::map<Point2F, size_t> uv_coordinates_ids;

    for (const Triangle& triangle : triangles_)
    {
        for (const Point3D& vertex : { triangle.p0, triangle.p1, triangle.p2 })
        {
            if (! vertices_ids.contains(vertex))
            {
                vertices_ids[vertex] = ordered_vertices.size();
                ordered_vertices.push_back(vertex);
            }
        }
        for (const std::optional<Point2F>& uv : { triangle.uv0, triangle.uv1, triangle.uv2 })
        {
            if (uv.has_value())
            {
                if (! uv_coordinates_ids.contains(uv.value()))
                {
                    uv_coordinates_ids[uv.value()] = ordered_uv_coordinates.size();
                    ordered_uv_coordinates.push_back(uv.value());
                }
            }
        }
    }

    for (const Point3D& vertex : ordered_vertices)
    {
        out << "v " << vertex.x_ << " " << vertex.y_ << " " << vertex.z_ << "\n";
    }

    for (const Point2F& uv_coordinate : ordered_uv_coordinates)
    {
        out << "vt " << uv_coordinate.x_ << " " << uv_coordinate.y_ << "\n";
    }

    const auto export_vertex = [&vertices_ids, &uv_coordinates_ids](const Point3D& vertex, const std::optional<Point2F>& uv_coordinate)
    {
        const size_t vertex_id = vertices_ids.find(vertex)->second;
        const size_t uv_id = uv_coordinate.has_value() ? uv_coordinates_ids.find(uv_coordinate.value())->second : 0;

        return uv_coordinate.has_value() ? fmt::format("{}/{}", vertex_id + 1, uv_id + 1) : std::to_string(vertex_id + 1);
    };

    std::optional<SVG::Color> current_color;
    for (const Triangle& tri : triangles_)
    {
        if (tri.color != current_color)
        {
            out << "usemtl " << materialName(tri.color) << "\n";
            current_color = tri.color;
            used_colors.insert(tri.color);
        }

        // OBJ indices are 1-based
        out << "f " << export_vertex(tri.p0, tri.uv0) << " " << export_vertex(tri.p1, tri.uv1) << " " << export_vertex(tri.p2, tri.uv2) << "\n";
    }

    std::ofstream out_material(material_filename);
    for (SVG::Color color : used_colors)
    {
        out_material << "newmtl " << materialName(color) << "\n";

        SVG::ColorObject color_rgb = SVG::ColorObject::toRgb(color);
        out_material << "Kd " << color_rgb.r_ << color_rgb.g_ << color_rgb.b_ << "\n"
                     << "\n";
    }
}

void OBJ::writeSphere(const Point3D& position, const double radius, const SVG::Color color, const size_t latitude_segments, const size_t longitude_segments)
{
    std::vector<std::vector<Point3D>> vertices(latitude_segments + 1);
    for (size_t i = 0; i <= latitude_segments; ++i)
    {
        const double theta = std::numbers::pi * i / latitude_segments;
        const double sin_theta = std::sin(theta);
        const double cos_theta = std::cos(theta);
        vertices[i].resize(longitude_segments + 1);
        for (size_t j = 0; j <= longitude_segments; ++j)
        {
            const double phi = 2.0 * std::numbers::pi * j / longitude_segments;
            const double sin_phi = std::sin(phi);
            const double cos_phi = std::cos(phi);
            const double x = radius * sin_theta * cos_phi;
            const double y = radius * sin_theta * sin_phi;
            const double z = radius * cos_theta;
            vertices[i][j] = Point3D(x, y, z) + position;
        }
    }

    for (size_t i = 0; i < latitude_segments; ++i)
    {
        for (size_t j = 0; j < longitude_segments; ++j)
        {
            const Point3D& v00 = vertices[i][j];
            const Point3D& v01 = vertices[i][j + 1];
            const Point3D& v10 = vertices[i + 1][j];
            const Point3D& v11 = vertices[i + 1][j + 1];

            if (i != 0)
            {
                writeTriangle(v00, v11, v01, color);
            }
            if (i != latitude_segments - 1)
            {
                writeTriangle(v00, v10, v11, color);
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
    triangles_.push_back(Triangle{ scalePosition(p0), scalePosition(p1), scalePosition(p2), color, uv0, uv1, uv2 });
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

std::string OBJ::materialName(const SVG::Color color) const
{
    return fmt::format("material_{}", SVG::toString(color));
}

} // namespace cura
