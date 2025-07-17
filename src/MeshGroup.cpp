// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "MeshGroup.h"

#include <fstream>
#include <limits>
#include <regex>
#include <stdio.h>
#include <string.h>

#include <fmt/format.h>
#include <range/v3/view/enumerate.hpp>
#include <scripta/logger.h>
#include <spdlog/spdlog.h>

#include "settings/types/Ratio.h" //For the shrinkage percentage and scale factor.
#include "utils/Matrix4x3D.h" //To transform the input meshes for shrinkage compensation and to align in command line mode.
#include "utils/Point3F.h" //To accept incoming meshes with floating point vertices.
#include "utils/gettime.h"
#include "utils/section_type.h"
#include "utils/string.h"

namespace cura
{

FILE* binaryMeshBlob = nullptr;

/* Custom fgets function to support Mac line-ends in Ascii STL files. OpenSCAD produces this when used on Mac */
void* fgets_(char* ptr, size_t len, FILE* f)
{
    while (len && fread(ptr, 1, 1, f) > 0)
    {
        if (*ptr == '\n' || *ptr == '\r')
        {
            *ptr = '\0';
            return ptr;
        }
        ptr++;
        len--;
    }
    return nullptr;
}

Point3LL MeshGroup::min() const
{
    if (meshes.size() < 1)
    {
        return Point3LL(0, 0, 0);
    }
    Point3LL ret(std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max());
    for (const Mesh& mesh : meshes)
    {
        if (mesh.settings_.get<bool>("infill_mesh") || mesh.settings_.get<bool>("cutting_mesh")
            || mesh.settings_.get<bool>("anti_overhang_mesh")) // Don't count pieces that are not printed.
        {
            continue;
        }
        Point3LL v = mesh.min();
        ret.x_ = std::min(ret.x_, v.x_);
        ret.y_ = std::min(ret.y_, v.y_);
        ret.z_ = std::min(ret.z_, v.z_);
    }
    return ret;
}

Point3LL MeshGroup::max() const
{
    if (meshes.size() < 1)
    {
        return Point3LL(0, 0, 0);
    }
    Point3LL ret(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min());
    for (const Mesh& mesh : meshes)
    {
        if (mesh.settings_.get<bool>("infill_mesh") || mesh.settings_.get<bool>("cutting_mesh")
            || mesh.settings_.get<bool>("anti_overhang_mesh")) // Don't count pieces that are not printed.
        {
            continue;
        }
        Point3LL v = mesh.max();
        ret.x_ = std::max(ret.x_, v.x_);
        ret.y_ = std::max(ret.y_, v.y_);
        ret.z_ = std::max(ret.z_, v.z_);
    }
    return ret;
}

void MeshGroup::clear()
{
    for (Mesh& m : meshes)
    {
        m.clear();
    }
}

void MeshGroup::finalize()
{
    // If the machine settings have been supplied, offset the given position vertices to the center of vertices (0,0,0) is at the bed center.
    Point3LL meshgroup_offset(0, 0, 0);
    if (! settings.get<bool>("machine_center_is_zero"))
    {
        meshgroup_offset.x_ = settings.get<coord_t>("machine_width") / 2;
        meshgroup_offset.y_ = settings.get<coord_t>("machine_depth") / 2;
    }

    // If a mesh position was given, put the mesh at this position in 3D space.
    for (Mesh& mesh : meshes)
    {
        Point3LL mesh_offset(mesh.settings_.get<coord_t>("mesh_position_x"), mesh.settings_.get<coord_t>("mesh_position_y"), mesh.settings_.get<coord_t>("mesh_position_z"));
        if (mesh.settings_.get<bool>("center_object"))
        {
            Point3LL object_min = mesh.min();
            Point3LL object_max = mesh.max();
            Point3LL object_size = object_max - object_min;
            mesh_offset += Point3LL(-object_min.x_ - object_size.x_ / 2, -object_min.y_ - object_size.y_ / 2, -object_min.z_);
        }
        mesh.translate(mesh_offset + meshgroup_offset);
    }
    scaleFromBottom(
        settings.get<Ratio>("material_shrinkage_percentage_xy"),
        settings.get<Ratio>("material_shrinkage_percentage_z")); // Compensate for the shrinkage of the material.
    for (const auto& [idx, mesh] : meshes | ranges::views::enumerate)
    {
        scripta::log(fmt::format("mesh_{}", idx), mesh, SectionType::NA);
    }
}

void MeshGroup::scaleFromBottom(const Ratio factor_xy, const Ratio factor_z)
{
    const Point3LL center = (max() + min()) / 2;
    const Point3LL origin(center.x_, center.y_, 0);

    const Matrix4x3D transformation = Matrix4x3D::scale(factor_xy, factor_xy, factor_z, origin);
    for (Mesh& mesh : meshes)
    {
        mesh.transform(transformation);
    }
}

bool loadMeshSTL_ascii(Mesh* mesh, const char* filename, const Matrix4x3D& matrix)
{
    FILE* f = fopen(filename, "rt");
    char buffer[1024];
    Point3F vertex;
    int n = 0;
    Point3LL v0(0, 0, 0), v1(0, 0, 0), v2(0, 0, 0);
    while (fgets_(buffer, sizeof(buffer), f))
    {
        if (sscanf(buffer, " vertex %f %f %f", &vertex.x_, &vertex.y_, &vertex.z_) == 3)
        {
            n++;
            switch (n)
            {
            case 1:
                v0 = matrix.apply(vertex.toPoint3d());
                break;
            case 2:
                v1 = matrix.apply(vertex.toPoint3d());
                break;
            case 3:
                v2 = matrix.apply(vertex.toPoint3d());
                mesh->addFace(v0, v1, v2);
                n = 0;
                break;
            }
        }
    }
    fclose(f);
    mesh->finish();
    return true;
}

bool loadMeshSTL_binary(Mesh* mesh, const char* filename, const Matrix4x3D& matrix)
{
    FILE* f = fopen(filename, "rb");

    fseek(f, 0L, SEEK_END);
    long long file_size = ftell(f); // The file size is the position of the cursor after seeking to the end.
    rewind(f); // Seek back to start.
    size_t face_count = (file_size - 80 - sizeof(uint32_t)) / 50; // Subtract the size of the header. Every face uses exactly 50 bytes.

    char buffer[80];
    // Skip the header
    if (fread(buffer, 80, 1, f) != 1)
    {
        fclose(f);
        return false;
    }

    uint32_t reported_face_count;
    // Read the face count. We'll use it as a sort of redundancy code to check for file corruption.
    if (fread(&reported_face_count, sizeof(uint32_t), 1, f) != 1)
    {
        fclose(f);
        return false;
    }
    if (reported_face_count != face_count)
    {
        spdlog::warn("Face count reported by file ({}) is not equal to actual face count ({}). File could be corrupt!", reported_face_count, face_count);
    }

    // For each face read:
    // float(x,y,z) = normal, float(X,Y,Z)*3 = vertexes, uint16_t = flags
    //  Every Face is 50 Bytes: Normal(3*float), Vertices(9*float), 2 Bytes Spacer
    mesh->faces_.reserve(face_count);
    mesh->vertices_.reserve(face_count);
    for (size_t i = 0; i < face_count; i++)
    {
        if (fread(buffer, 50, 1, f) != 1)
        {
            fclose(f);
            return false;
        }
        float* v = reinterpret_cast<float*>(buffer) + 3;

        Point3LL v0 = matrix.apply(Point3F(v[0], v[1], v[2]).toPoint3d());
        Point3LL v1 = matrix.apply(Point3F(v[3], v[4], v[5]).toPoint3d());
        Point3LL v2 = matrix.apply(Point3F(v[6], v[7], v[8]).toPoint3d());
        mesh->addFace(v0, v1, v2);
    }
    fclose(f);
    mesh->finish();
    return true;
}

bool loadMeshSTL(Mesh* mesh, const char* filename, const Matrix4x3D& matrix)
{
    FILE* f = fopen(filename, "rb");
    if (f == nullptr)
    {
        return false;
    }

    // assign filename to mesh_name
    mesh->mesh_name_ = filename;

    // Skip any whitespace at the beginning of the file.
    unsigned long long num_whitespace = 0; // Number of whitespace characters.
    unsigned char whitespace;
    if (fread(&whitespace, 1, 1, f) != 1)
    {
        fclose(f);
        return false;
    }
    while (isspace(whitespace))
    {
        num_whitespace++;
        if (fread(&whitespace, 1, 1, f) != 1)
        {
            fclose(f);
            return false;
        }
    }
    fseek(f, num_whitespace, SEEK_SET); // Seek to the place after all whitespace (we may have just read too far).

    char buffer[6];
    if (fread(buffer, 5, 1, f) != 1)
    {
        fclose(f);
        return false;
    }
    fclose(f);

    buffer[5] = '\0';
    if (stringcasecompare(buffer, "solid") == 0)
    {
        bool load_success = loadMeshSTL_ascii(mesh, filename, matrix);
        if (! load_success)
            return false;

        // This logic is used to handle the case where the file starts with
        // "solid" but is a binary file.
        if (mesh->faces_.size() < 1)
        {
            mesh->clear();
            return loadMeshSTL_binary(mesh, filename, matrix);
        }
        return true;
    }
    return loadMeshSTL_binary(mesh, filename, matrix);
}

bool loadMeshOBJ(Mesh* mesh, const std::string& filename, const Matrix4x3D& matrix)
{
    std::ifstream file(filename);
    if (! file.is_open())
    {
        spdlog::error("Could not open OBJ file: {}", filename);
        return false;
    }

    std::vector<Point3LL> vertices;
    std::vector<Point2F> uv_coordinates;
    std::string line;
    std::regex main_regex(R"((v|vt|f)\s+(.*))");
    std::regex vertex_regex(R"(([-+]?[0-9]*\.?[0-9]+)\s+([-+]?[0-9]*\.?[0-9]+)\s+([-+]?[0-9]*\.?[0-9]+))");
    std::regex uv_regex(R"(([-+]?[0-9]*\.?[0-9]+)\s+([-+]?[0-9]*\.?[0-9]+))");
    std::regex face_indices_regex(R"((\d+)(?:\/(\d*))?(?:\/(?:\d*))?)");

    auto get_uv_coordinates = [&uv_coordinates](std::optional<size_t> uv_index) -> std::optional<Point2F>
    {
        if (uv_index.has_value() && uv_index.value() < uv_coordinates.size())
        {
            return std::make_optional(uv_coordinates[uv_index.value()]);
        }
        return std::nullopt;
    };

    while (std::getline(file, line))
    {
        std::smatch matches;

        if (! std::regex_match(line, matches, main_regex))
        {
            // Unrecognized line, just skip
            continue;
        }

        const std::string line_identifier = matches[1].str();
        const std::string payload = matches[2].str();

        if (line_identifier == "v" && std::regex_match(payload, matches, vertex_regex))
        {
            const float x = std::stof(matches[1].str());
            const float y = std::stof(matches[2].str());
            const float z = std::stof(matches[3].str());
            vertices.push_back(matrix.apply(Point3D(x, y, z)));
        }
        else if (line_identifier == "vt" && std::regex_match(line, matches, uv_regex))
        {
            const float u = std::stof(matches[1].str());
            const float v = std::stof(matches[2].str());
            uv_coordinates.push_back(Point2F(u, v));
        }
        else if (line_identifier == "f")
        {
            struct Vertex
            {
                size_t index;
                std::optional<size_t> uv_index;
            };

            std::vector<Vertex> vertex_indices;
            std::sregex_iterator it(payload.begin(), payload.end(), face_indices_regex);
            std::sregex_iterator end;

            while (it != end)
            {
                std::smatch vertex_match = *it;
                if (vertex_match.size() >= 2)
                {
                    Vertex vertex;
                    vertex.index = std::stoul(vertex_match[1].str()) - 1;
                    if (vertex_match[2].matched && vertex_match[2].length() > 0)
                    {
                        vertex.uv_index = std::stoul(vertex_match[2].str()) - 1;
                    }
                    vertex_indices.push_back(vertex);
                }
                ++it;
            }

            // Triangulate the face
            if (vertex_indices.size() >= 3)
            {
                for (size_t i = 1; i < vertex_indices.size() - 1; ++i)
                {
                    const Vertex& v0 = vertex_indices[0];
                    const Vertex& v1 = vertex_indices[i];
                    const Vertex& v2 = vertex_indices[i + 1];

                    if (v0.index < vertices.size() && v1.index < vertices.size() && v2.index < vertices.size())
                    {
                        mesh->addFace(
                            vertices[v0.index],
                            vertices[v1.index],
                            vertices[v2.index],
                            get_uv_coordinates(v0.uv_index),
                            get_uv_coordinates(v1.uv_index),
                            get_uv_coordinates(v2.uv_index));
                    }
                }
            }
        }
    }

    mesh->finish();
    return ! mesh->faces_.empty();
}

bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const char* filename, const Matrix4x3D& transformation, Settings& object_parent_settings)
{
    TimeKeeper load_timer;

    const char* ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".stl") == 0 || strcmp(ext, ".STL") == 0))
    {
        Mesh mesh(object_parent_settings);
        if (loadMeshSTL(&mesh, filename, transformation)) // Load it! If successful...
        {
            meshgroup->meshes.push_back(mesh);
            spdlog::info("loading '{}' took {:03.3f} seconds", filename, load_timer.restart());
            return true;
        }
        spdlog::warn("loading '{}' failed", filename);
        return false;
    }

    if (ext && (strcmp(ext, ".obj") == 0 || strcmp(ext, ".OBJ") == 0))
    {
        Mesh mesh(object_parent_settings);
        if (loadMeshOBJ(&mesh, filename, transformation)) // Load it! If successful...
        {
            meshgroup->meshes.push_back(mesh);
            spdlog::info("loading '{}' took {:03.3f} seconds", filename, load_timer.restart());
            return true;
        }
        spdlog::warn("loading OBJ '{}' failed", filename);
        return false;
    }

    spdlog::warn("Unable to recognize the extension of the file. Currently only .stl and .STL are supported.");
    return false;
}

} // namespace cura
