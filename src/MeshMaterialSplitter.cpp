// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

// #include <CGAL/AABB_face_graph_triangle_primitive.h>
// #include <CGAL/AABB_traits_3.h>
// #include <CGAL/Boolean_set_operations_2.h>
// #include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
// #include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Surface_mesh.h>
// #include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/alpha_wrap_3.h>

#include <spdlog/spdlog.h>

#include "MeshGroup.h"
#include "Slice.h"
#include "geometry/Shape.h"
#include "mesh.h"
#include "png++/image.hpp"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"


namespace cura::MeshMaterialSplitter
{

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Triangle_2 = Kernel::Triangle_2;
using Delaunay = CGAL::Delaunay_triangulation_3<Kernel>;
using PolygonMesh = CGAL::Surface_mesh<Kernel::Point_3>;
// using Polygon_2 = CGAL::Polygon_2<Kernel>;
// using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<Kernel>;
// using Triangulator_2 = CGAL::Polygon_triangulation_decomposition_2<Kernel>;

void exportMesh(const PolygonMesh& mesh, const std::string& filename)
{
    PolygonMesh exported_mesh = mesh;

    std::ofstream out(fmt::format("/home/erwan/test/CURA-12449_handling-painted-models/{}.obj", filename));
    for (CGAL::SM_Vertex_index vertex : vertices(exported_mesh))
    {
        Point_3& p = exported_mesh.point(vertex);
        p = Point_3(p.x() * 0.001, p.y() * 0.001, p.z() * 0.001);
    }
    CGAL::IO::write_OBJ(out, exported_mesh);
}

void exportPointsCloud(const std::vector<Point_3>& points_cloud, const std::string& filename)
{
    PolygonMesh exported_mesh;
    for (const Point_3& point : points_cloud)
    {
        exported_mesh.add_vertex(point);
    }
    exportMesh(exported_mesh, filename);
}

Point2F getPixelCoordinates(const Point2F& uv_coordinates, const png::image<png::rgb_pixel>& image)
{
    const uint32_t width = image.get_width();
    const uint32_t height = image.get_height();
    return Point2F(
        std::clamp(static_cast<uint32_t>(uv_coordinates.x_ * width), static_cast<uint32_t>(0), width - 1),
        std::clamp(static_cast<uint32_t>(height - uv_coordinates.y_ * height), static_cast<uint32_t>(0), height - 1));
}

#if 0
Point3LL getSpaceCoordinates(const Point_2& pixel_coordinates, const Point2F triangle_coordinates[3], const Mesh& mesh, const MeshFace& face)
{
    // Calculate 3D space coordinates from pixel coordinates using barycentric coordinates
    const Point2F p0 = triangle_coordinates[0];
    const Point2F p1 = triangle_coordinates[1];
    const Point2F p2 = triangle_coordinates[2];

    // Calculate barycentric coordinates
    const float area = 0.5f * ((p1.x_ - p0.x_) * (p2.y_ - p0.y_) - (p2.x_ - p0.x_) * (p1.y_ - p0.y_));
    const float u = 0.5f * ((p1.x_ - pixel_coordinates.x()) * (p2.y_ - pixel_coordinates.y()) - (p2.x_ - pixel_coordinates.x()) * (p1.y_ - pixel_coordinates.y())) / area;
    const float v = 0.5f * ((pixel_coordinates.x() - p0.x_) * (p2.y_ - p0.y_) - (p2.x_ - p0.x_) * (pixel_coordinates.y() - p0.y_)) / area;
    const float w = 1.0f - u - v;

    // Apply barycentric coordinates to get 3D position
    const Point3LL& v0 = mesh.vertices_[face.vertex_index_[0]].p_;
    const Point3LL& v1 = mesh.vertices_[face.vertex_index_[1]].p_;
    const Point3LL& v2 = mesh.vertices_[face.vertex_index_[2]].p_;

    return Point3LL(u * v0.x_ + v * v1.x_ + w * v2.x_, u * v0.y_ + v * v1.y_ + w * v2.y_, u * v0.z_ + v * v1.z_ + w * v2.z_);
}
#endif

#if 0
double getTriangleArea(const Point_3& p0, const Point_3& p1, const Point_3& p2)
{
    const Kernel::Vector_3 edge_0(p1 - p0);
    const Kernel::Vector_3 edge_1(p2 - p0);

    return std::sqrt(CGAL::cross_product(edge_0, edge_1).squared_length()) / 2.0;
}
#endif

std::optional<Point_3> getBarycentricCoordinates(const Point_3& point, const Point_3& p0, const Point_3& p1, const Point_3& p2)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const Kernel::Vector_3 v0 = p1 - p0;
    const Kernel::Vector_3 v1 = p2 - p0;
    const Kernel::Vector_3 v2 = point - p0;

    // Compute dot products
    const double d00 = v0 * v0;
    const double d01 = v0 * v1;
    const double d11 = v1 * v1;
    const double d20 = v2 * v0;
    const double d21 = v2 * v1;

    // Calculate denominator for barycentric coordinates
    const double denom = d00 * d11 - d01 * d01;

    // Check if triangle is degenerate
    if (std::abs(denom) < 0.000001)
    {
        return std::nullopt;
    }

    // Calculate barycentric coordinates
    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;

    return Point_3(u, v, w);
}
#if 0
void splitFaceToTexture(const Mesh& mesh, const MeshFace& face, const png::image<png::rgb_pixel>& image, Mesh& output_mesh)
{
    // First, convert UV coordinates of the 3 points to pixel coordinates on the image
    Point2F pixel_coordinates[3];
    bool all_uv_coordinates = true;
    bool face_split = false;

    for (size_t i = 0; i < 3 && all_uv_coordinates; ++i)
    {
        if (face.uv_coordinates_[i].has_value())
        {
            pixel_coordinates[i] = getPixelCoordinates(face.uv_coordinates_[i].value(), image);
        }
        else
        {
            all_uv_coordinates = false;
        }
    }

    if (all_uv_coordinates)
    {
        // Now get the bounding box of the triangle on the image
        Point2F min;
        min.x_ = std::min({ pixel_coordinates[0].x_, pixel_coordinates[1].x_, pixel_coordinates[2].x_ });
        min.y_ = std::min({ pixel_coordinates[0].y_, pixel_coordinates[1].y_, pixel_coordinates[2].y_ });

        Point2F max;
        max.x_ = std::max({ pixel_coordinates[0].x_, pixel_coordinates[1].x_, pixel_coordinates[2].x_ });
        max.y_ = std::max({ pixel_coordinates[0].y_, pixel_coordinates[1].y_, pixel_coordinates[2].y_ });

        Point2LL min_pixel(min.x_, min.y_);
        Point2LL max_pixel(max.x_, max.y_);

        // Generate polygons for all pixel triangles in the bounding box

        const Triangle_2 face_triangle(
            Point_2(pixel_coordinates[0].x_, pixel_coordinates[0].y_),
            Point_2(pixel_coordinates[1].x_, pixel_coordinates[1].y_),
            Point_2(pixel_coordinates[2].x_, pixel_coordinates[2].y_));

        std::vector<Triangle_2> triangles;
        Shape shape;
        for (coord_t x = min_pixel.X; x < max_pixel.X; ++x)
        {
            for (coord_t y = min_pixel.Y; y < max_pixel.Y; ++y)
            {
                const Triangle_2 triangle_1(Point_2(x, y), Point_2(x + 1, y), Point_2(x, y + 1));
                const Triangle_2 triangle_2(Point_2(x + 1, y), Point_2(x + 1, y + 1), Point_2(x, y + 1));

                for (const Triangle_2& triangle : { triangle_1, triangle_2 })
                {
                    std::list<Triangle_2> split_triangle_parts;
                    auto intersect = CGAL::intersection(triangle, face_triangle);

                    if (intersect)
                    {
                        if (const Triangle_2* triangle_result = std::get_if<Triangle_2>(&*intersect))
                        {
                            output_mesh.addFace(
                                getSpaceCoordinates((*triangle_result)[0], pixel_coordinates, mesh, face),
                                getSpaceCoordinates((*triangle_result)[1], pixel_coordinates, mesh, face),
                                getSpaceCoordinates((*triangle_result)[2], pixel_coordinates, mesh, face),
                                face.uv_coordinates_[0],
                                face.uv_coordinates_[1],
                                face.uv_coordinates_[2]);
                        }
                        else if (const std::vector<Point_2>* points_result = std::get_if<std::vector<Point_2>>(&*intersect))
                        {
                            // Convert points to a polygon
                            Polygon_2 polygon;
                            for (const auto& point : *points_result)
                            {
                                polygon.push_back(point);
                            }

                            // Triangulate the polygon
                            std::list<Polygon_2> triangles;
                            Triangulator_2 triangulator;
                            triangulator(polygon, std::back_inserter(triangles));

                            // Add each triangulated part to the mesh
                            for (const auto& triangle : triangles)
                            {
                                output_mesh.addFace(
                                    getSpaceCoordinates(triangle[0], pixel_coordinates, mesh, face),
                                    getSpaceCoordinates(triangle[1], pixel_coordinates, mesh, face),
                                    getSpaceCoordinates(triangle[2], pixel_coordinates, mesh, face),
                                    face.uv_coordinates_[0],
                                    face.uv_coordinates_[1],
                                    face.uv_coordinates_[2]);
                            }
                        }
                        else
                        {
                            spdlog::info("The result is something unexpected ?!");
                        }
                    }

                    {
                        // const Polygon_2& split_triangle_outer = split_triangle_part.outer_boundary();
                        // if (split_triangle_outer.size() == 3)
                        // {
                        //     output_mesh.addFace(
                        //         getSpaceCoordinates(split_triangle_outer[0], pixel_coordinates, mesh, face),
                        //         getSpaceCoordinates(split_triangle_outer[1], pixel_coordinates, mesh, face),
                        //         getSpaceCoordinates(split_triangle_outer[2], pixel_coordinates, mesh, face),
                        //         face.uv_coordinates_[0],
                        //         face.uv_coordinates_[1],
                        //         face.uv_coordinates_[2]);
                        // }
                        // else if (split_triangle_outer.size() == 4)
                        // {
                        // }
                        // More than 4 should not happen as we are intersecting a triangle with a triangle
                    }
                }
            }
        }

        face_split = true;
    }

    if (! face_split)
    {
        // Face didn't require splitting, just add it back to the output mesh
        output_mesh.addFace(
            mesh.vertices_[face.vertex_index_[0]].p_,
            mesh.vertices_[face.vertex_index_[1]].p_,
            mesh.vertices_[face.vertex_index_[2]].p_,
            face.uv_coordinates_[0],
            face.uv_coordinates_[1],
            face.uv_coordinates_[2]);
    }
}
#endif
void makeMeshFromPointsCloud(const std::vector<Point_3>& points_cloud, PolygonMesh& output_mesh, const coord_t points_grid_resolution)
{
    const double alpha = points_grid_resolution * 5.0;
    // const double alpha = 1000.0;
    const double offset = alpha / 50.0;

    CGAL::alpha_wrap_3(points_cloud, alpha, offset, output_mesh);
}

std::tuple<Point_3, Point_3, Point_3> getFaceVertices(const PolygonMesh& mesh, CGAL::SM_Face_index face)
{
    CGAL::SM_Halfedge_index h = mesh.halfedge(face);
    Point_3 point_A = mesh.point(mesh.source(h));
    h = mesh.next(h);
    Point_3 point_B = mesh.point(mesh.source(h));
    Point_3 point_C = mesh.point(mesh.target(h));

    return std::make_tuple(point_A, point_B, point_C);
}

void makeModifierMesh(const PolygonMesh& mesh, const AABB3D& bounding_box, const png::image<png::rgb_pixel>& image, PolygonMesh& output_mesh)
{
    constexpr coord_t points_grid_resolution = 400;
    constexpr coord_t expand_width = points_grid_resolution * 2.0;

    AABB3D expanded_bounding_box = bounding_box;
    expanded_bounding_box.expand(expand_width);

    // Create an AABB tree for efficient spatial queries
    typedef CGAL::AABB_face_graph_triangle_primitive<PolygonMesh> Primitive;
    typedef CGAL::AABB_traits_3<Kernel, Primitive> AABB_traits;
    typedef CGAL::AABB_tree<AABB_traits> Tree;

    spdlog::info("Prepate AABB tree for fast look-up");

    Tree tree(faces(mesh).begin(), faces(mesh).end(), mesh);
    tree.accelerate_distance_queries();

    std::vector<coord_t> x_values;
    for (coord_t x = expanded_bounding_box.min_.x_; x < expanded_bounding_box.max_.x_; x += points_grid_resolution)
    {
        x_values.push_back(x);
    }

    std::vector<Point_3> points_cloud;
    std::mutex mutex;

    spdlog::info("Generate Voronoi points cloudd");
    TimeKeeper time_keeper;
    time_keeper.restart();

    cura::parallel_for(
        x_values,
        [&](auto iterator)
        {
            const coord_t x = *iterator;

            // spdlog::info("Generate grid with X={}", x);

            for (coord_t y = expanded_bounding_box.min_.y_; y < expanded_bounding_box.max_.y_; y += points_grid_resolution)
            {
                for (coord_t z = expanded_bounding_box.min_.z_; z < expanded_bounding_box.max_.z_; z += points_grid_resolution)
                {
                    // Let's call A,B,C the vertices of the closest triangle, D is the grid point being evaluated, and E its projection on the triangle
                    // // Find the closest face and project the point to it
                    Point_3 point_D(x, y, z);

                    // Find closest point and face
                    std::pair<Point_3, Primitive::Id> closest_point_and_primitive = tree.closest_point_and_primitive(point_D);
                    CGAL::SM_Face_index closest_face = closest_point_and_primitive.second;
                    // CGAL::SM_Face_index closest_face = CGAL::SM_Face_index(0);

                    // Get the face vertices and UV coordinates
                    auto uv_coords = mesh.property_map<CGAL::SM_Face_index, std::array<Point_2, 3>>("f:uv_coords").value();
                    std::array<Point_2, 3> face_uvs = uv_coords[closest_face];

                    // Get the vertices of the face
                    Point_3 point_A;
                    Point_3 point_B;
                    Point_3 point_C;
                    std::tie(point_A, point_B, point_C) = getFaceVertices(mesh, closest_face);

                    // First, calculate the normal vector of the triangle
                    Kernel::Vector_3 edge1 = point_B - point_A;
                    Kernel::Vector_3 edge2 = point_C - point_A;
                    Kernel::Vector_3 normal = CGAL::cross_product(edge1, edge2);
                    normal /= std::sqrt(normal.squared_length());

                    Kernel::Vector_3 vec_DA(point_D - point_A);

                    double distance_to_triangle = vec_DA * normal;
                    if (distance_to_triangle > expand_width)
                    {
                        continue;
                    }
                    Kernel::Point_3 point_E = point_D - distance_to_triangle * normal;

                    std::optional<Point_3> barycentric_coordinates_opt = getBarycentricCoordinates(point_E, point_A, point_B, point_C);
                    if (! barycentric_coordinates_opt.has_value())
                    {
                        continue;
                    }

                    // Calculate the UV coordinates using barycentric interpolation
                    Point_3 barycentric_coordinates = barycentric_coordinates_opt.value();
                    Point2F uv_coords_at_point(
                        barycentric_coordinates.x() * face_uvs[2].x() + barycentric_coordinates.y() * face_uvs[0].x() + barycentric_coordinates.z() * face_uvs[1].x(),
                        barycentric_coordinates.x() * face_uvs[2].y() + barycentric_coordinates.y() * face_uvs[0].y() + barycentric_coordinates.z() * face_uvs[1].y());

                    Point2F pixel_coordinates = getPixelCoordinates(uv_coords_at_point, image);

                    // Get the color at this position
                    png::rgb_pixel color = image.get_pixel(pixel_coordinates.x_, pixel_coordinates.y_);

                    if (color.red > 128)
                    {
                        mutex.lock();
                        points_cloud.push_back(point_D);
                        mutex.unlock();
                    }
                }
            }
        });

    spdlog::info("Points cloud computation took {}s", time_keeper.restart());

    exportPointsCloud(points_cloud, "points_cloud");

    spdlog::info("Make mesh from points cloud");
    makeMeshFromPointsCloud(points_cloud, output_mesh, points_grid_resolution);
}

void registerModifiedMesh(MeshGroup* meshgroup, const PolygonMesh& output_mesh)
{
    ExtruderTrain& extruder = Application::getInstance().current_slice_->scene.extruders.at(1);
    Mesh modifier_mesh(extruder.settings_);
    for (const CGAL::SM_Face_index face : output_mesh.faces())
    {
        Point_3 point_A;
        Point_3 point_B;
        Point_3 point_C;
        std::tie(point_A, point_B, point_C) = getFaceVertices(output_mesh, face);

        modifier_mesh.addFace(Point3LL(point_A.x(), point_A.y(), point_A.z()), Point3LL(point_B.x(), point_B.y(), point_B.z()), Point3LL(point_C.x(), point_C.y(), point_C.z()));
    }

    modifier_mesh.settings_.add("cutting_mesh", "true");
    modifier_mesh.settings_.add("extruder_nr", "1");

    meshgroup->meshes.push_back(modifier_mesh);
}

void splitMesh(Mesh& mesh, MeshGroup* meshgroup)
{
    png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/dino-texture.png");

    // Copy mesh but clear faces, so that we keep the settings data
    // Mesh texture_split_mesh = mesh;
    // texture_split_mesh.clear();
    // for (const MeshFace& face : mesh.faces_)
    // {
    //     splitFaceToTexture(mesh, face, image, texture_split_mesh);
    // }
    // mesh = texture_split_mesh;

    PolygonMesh converted_mesh;

    for (const MeshVertex& vertex : mesh.vertices_)
    {
        converted_mesh.add_vertex(Point_3(vertex.p_.x_, vertex.p_.y_, vertex.p_.z_));
    }

    auto uv_coords = converted_mesh.add_property_map<CGAL::SM_Face_index, std::array<Point_2, 3>>("f:uv_coords").first;
    for (const MeshFace& face : mesh.faces_)
    {
        CGAL::SM_Face_index face_index
            = converted_mesh.add_face(CGAL::SM_Vertex_index(face.vertex_index_[0]), CGAL::SM_Vertex_index(face.vertex_index_[1]), CGAL::SM_Vertex_index(face.vertex_index_[2]));

        if (face_index == PolygonMesh::null_face())
        {
            continue;
        }

        std::array<Point_2, 3> face_uvs;

        for (size_t j = 0; j < 3; ++j)
        {
            if (face.uv_coordinates_[j].has_value())
            {
                const auto& uv = face.uv_coordinates_[j].value();
                face_uvs[j] = Point_2(uv.x_, uv.y_);
            }
            else
            {
                face_uvs[j] = Point_2(-1.0, -1.0);
            }
        }

        uv_coords[face_index] = face_uvs;
    }

    PolygonMesh output_mesh;
    makeModifierMesh(converted_mesh, mesh.getAABB(), image, output_mesh);
    registerModifiedMesh(meshgroup, output_mesh);

    exportMesh(converted_mesh, "converted_mesh");
    exportMesh(output_mesh, "output_mesh");
}

} // namespace cura::MeshMaterialSplitter