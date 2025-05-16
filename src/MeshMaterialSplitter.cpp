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
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/alpha_wrap_3.h>

#include <range/v3/algorithm/all_of.hpp>
#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/algorithm/remove.hpp>
#include <range/v3/algorithm/remove_if.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/transform.hpp>
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
// using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Triangle_2 = Kernel::Triangle_2;
using Triangle_3 = Kernel::Triangle_3;
using Plane_3 = Kernel::Plane_3;
using Vector_3 = Kernel::Vector_3;
using Direction_3 = Kernel::Direction_3;
using Delaunay = CGAL::Delaunay_triangulation_3<Kernel>;
using PolygonMesh = CGAL::Surface_mesh<Kernel::Point_3>;
using TreeTraits = CGAL::Search_traits_3<Kernel>;
using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<TreeTraits>;
using SearchTree = Neighbor_search::Tree;
using Fuzzy_Sphere = CGAL::Fuzzy_sphere<TreeTraits>;

using Kernel_S64 = CGAL::Simple_cartesian<int64_t>;
using Point_3S64 = Kernel_S64::Point_3;

using Kernel_U32 = CGAL::Simple_cartesian<uint32_t>;
using Point_3U32 = Kernel_U32::Point_3;
using Point_2U32 = Kernel_U32::Point_2;

using Kernel_S8 = CGAL::Simple_cartesian<int8_t>;
using Point_3S8 = Kernel_S8::Point_3;
using Vector_3S8 = Kernel_S8::Vector_3;

using Kernel_U8 = CGAL::Simple_cartesian<uint8_t>;
using Point_3U8 = Kernel_U8::Point_3;

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

template<typename PointContainer>
void exportPointsCloud(const PointContainer& points_cloud, const std::string& filename)
{
    PolygonMesh exported_mesh;
    for (const Point_3& point : points_cloud)
    {
        exported_mesh.add_vertex(point);
    }
    exportMesh(exported_mesh, filename);
}

class GrowingPointsCloud
{
public:
    explicit GrowingPointsCloud(const size_t extruder_nr, std::vector<Point_3>&& shell_points)
        : extruder_nr_(extruder_nr)
        , growable_shell_(std::move(shell_points))
        , covered_domain_(growable_shell_)
    {
        covered_domain_.build();
    }

    size_t getExtruderNr() const
    {
        return extruder_nr_;
    }

    const SearchTree& getSearchTree() const
    {
        return covered_domain_;
    }

    void exportTo(const std::string& filename) const
    {
        PolygonMesh exported_mesh;

        for (const Point_3& point : covered_domain_)
        {
            exported_mesh.add_vertex(point);
        }
        exportMesh(exported_mesh, filename);
    }

    static std::vector<Vector_3> makeGrowDeltas()
    {
        std::vector<Vector_3> grow_deltas;

        for (double delta_x : { grow_radius, -grow_radius })
        {
            grow_deltas.emplace_back(delta_x, 0, 0);
        }

        for (double delta_y : { grow_radius, -grow_radius })
        {
            grow_deltas.emplace_back(0, delta_y, 0);
        }

        for (double delta_z : { grow_radius, -grow_radius })
        {
            grow_deltas.emplace_back(0, 0, delta_z);
        }

        const coord_t grow_edge = (grow_radius * std::sqrt(3.0)) / 3.0;
        for (coord_t delta_x : { grow_edge, -grow_edge })
        {
            for (coord_t delta_y : { grow_edge, -grow_edge })
            {
                for (coord_t delta_z : { grow_edge, -grow_edge })
                {
                    grow_deltas.emplace_back(delta_x, delta_y, delta_z);
                }
            }
        }

        return grow_deltas;
    }

    static void doWatershed(std::vector<std::shared_ptr<GrowingPointsCloud>>& points_clouds, const CGAL::Bbox_3& boundaries)
    {
        std::vector<Vector_3> grow_deltas = makeGrowDeltas();

        bool point_grown;

        size_t iteration = 0;
        do
        {
            point_grown = false;

            std::map<std::shared_ptr<GrowingPointsCloud>, std::vector<Point_3>> grow_results;

            for (auto [index, points_cloud] : points_clouds | ranges::views::enumerate)
            {
                points_cloud->exportTo(fmt::format("points_cloud_{}_it_{}", index, iteration));
                grow_results[points_cloud] = points_cloud->evaluateGrowing2(points_clouds, boundaries, grow_deltas);
            }


            for (auto iterator = grow_results.begin(); iterator != grow_results.end(); ++iterator)
            {
                const std::shared_ptr<GrowingPointsCloud>& points_cloud = iterator->first;
                spdlog::info("Apply growing for ex {}", points_cloud->getExtruderNr());
                point_grown |= points_cloud->applyGrowing2(std::move(iterator->second));
                spdlog::info("Ended applying growing");
            }

            iteration++;
        } while (point_grown);
    }


private:
    enum class GrowCapacity
    {
        Unknown, // We don't know yet
        CanGrow, // Growable point can be accepted, it is not close to anything else
        BlockedBySimilar, // Growable point is blocked because too close to other points of a similar points cloud
        BlockedByDifferent, // Growable point is blocked because too close to other points of a different points cloud, or outside boundaries
    };

    struct GrowingCandidate
    {
        Point_3 position;
        GrowCapacity capacity{ GrowCapacity::Unknown };
    };

    struct GrowingPoint
    {
        std::vector<GrowingCandidate> grow_candidates;
    };

private:
    bool evaluateGrowCandidate2(const Point_3& grow_position, const std::vector<std::shared_ptr<GrowingPointsCloud>>& points_clouds, const CGAL::Bbox_3& boundaries) const
    {
        if (grow_position.x() < boundaries.xmin() || grow_position.x() > boundaries.xmax() || grow_position.y() < boundaries.ymin() || grow_position.y() > boundaries.ymax()
            || grow_position.z() < boundaries.zmin() || grow_position.z() > boundaries.zmax())
        {
            return false;
        }

        Fuzzy_Sphere lookup_sphere_similar(grow_position, GrowingPointsCloud::grow_radius * 0.95, 0);
        Fuzzy_Sphere lookup_sphere_different(grow_position, GrowingPointsCloud::grow_radius * sqrt(2) * 1.01, 0);

        for (const std::shared_ptr<GrowingPointsCloud>& points_cloud : points_clouds)
        {
            const bool same_extruder = points_cloud->getExtruderNr() == getExtruderNr();

            // const Fuzzy_Sphere &lookup_sphere = same_extruder ? lookup_sphere_similar : lookup_sphere_different;
            const Fuzzy_Sphere& lookup_sphere = lookup_sphere_similar;

            std::vector<Point_3> nearby_points;
            points_cloud->covered_domain_.search(std::back_inserter(nearby_points), lookup_sphere);

            // if (! nearby_points.empty() && ((points_cloud.get() != this || (&shell != &grown_shells_.back()) || nearby_points.size() > 1)))
            if (! nearby_points.empty())
            {
                return false;
            }
        }

        return true;
    }

    bool applyGrowing2(std::vector<Point_3>&& new_shell)
    {
        const bool has_grown_shell = ! new_shell.empty();

        if (add_growable_to_covered_)
        {
            covered_domain_.insert(growable_shell_.begin(), growable_shell_.end());
            covered_domain_.build();
        }
        add_growable_to_covered_ = true;

        growable_shell_ = std::move(new_shell);

        return has_grown_shell;
    }

    std::vector<Point_3>
        evaluateGrowing2(const std::vector<std::shared_ptr<GrowingPointsCloud>>& points_clouds, const CGAL::Bbox_3& boundaries, const std::vector<Vector_3>& grow_deltas) const
    {
        spdlog::info("Start evaluating {} points cloud candidates for ex {}", (growable_shell_.size() * grow_deltas.size()), getExtruderNr());

        std::vector<Point_3> new_shell;

        std::mutex mutex;
        cura::parallel_for(
            growable_shell_,
            [&](auto iterator)
            {
                const Point_3& outer_shell_point = *iterator;

                for (const Vector_3& grow_delta : grow_deltas)
                {
                    Point_3 grow_candidate = outer_shell_point + grow_delta;
                    if (evaluateGrowCandidate2(grow_candidate, points_clouds, boundaries))
                    {
                        mutex.lock();
                        new_shell.push_back(grow_candidate);
                        mutex.unlock();
                    }
                }
            });

        return new_shell;
    }

public:
    static constexpr coord_t grow_radius = 1000;
    static constexpr coord_t grow_radius_squared = grow_radius * grow_radius;

private:
    const size_t extruder_nr_;
    std::vector<Point_3> growable_shell_;
    SearchTree covered_domain_;
    bool add_growable_to_covered_{ false };
};

Point_2 getPixelCoordinates(const Point2F& uv_coordinates, const png::image<png::rgb_pixel>& image)
{
    const uint32_t width = image.get_width();
    const uint32_t height = image.get_height();
    return Point_2(
        std::clamp(static_cast<uint32_t>(uv_coordinates.x_ * width), static_cast<uint32_t>(0), width - 1),
        std::clamp(static_cast<uint32_t>(height - uv_coordinates.y_ * height), static_cast<uint32_t>(0), height - 1));
}

Point_2 getPixelCoordinates(const Point_2& uv_coordinates, const png::image<png::rgb_pixel>& image)
{
    const uint32_t width = image.get_width();
    const uint32_t height = image.get_height();
    return Point_2(
        std::clamp(static_cast<uint32_t>(uv_coordinates.x() * width), static_cast<uint32_t>(0), width - 1),
        std::clamp(static_cast<uint32_t>(height - uv_coordinates.y() * height), static_cast<uint32_t>(0), height - 1));
}

#if 1
Point3LL getSpaceCoordinates(const Point_2& pixel_coordinates, const Point_2 triangle_coordinates[3], const Mesh& mesh, const MeshFace& face)
{
    // Calculate 3D space coordinates from pixel coordinates using barycentric coordinates
    const Point_2& p0 = triangle_coordinates[0];
    const Point_2& p1 = triangle_coordinates[1];
    const Point_2& p2 = triangle_coordinates[2];

    // Calculate barycentric coordinates
    const float area = 0.5f * ((p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y()));
    const float u = 0.5f * ((p1.x() - pixel_coordinates.x()) * (p2.y() - pixel_coordinates.y()) - (p2.x() - pixel_coordinates.x()) * (p1.y() - pixel_coordinates.y())) / area;
    const float v = 0.5f * ((pixel_coordinates.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (pixel_coordinates.y() - p0.y())) / area;
    const float w = 1.0f - u - v;

    // Apply barycentric coordinates to get 3D position
    const Point3LL& v0 = mesh.vertices_[face.vertex_index_[0]].p_;
    const Point3LL& v1 = mesh.vertices_[face.vertex_index_[1]].p_;
    const Point3LL& v2 = mesh.vertices_[face.vertex_index_[2]].p_;

    return Point3LL(u * v0.x_ + v * v1.x_ + w * v2.x_, u * v0.y_ + v * v1.y_ + w * v2.y_, u * v0.z_ + v * v1.z_ + w * v2.z_);
}
#endif

Point_3 getSpaceCoordinates(const Point_2& pixel_coordinates, const Triangle_2& triangle_coordinates, const Triangle_3& triangle)
{
    // Calculate 3D space coordinates from pixel coordinates using barycentric coordinates
    const Point_2& p0 = triangle_coordinates[0];
    const Point_2& p1 = triangle_coordinates[1];
    const Point_2& p2 = triangle_coordinates[2];

    // Calculate barycentric coordinates
    const float area = 0.5f * ((p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y()));
    const float u = 0.5f * ((p1.x() - pixel_coordinates.x()) * (p2.y() - pixel_coordinates.y()) - (p2.x() - pixel_coordinates.x()) * (p1.y() - pixel_coordinates.y())) / area;
    const float v = 0.5f * ((pixel_coordinates.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (pixel_coordinates.y() - p0.y())) / area;
    const float w = 1.0f - u - v;

    // Apply barycentric coordinates to get 3D position
    const Point_3& v0 = triangle[0];
    const Point_3& v1 = triangle[1];
    const Point_3& v2 = triangle[2];

    return Point_3(u * v0.x() + v * v1.x() + w * v2.x(), u * v0.y() + v * v1.y() + w * v2.y(), u * v0.z() + v * v1.z() + w * v2.z());
}

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

std::optional<Point_3> getBarycentricCoordinates(const Point_2& point, const Point_2& p0, const Point_2& p1, const Point_2& p2)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const Kernel::Vector_2 v0(p1 - p0);
    const Kernel::Vector_2 v1(p2 - p0);
    const Kernel::Vector_2 v2(point - p0);

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

    // Return as a Point_3 where x/y/z represent the barycentric coordinates u/v/w
    return Point_3(u, v, w);
}

Point_2 getUVFromBarycentricCoordinates(const Point_3& barycentric_coordinates, const std::array<Point_2, 3>& face_uvs)
{
    return Point_2(
        barycentric_coordinates.x() * face_uvs[2].x() + barycentric_coordinates.y() * face_uvs[0].x() + barycentric_coordinates.z() * face_uvs[1].x(),
        barycentric_coordinates.x() * face_uvs[2].y() + barycentric_coordinates.y() * face_uvs[0].y() + barycentric_coordinates.z() * face_uvs[1].y());
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

template<typename PointsContainer>
void makeMeshFromPointsCloud(const PointsContainer& points_cloud, PolygonMesh& output_mesh, const coord_t points_grid_resolution)
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

Triangle_3 getFaceTriangle(const PolygonMesh& mesh, CGAL::SM_Face_index face)
{
    std::tuple<Point_3, Point_3, Point_3> points = getFaceVertices(mesh, face);
    return Triangle_3(std::get<0>(points), std::get<1>(points), std::get<2>(points));
}

#if 0
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
#endif

#if 0
std::vector<GrowingPointsCloud> makeInitialPointsCloudsFromTexture(const Mesh& mesh, const png::image<png::rgb_pixel>& image)
{
    std::vector<GrowingPointsCloud> points_clouds;

    for (const MeshFace& face : mesh.faces_)
    {
        // First, convert UV coordinates of the 3 points to pixel coordinates on the image
#warning Replace this by Triangle_2
        Point_2 face_pixel_coordinates[3];
        bool all_uv_coordinates = true;

        for (size_t i = 0; i < 3 && all_uv_coordinates; ++i)
        {
            if (face.uv_coordinates_[i].has_value())
            {
                face_pixel_coordinates[i] = getPixelCoordinates(face.uv_coordinates_[i].value(), image);
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
            min.x_ = std::min({ face_pixel_coordinates[0].x(), face_pixel_coordinates[1].x(), face_pixel_coordinates[2].x() });
            min.y_ = std::min({ face_pixel_coordinates[0].y(), face_pixel_coordinates[1].y(), face_pixel_coordinates[2].y() });

            Point2F max;
            max.x_ = std::max({ face_pixel_coordinates[0].x(), face_pixel_coordinates[1].x(), face_pixel_coordinates[2].x() });
            max.y_ = std::max({ face_pixel_coordinates[0].y(), face_pixel_coordinates[1].y(), face_pixel_coordinates[2].y() });

            const Point2LL min_pixel(min.x_, min.y_);
            const Point2LL max_pixel(max.x_, max.y_);

            for (coord_t x = min_pixel.X; x < max_pixel.X; ++x)
            {
                for (coord_t y = min_pixel.Y; y < max_pixel.Y; ++y)
                {
                    const Point_2 pixel_center(Point_2(x + 0.5, y + 0.5));
                    const std::optional<Point_3> barycentric_coordinates
                        = getBarycentricCoordinates(pixel_center, face_pixel_coordinates[0], face_pixel_coordinates[1], face_pixel_coordinates[2]);

                    if (! barycentric_coordinates.has_value() || barycentric_coordinates.value().x() < 0 || barycentric_coordinates.value().y() < 0
                        || barycentric_coordinates.value().z() < 0)
                    {
                        // Triangle is invalid, or point is outside the triangle
                        continue;
                    }

                    const Point3LL pixel_3d = getSpaceCoordinates(pixel_center, face_pixel_coordinates, mesh, face);
                    const png::rgb_pixel color = image.get_pixel(x, y);

                    const size_t extruder_nr = color.red > 128 ? 0 : 1;
                    auto iterator = ranges::find_if(
                        points_clouds,
                        [&extruder_nr](const GrowingPointsCloud& points_cloud)
                        {
                            return points_cloud.getExtruderNr() == extruder_nr;
                        });
                    if (iterator == points_clouds.end())
                    {
                        points_clouds.emplace_back(extruder_nr);
                    }
                    iterator = ranges::find_if(
                        points_clouds,
                        [&extruder_nr](const GrowingPointsCloud& points_cloud)
                        {
                            return points_cloud.getExtruderNr() == extruder_nr;
                        });

                    iterator->addInitialPoint(pixel_3d);
                }
            }
        }
    }

    return points_clouds;
}
#endif

struct OrthonormalPlane
{
    Point_3 origin;
    Vector_3 base1;
    Vector_3 base2;

    OrthonormalPlane(const Plane_3& plane)
    {
        origin = plane.point();
        base1 = plane.base1();
        base1 /= std::sqrt(base1.squared_length());
        base2 = plane.base2();
        base2 /= std::sqrt(base2.squared_length());
    }
};

Point_2 toPlanCoordinates(const Point_3& point, const OrthonormalPlane& plane)
{
    const Vector_3 delta = point - plane.origin;
    return Point_2(delta * plane.base1, delta * plane.base2);
}

Point_3 fromPlanCoordinates(const Point_2& point, const OrthonormalPlane& plane)
{
    return plane.origin + point.x() * plane.base1 + point.y() * plane.base2;
}

CGAL::Bbox_2 expand(const CGAL::Bbox_2& bounding_box, const double offset)
{
    return CGAL::Bbox_2(bounding_box.xmin() - offset, bounding_box.ymin() - offset, bounding_box.xmax() + offset, bounding_box.ymax() + offset);
}

CGAL::Bbox_3 expand(const CGAL::Bbox_3& bounding_box, const double offset)
{
    return CGAL::Bbox_3(
        bounding_box.xmin() - offset,
        bounding_box.ymin() - offset,
        bounding_box.zmin() - offset,
        bounding_box.xmax() + offset,
        bounding_box.ymax() + offset,
        bounding_box.zmax() + offset);
}

std::vector<std::shared_ptr<GrowingPointsCloud>> makeInitialPointsCloudsFromTexture2(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image)
{
    std::map<size_t, std::vector<Point_3>> raw_points_clouds;
    auto faces = mesh.faces();
    auto uv_coords = mesh.property_map<CGAL::SM_Face_index, std::array<Point_2, 3>>("f:uv_coords").value();

    std::mutex mutex;
    run_multiple_producers_ordered_consumer(
        0,
        faces.size(),
        [&](size_t face_index)
        {
            const CGAL::SM_Face_index face = *std::next(faces.begin(), face_index);
            const std::array<Point_2, 3> face_uvs = uv_coords[face];

            // Now get the bounding box of the triangle on the supporting plane
            const Triangle_3 triangle = getFaceTriangle(mesh, face);
            const OrthonormalPlane plane(triangle.supporting_plane());

            const Triangle_2 triangle_on_plane(toPlanCoordinates(triangle[0], plane), toPlanCoordinates(triangle[1], plane), toPlanCoordinates(triangle[2], plane));

            if (triangle_on_plane.is_degenerate())
            {
                return false;
            }

            const CGAL::Bbox_2 bounding_box = expand(triangle_on_plane.bbox(), -GrowingPointsCloud::grow_radius / 2);

            for (double x = bounding_box.xmin(); x <= bounding_box.xmax(); x += GrowingPointsCloud::grow_radius)
            {
                for (double y = bounding_box.ymin(); y <= bounding_box.ymax(); y += GrowingPointsCloud::grow_radius)
                {
                    const Point_2 sampled_triangle_point(Point_2(x, y));
                    const std::optional<Point_3> barycentric_coordinates
                        = getBarycentricCoordinates(sampled_triangle_point, triangle_on_plane[0], triangle_on_plane[1], triangle_on_plane[2]);

                    if (! barycentric_coordinates.has_value() || barycentric_coordinates.value().x() < 0 || barycentric_coordinates.value().y() < 0
                        || barycentric_coordinates.value().z() < 0)
                    {
                        // Triangle is invalid, or point is outside the triangle
                        continue;
                    }

                    const Point_2 uv_coords_at_point = getUVFromBarycentricCoordinates(barycentric_coordinates.value(), face_uvs);
                    const Point_2 pixel_coordinates = getPixelCoordinates(uv_coords_at_point, image);

                    // Get the color at this position
                    const png::rgb_pixel color = image.get_pixel(pixel_coordinates.x(), pixel_coordinates.y());

                    const size_t extruder_nr = color.red > 128 ? 0 : 1;
                    const Point_3 point_3d = fromPlanCoordinates(sampled_triangle_point, plane);

                    mutex.lock();
                    std::vector<Point_3>& raw_points_cloud = raw_points_clouds[extruder_nr];
                    // auto iterator = ranges::find_if(
                    //     raw_points_cloud,
                    //     [&point_3d](const Point_3& point_in_points_cloud)
                    //     {
                    //         constexpr coord_t epsilon = 5 * 5;
                    //         return (point_in_points_cloud - point_3d).squared_length() < epsilon;
                    //     });
                    //
                    // if (iterator == raw_points_cloud.end())
                    {
                        raw_points_cloud.push_back(point_3d);
                    }
                    mutex.unlock();
                }
            }

            return true;
        },
        [](bool result) {});

    std::vector<std::shared_ptr<GrowingPointsCloud>> points_clouds;

    for (auto& [extruder_nr, raw_points_cloud] : raw_points_clouds)
    {
        points_clouds.emplace_back(std::make_shared<GrowingPointsCloud>(extruder_nr, std::move(raw_points_cloud)));
    }

    return points_clouds;
}

class OctreeNode
{
public:
    enum class ExtruderOccupation : uint8_t
    {
        Unknown = 0,
        InsideMesh = 1, // Voxel is tagged as being inside the mesh, but not assigned yet
        OutsideMesh = 2, // Voxel is tagged as being outside the mesh
        Occupied = 3, // When occupied, the actual extruder value is the value - Occupied
    };

    using Ptr = std::shared_ptr<OctreeNode>;
    using ChildNodes = std::array<Ptr, 8>;

    OctreeNode(OctreeNode* parent = nullptr, const ExtruderOccupation& occupation = ExtruderOccupation::Unknown)
        : data_(occupation)
        , parent_(parent)
    {
    }

    bool hasChildren() const
    {
        return std::holds_alternative<ChildNodes>(data_);
    }

    Ptr getChild(const uint8_t index) const
    {
        return std::get<ChildNodes>(data_)[index];
    }

    OctreeNode* getParent()
    {
        return parent_;
    }

    void splitToChildren()
    {
        if (const ExtruderOccupation* current_occupation_ptr = std::get_if<ExtruderOccupation>(&data_))
        {
            const ExtruderOccupation current_occupation = *current_occupation_ptr;
            data_ = ChildNodes();
            for (Ptr& child_node : std::get<ChildNodes>(data_))
            {
                child_node = std::make_shared<OctreeNode>(this, current_occupation);
            }
        }
    }

    bool compressIfPossible()
    {
        if (const ChildNodes* child_nodes = std::get_if<ChildNodes>(&data_))
        {
            std::optional<ExtruderOccupation> current_occupation;
            for (const Ptr& child_node : *child_nodes)
            {
                if (child_node->hasChildren() || (current_occupation.has_value() && child_node->getOccupation() != current_occupation.value()))
                {
                    return false;
                }

                current_occupation = child_node->getOccupation();
            }

            data_ = current_occupation.value();
        }

        // If already containing single occupation, indicate as compressed
        return true;
    }

    const ExtruderOccupation& getOccupation() const
    {
        return std::get<ExtruderOccupation>(data_);
    }

    void setOccupation(const ExtruderOccupation& occupation)
    {
        data_ = occupation;
    }

private:
    std::variant<ExtruderOccupation, ChildNodes> data_;
    OctreeNode* parent_;
};

class VoxelSpace
{
public:
    using LocalCoordinates = Point_3U32;

public:
    explicit VoxelSpace(const PolygonMesh& mesh)
        : root_(std::make_shared<OctreeNode>())
    {
        const CGAL::Bbox_3 expanded_bounding_box = expand(CGAL::Polygon_mesh_processing::bbox(mesh), max_definition_ * 2);
        origin_ = Vector_3(expanded_bounding_box.xmin(), expanded_bounding_box.ymin(), expanded_bounding_box.zmin());
        definition_ = Point_3(expanded_bounding_box.x_span(), expanded_bounding_box.y_span(), expanded_bounding_box.z_span());

        double max_definition;
        do
        {
            max_depth_++;
            definition_ = Point_3(definition_.x() / 2.0, definition_.y() / 2.0, definition_.z() / 2.0);
            max_definition = std::max({ definition_.x(), definition_.y(), definition_.z() });
        } while (max_definition > max_definition_);

        max_coordinate_ = (1 << max_depth_) - 1;
    }

    const Point_3& getDefinition() const
    {
        return definition_;
    }

    Point_3 toGlobalCoordinates(const LocalCoordinates& position) const
    {
        return Point_3(position.x() * definition_.x(), position.y() * definition_.y(), position.z() * definition_.z()) + origin_;
    }

    void setExtruderNr(const Point_3& position, const size_t extruder_nr)
    {
        setOccupation(toLocalCoordinates(position), static_cast<OctreeNode::ExtruderOccupation>(static_cast<uint8_t>(OctreeNode::ExtruderOccupation::Occupied) + extruder_nr));
    }

    void setOccupation(const Point_3& position, const OctreeNode::ExtruderOccupation& occupation)
    {
        setOccupation(toLocalCoordinates(position), occupation);
    }

    bool setOccupation(const LocalCoordinates& position, const OctreeNode::ExtruderOccupation& occupation)
    {
        auto [depth, node] = getNode(position);
        if (node->getOccupation() != occupation)
        {
            bool children_split = false;
            while (depth < max_depth_)
            {
                // Node is not a leaf node, we have to split it to set exactly this position and not the others around
                node->splitToChildren();
                node = findSubNode(node, depth++, position);
                children_split = true;
            }

            node->setOccupation(occupation);

            if (! children_split)
            {
                // We have just changed the occupation value of an existing node, so try and check whether the tree can be compressed now
                OctreeNode* node_ptr = node.get();
                do
                {
                    node_ptr = node_ptr->getParent();
                } while (node_ptr && node_ptr->compressIfPossible());
            }

            return true;
        }

        return false;
    }

    uint8_t getExtruderNr(const LocalCoordinates& local_position) const
    {
        return static_cast<uint8_t>(getOccupation(local_position)) - static_cast<uint8_t>(OctreeNode::ExtruderOccupation::Occupied);
    }

    const OctreeNode::ExtruderOccupation& getOccupation(const LocalCoordinates& local_position) const
    {
        return std::get<1>(getNode(local_position))->getOccupation();
    }

    OctreeNode::ExtruderOccupation getOccupation(const Point_3& position) const
    {
        return getOccupation(toLocalCoordinates(position));
    }

    std::vector<LocalCoordinates> getFilledVoxels() const
    {
        std::vector<LocalCoordinates> coordinates;

        std::function<void(const std::shared_ptr<OctreeNode>&, LocalCoordinates, int)> collect_filled_voxels;
        collect_filled_voxels = [this, &coordinates, &collect_filled_voxels](const std::shared_ptr<OctreeNode>& node, const LocalCoordinates& coord, const int depth)
        {
            if (! node)
            {
                return;
            }

            if (depth == max_depth_)
            {
                if (node->getOccupation() >= OctreeNode::ExtruderOccupation::Occupied)
                {
                    coordinates.push_back(coord);
                }
                return;
            }

            if (node->hasChildren())
            {
                for (uint8_t i = 0; i < 8; ++i)
                {
                    auto child = node->getChild(i);
                    LocalCoordinates child_coord = coord;
                    if (i & 1)
                    {
                        child_coord = LocalCoordinates(child_coord.x() + (1ULL << (max_depth_ - depth - 1)), child_coord.y(), child_coord.z());
                    }
                    if (i & 2)
                    {
                        child_coord = LocalCoordinates(child_coord.x(), child_coord.y() + (1ULL << (max_depth_ - depth - 1)), child_coord.z());
                    }
                    if (i & 4)
                    {
                        child_coord = LocalCoordinates(child_coord.x(), child_coord.y(), child_coord.z() + (1ULL << (max_depth_ - depth - 1)));
                    }
                    collect_filled_voxels(child, child_coord, depth + 1);
                }
            }
            else
            {
                if (node->getOccupation() >= OctreeNode::ExtruderOccupation::Occupied)
                {
                    // Fill all voxels in this region
                    size_t voxels_per_side = 1ULL << (max_depth_ - depth);
                    for (size_t dx = 0; dx < voxels_per_side; ++dx)
                    {
                        for (size_t dy = 0; dy < voxels_per_side; ++dy)
                        {
                            for (size_t dz = 0; dz < voxels_per_side; ++dz)
                            {
                                coordinates.push_back(LocalCoordinates(coord.x() + dx, coord.y() + dy, coord.z() + dz));
                            }
                        }
                    }
                }
            }
        };
        collect_filled_voxels(root_, LocalCoordinates(0, 0, 0), 0);

        return coordinates;
    }

    std::vector<LocalCoordinates> getVoxelsAround(const LocalCoordinates& point) const
    {
        std::vector<LocalCoordinates> voxels_around;
#if 1
        voxels_around.reserve(3 * 3 * 3 - 1);

        for (const int8_t delta_x : { -1, 0, 1 })
        {
            const int64_t pos_x = point.x() + delta_x;
            if (pos_x < 0 || pos_x > max_coordinate_)
            {
                continue;
            }

            for (const int8_t delta_y : { -1, 0, 1 })
            {
                const int64_t pos_y = point.y() + delta_y;
                if (pos_y < 0 || pos_y > max_coordinate_)
                {
                    continue;
                }

                for (const int8_t delta_z : { -1, 0, 1 })
                {
                    const int64_t pos_z = point.z() + delta_z;
                    if (pos_z < 0 || pos_z > max_coordinate_)
                    {
                        continue;
                    }

                    if (delta_x || delta_y || delta_z)
                    {
                        voxels_around.emplace_back(pos_x, pos_y, pos_z);
                    }
                }
            }
        }
#else
        voxels_around.reserve(2 * 3);

        for (const int8_t delta_x : { -1, 1 })
        {
            const int64_t pos_x = point.x() + delta_x;
            if (pos_x < 0 || pos_x > max_coordinate_)
            {
                continue;
            }

            voxels_around.emplace_back(pos_x, point.y(), point.z());
        }

        for (const int8_t delta_y : { -1, 1 })
        {
            const int64_t pos_y = point.y() + delta_y;
            if (pos_y < 0 || pos_y > max_coordinate_)
            {
                continue;
            }

            voxels_around.emplace_back(point.x(), pos_y, point.z());
        }

        for (const int8_t delta_z : { -1, 1 })
        {
            const int64_t pos_z = point.z() + delta_z;
            if (pos_z < 0 || pos_z > max_coordinate_)
            {
                continue;
            }

            voxels_around.emplace_back(point.x(), point.y(), pos_z);
        }
#endif

        return voxels_around;
    }

public:
    static constexpr double max_definition_ = 200;

private:
    LocalCoordinates toLocalCoordinates(const Point_3& position) const
    {
        const Point_3 position_in_space = position - origin_;
        return LocalCoordinates(position_in_space.x() / definition_.x(), position_in_space.y() / definition_.y(), position_in_space.z() / definition_.z());
    }

    OctreeNode::Ptr findSubNode(const OctreeNode::Ptr& parent, const uint8_t parent_depth, const LocalCoordinates& local_position) const
    {
        // Calculate the child index based on the current depth and position
        uint8_t child_index = 0;
        const uint8_t bit = max_depth_ - parent_depth - 1;
        child_index |= (local_position.x() >> bit) & 1;
        child_index |= ((local_position.y() >> bit) & 1) << 1;
        child_index |= ((local_position.z() >> bit) & 1) << 2;

        return parent->getChild(child_index);
    }

    std::tuple<uint8_t, OctreeNode::Ptr> getNode(const LocalCoordinates& local_position) const
    {
        uint8_t actual_depth = 0;
        OctreeNode::Ptr actual_node = root_;
        while (actual_node->hasChildren())
        {
            actual_node = findSubNode(actual_node, actual_depth++, local_position);
        }

        return std::make_tuple(actual_depth, actual_node);
    }

private:
    OctreeNode::Ptr root_;
    uint8_t max_depth_{ 0 };
    Point_3 definition_;
    Vector_3 origin_;
    uint32_t max_coordinate_;
};

void makeInitialVoxelSpaceFromTexture(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image, VoxelSpace& voxel_space)
{
    const auto faces = mesh.faces();
    const auto uv_coords = mesh.property_map<CGAL::SM_Face_index, std::array<Point_2, 3>>("f:uv_coords").value();

    std::mutex mutex;
    run_multiple_producers_ordered_consumer(
        0,
        faces.size(),
        [&](const size_t face_index)
        {
            const CGAL::SM_Face_index face = *std::next(faces.begin(), face_index);
            const std::array<Point_2, 3> face_uvs = uv_coords[face];
            const Triangle_2 face_pixel_coordinates(getPixelCoordinates(face_uvs[2], image), getPixelCoordinates(face_uvs[0], image), getPixelCoordinates(face_uvs[1], image));
            const Triangle_3 triangle = getFaceTriangle(mesh, face);

            // Now get the bounding box of the triangle on the image
            const Point_2U32 min(
                std::min({ face_pixel_coordinates[0].x(), face_pixel_coordinates[1].x(), face_pixel_coordinates[2].x() }),
                std::min({ face_pixel_coordinates[0].y(), face_pixel_coordinates[1].y(), face_pixel_coordinates[2].y() }));
            const Point_2U32 max(
                std::max({ face_pixel_coordinates[0].x(), face_pixel_coordinates[1].x(), face_pixel_coordinates[2].x() }),
                std::max({ face_pixel_coordinates[0].y(), face_pixel_coordinates[1].y(), face_pixel_coordinates[2].y() }));

            for (uint32_t x = min.x(); x < max.x(); ++x)
            {
                for (uint32_t y = min.y(); y < max.y(); ++y)
                {
                    const Point_2 pixel_center(x + 0.5, y + 0.5);
                    const std::optional<Point_3> barycentric_coordinates
                        = getBarycentricCoordinates(pixel_center, face_pixel_coordinates[0], face_pixel_coordinates[1], face_pixel_coordinates[2]);

                    if (! barycentric_coordinates.has_value() || barycentric_coordinates.value().x() < 0 || barycentric_coordinates.value().y() < 0
                        || barycentric_coordinates.value().z() < 0)
                    {
                        // Triangle is invalid, or point is outside the triangle
                        continue;
                    }

                    const Point_3 pixel_3d = getSpaceCoordinates(pixel_center, face_pixel_coordinates, triangle);
                    const png::rgb_pixel color = image.get_pixel(x, y);
                    const size_t extruder_nr = color.red < 128 ? 0 : 1;

                    mutex.lock();
                    voxel_space.setExtruderNr(pixel_3d, extruder_nr);
                    mutex.unlock();
                }
            }
            return true;
        },
        [](bool result) {});
}

void makeModifierMeshWatershed(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image, PolygonMesh& output_mesh)
{
    std::vector<std::shared_ptr<GrowingPointsCloud>> points_clouds = makeInitialPointsCloudsFromTexture2(mesh, image);
    // exportPointsClouds(points_clouds, "initial_points_cloud");

    const CGAL::Bbox_3 expanded_bounding_box = expand(CGAL::Polygon_mesh_processing::bbox(mesh), GrowingPointsCloud::grow_radius * 1.5);
    GrowingPointsCloud::doWatershed(points_clouds, expanded_bounding_box);

    points_clouds[1]->exportTo("final_contour");
    makeMeshFromPointsCloud(points_clouds[1]->getSearchTree(), output_mesh, GrowingPointsCloud::grow_radius);
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

void makeModifierMeshVoxelSpace(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image, PolygonMesh& output_mesh)
{
    spdlog::info("Fill original voxels based on texture data");
    VoxelSpace voxel_space(mesh);
    makeInitialVoxelSpaceFromTexture(mesh, image, voxel_space);

    spdlog::info("Get initially filled voxels");
    std::vector<VoxelSpace::LocalCoordinates> filled_voxels = voxel_space.getFilledVoxels();

    spdlog::info("Export initial meshes");
    std::map<size_t, PolygonMesh> initial_meshes;
    for (const VoxelSpace::LocalCoordinates& local_coord : filled_voxels)
    {
        initial_meshes[voxel_space.getExtruderNr(local_coord)].add_vertex(voxel_space.toGlobalCoordinates(local_coord));
    }
    for (auto iterator = initial_meshes.begin(); iterator != initial_meshes.end(); ++iterator)
    {
        exportMesh(iterator->second, fmt::format("initial_points_cloud_{}", iterator->first));
    }

    spdlog::info("prepare cleaned mesh");
    // Generate a clean and approximate version of the mesh by alpha-wrapping it, so that we can do proper inside-mesh checking
    PolygonMesh cleaned_mesh;
    constexpr double alpha = 1000.0;
    constexpr double offset = 100.0;
    CGAL::alpha_wrap_3(mesh, alpha, offset, cleaned_mesh);

    exportMesh(cleaned_mesh, "cleaned_mesh");

    CGAL::Side_of_triangle_mesh<PolygonMesh, Kernel> inside_mesh(cleaned_mesh);

    while (! filled_voxels.empty())
    {
        spdlog::info("Voting for {} voxels", filled_voxels.size());

        std::mutex mutex;
        std::map<VoxelSpace::LocalCoordinates, std::map<OctreeNode::ExtruderOccupation, uint8_t>> voxels_votes;

        cura::parallel_for(
            filled_voxels,
            [&](auto iterator)
            {
                const VoxelSpace::LocalCoordinates& filled_voxel = *iterator;
                mutex.lock();
                const OctreeNode::ExtruderOccupation actual_filled_occupation = voxel_space.getOccupation(filled_voxel);
                mutex.unlock();
                std::map<VoxelSpace::LocalCoordinates, std::map<uint8_t, uint8_t>> voxels_votes_local;

                for (const VoxelSpace::LocalCoordinates& voxel_around : voxel_space.getVoxelsAround(filled_voxel))
                {
                    mutex.lock();
                    OctreeNode::ExtruderOccupation occupation = voxel_space.getOccupation(voxel_around);
                    mutex.unlock();
                    if (occupation == OctreeNode::ExtruderOccupation::Unknown)
                    {
                        const Point_3 global_voxel_point = voxel_space.toGlobalCoordinates(voxel_around);
                        if (inside_mesh(global_voxel_point) == CGAL::ON_UNBOUNDED_SIDE)
                        {
                            occupation = OctreeNode::ExtruderOccupation::OutsideMesh;
                        }
                        else
                        {
                            occupation = OctreeNode::ExtruderOccupation::InsideMesh;
                        }

                        mutex.lock();
                        voxel_space.setOccupation(voxel_around, occupation);
                        mutex.unlock();
                    }

                    if (occupation == OctreeNode::ExtruderOccupation::InsideMesh)
                    {
                        // Voxel is not occupied yet, so vote to fill it
                        mutex.lock();
                        ++voxels_votes[voxel_around][actual_filled_occupation];
                        mutex.unlock();
                    }
                }
            });

        spdlog::info("Apply growing");

        std::vector<VoxelSpace::LocalCoordinates> new_filled_voxels;
        new_filled_voxels.reserve(voxels_votes.size());
        for (auto iterator = voxels_votes.begin(); iterator != voxels_votes.end(); ++iterator)
        {
            const auto max_vote = ranges::max_element(
                iterator->second,
                [](const auto& vote_a, const auto& vote_b)
                {
                    return vote_a.second < vote_b.second;
                });
            {
            }

            voxel_space.setOccupation(iterator->first, max_vote->first);
            new_filled_voxels.push_back(iterator->first);
        }

        filled_voxels = std::move(new_filled_voxels);
    }

    spdlog::info("Making final points cloud");
    std::vector<Point_3> points_cloud;
    for (const VoxelSpace::LocalCoordinates& local_coord : voxel_space.getFilledVoxels())
    {
        if (voxel_space.getExtruderNr(local_coord) == 1)
        {
            points_cloud.push_back(voxel_space.toGlobalCoordinates(local_coord));
        }
    }
    exportPointsCloud(points_cloud, "final_contour");

    // const CGAL::Bbox_3 expanded_bounding_box = expand(CGAL::Polygon_mesh_processing::bbox(mesh), GrowingPointsCloud::grow_radius * 1.5);
    // GrowingPointsCloud::doWatershed(points_clouds, expanded_bounding_box);
    //
    // points_clouds[1]->exportTo("final_contour");
    const Point_3& definition = voxel_space.getDefinition();
    spdlog::info("Making mesh from points cloud");
    makeMeshFromPointsCloud(points_cloud, output_mesh, std::max({ definition.x(), definition.y(), definition.z() }));
}

void splitMesh(Mesh& mesh, MeshGroup* meshgroup)
{
    // png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/texture.png");
    // png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/texture-high.png");
    png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/dino-texture.png");

    // Copy mesh but clear faces, so that we keep the settings data
    // Mesh texture_split_mesh = mesh;
    // texture_split_mesh.clear();
    // for (const MeshFace& face : mesh.faces_)
    // {
    //     splitFaceToTexture(mesh, face, image, texture_split_mesh);
    // }
    // mesh = texture_split_mesh;
    // bla

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
    // makeModifierMesh(converted_mesh, mesh.getAABB(), image, output_mesh);
    // makeModifierMeshWatershed(converted_mesh, image, output_mesh);
    makeModifierMeshVoxelSpace(converted_mesh, image, output_mesh);
    registerModifiedMesh(meshgroup, output_mesh);

    exportMesh(converted_mesh, "converted_mesh");
    exportMesh(output_mesh, "output_mesh");
}

} // namespace cura::MeshMaterialSplitter