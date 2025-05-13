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

#include <range/v3/algorithm/remove.hpp>
#include <range/v3/algorithm/remove_if.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/transform.hpp>#include <spdlog/spdlog.h>
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

void exportPointsCloud(const std::vector<Point3LL>& points_cloud, const std::string& filename)
{
    PolygonMesh exported_mesh;
    for (const Point3LL& point : points_cloud)
    {
        exported_mesh.add_vertex(Point_3(point.x_, point.y_, point.z_));
    }
    exportMesh(exported_mesh, filename);
}

class GrowingPointsCloud
{
public:
    struct Shell
    {
        std::vector<Point_3> points;
        SearchTree search_tree;

        explicit Shell(std::vector<Point_3>&& shell_points)
            : points(std::move(shell_points))
            , search_tree(points.begin(), points.end())
        {
            search_tree.build();
        }

        explicit Shell()
        {
        }

        void expand(const Point_3& point)
        {
            search_tree.insert(point);
            points.push_back(point);
        }

        void expand(const std::vector<Point_3>& new_points)
        {
            search_tree.insert(new_points.begin(), new_points.end());
            points.insert(points.end(), new_points.begin(), new_points.end());
            search_tree.build();
        }
    };

public:
    explicit GrowingPointsCloud(const size_t extruder_nr, std::vector<Point_3>&& shell_points)
        : extruder_nr_(extruder_nr)
    {
        pushShell(std::move(shell_points));
    }

    size_t getExtruderNr() const
    {
        return extruder_nr_;
    }

    // void addInitialPoint(const Point3LL& point)
    // {
    //     std::vector<Point_3>& outer_shell = grown_shells_.back().points;
    //     constexpr coord_t epsilon = 5 * 5;
    //     for (const Point_3& actual_point : outer_shell)
    //     {
    //         if ((point - actual_point).vSize2() < epsilon)
    //         {
    //             return;
    //         }
    //     }
    //
    //     outer_shell.push_back(point);
    // }

    void pushShell(std::vector<Point_3>&& shell_points)
    {
        grown_shells_.emplace_back(std::move(shell_points));
    }

    void expandContour(const Point_3& contour_point)
    {
        contour_.expand(contour_point);
    }

    void expandContour(const std::vector<Point_3>& contour_point)
    {
        contour_.expand(contour_point);
    }

    const std::deque<Shell>& getShells() const
    {
        return grown_shells_;
    }

    const std::vector<Point_3>& getContour() const
    {
        return contour_.points;
    }

    auto getAllShellPoints() const
    {
        return getShells()
             | ranges::views::transform(
                   [](const Shell& shell)
                   {
                       return shell.points;
                   })
             | ranges::views::join;
    }

    void exportTo(const std::string& filename) const
    {
        PolygonMesh exported_mesh;

        for (const Point_3& point : getAllShellPoints())
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

    void makeGrowCandidates(const std::vector<Vector_3>& grow_deltas)
    {
#warning Maybe we dont need to generate them all beforehand
        // const std::vector<Point_3>& outer_shell = grown_shells_.back().points;
        // growable_points_.clear();
        // growable_points_.reserve(outer_shell.size());
        // for (const Point_3& last_grown_points : outer_shell)
        // {
        //     GrowingPoint growing_point;
        //
        //     for (const Vector_3& grow_delta : grow_deltas)
        //     {
        //         growing_point.grow_candidates.push_back({ .position = last_grown_points + grow_delta });
        //     }
        //
        //     growable_points_.push_back(growing_point);
        // }
    }

    static void doWatershed(std::vector<std::shared_ptr<GrowingPointsCloud>>& points_clouds, const CGAL::Bbox_3& boundaries)
    {
        std::vector<Vector_3> grow_deltas = makeGrowDeltas();

        bool point_grown;

        size_t iteration = 0;
        do
        {
            point_grown = false;

            std::map<std::shared_ptr<GrowingPointsCloud>, std::tuple<std::vector<Point_3>, std::vector<Point_3>>> grow_results;

            for (auto [index, points_cloud] : points_clouds | ranges::views::enumerate)
            {
                points_cloud->exportTo(fmt::format("points_cloud_{}_it_{}", index, iteration));
                spdlog::info("Apply growing for ex {}", points_cloud->getExtruderNr());
                grow_results[points_cloud] = points_cloud->evaluateGrowing(points_clouds, boundaries, grow_deltas);
                spdlog::info("Ended applying growing");
            }


            for (auto iterator = grow_results.begin(); iterator != grow_results.end(); ++iterator)
            {
                const std::shared_ptr<GrowingPointsCloud>& points_cloud = iterator->first;
                std::tuple<std::vector<Point_3>, std::vector<Point_3>>& grow_result = iterator->second;
                spdlog::info("Apply growing for ex {}", points_cloud->getExtruderNr());
                point_grown |= points_cloud->applyGrowing(std::move(std::get<0>(grow_result)), std::get<1>(grow_result));
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
    GrowCapacity evaluateGrowCandidate(const Point_3& grow_position, const std::vector<std::shared_ptr<GrowingPointsCloud>>& points_clouds, const CGAL::Bbox_3& boundaries) const
    {
        if (grow_position.x() < boundaries.xmin() || grow_position.x() > boundaries.xmax() || grow_position.y() < boundaries.ymin() || grow_position.y() > boundaries.ymax()
            || grow_position.z() < boundaries.zmin() || grow_position.z() > boundaries.zmax())
        {
            return GrowCapacity::BlockedByDifferent;
        }

        GrowCapacity grow_capacity = GrowCapacity::CanGrow;

        Fuzzy_Sphere lookup_sphere_similar(grow_position, GrowingPointsCloud::grow_radius * 0.95, 0);
        Fuzzy_Sphere lookup_sphere_different(grow_position, GrowingPointsCloud::grow_radius * sqrt(2) * 1.01, 0);

        for (const std::shared_ptr<GrowingPointsCloud>& points_cloud : points_clouds)
        {
            const bool same_extruder = points_cloud->getExtruderNr() == getExtruderNr();

            for (const Shell* shell : ranges::views::concat(
                     points_cloud->grown_shells_
                         | ranges::views::transform(
                             [](const Shell& s)
                             {
                                 return &s;
                             }),
                     ranges::views::single(&points_cloud->contour_)))
            // for (const Shell& shell : points_cloud->getShells())
            {
                if (shell->points.empty())
                {
                    continue;
                }

                // Fuzzy_Sphere lookup_sphere(grow_position, GrowingPointsCloud::grow_radius * 0.95, 0);
                //  Fuzzy_Sphere lookup_sphere(grow_position, GrowingPointsCloud::grow_radius * sqrt(2) * 0.99, 0);
                //  std::optional<Point_3> result = shell.search_tree.search_any_point(lookup_sphere);

                const Fuzzy_Sphere lookup_sphere = same_extruder ? lookup_sphere_similar : lookup_sphere_different;

                std::vector<Point_3> nearby_points;
                shell->search_tree.search(std::back_inserter(nearby_points), lookup_sphere);

                // if (! nearby_points.empty() && ((points_cloud.get() != this || (&shell != &grown_shells_.back()) || nearby_points.size() > 1)))
                if (! nearby_points.empty())
                {
                    GrowCapacity new_capacity = same_extruder ? GrowCapacity::BlockedBySimilar : GrowCapacity::BlockedByDifferent;
                    grow_capacity = std::max(grow_capacity, new_capacity);

                    if (grow_capacity == GrowCapacity::BlockedByDifferent)
                    {
                        return grow_capacity;
                    }
                }

#if 0
                for (auto [index, shell_point] : shell.points | ranges::views::enumerate)
                {
                    if (points_cloud.get() == this && (&shell == &grown_shells_.back()) && (index == grow_candidate_index || grow_capacity == GrowCapacity::BlockedBySimilar))
                    {
                        // Ignore origin point, we are allowed to be close to it
                        continue;
                    }

                    const double distance = (shell_point - grow_position).squared_length();
                    if (distance < grow_radius_squared)
                    {
                        GrowCapacity new_capacity = points_cloud->getExtruderNr() == getExtruderNr() ? GrowCapacity::BlockedBySimilar : GrowCapacity::BlockedByDifferent;
                        grow_capacity = std::max(grow_capacity, new_capacity);

                        if (grow_capacity == GrowCapacity::BlockedByDifferent)
                        {
                            return grow_capacity;
                        }
                    }
                }
#endif
            }
        }

        return grow_capacity;
    }

    bool applyGrowing(std::vector<Point_3>&& new_shell, const std::vector<Point_3>& new_contour_points)
    {
        const bool has_grown_shell = ! new_shell.empty();
        pushShell(std::move(new_shell));

        // Remove inner shells, we don't need them anymore
        while (grown_shells_.size() > 2)
        {
            grown_shells_.pop_front();
        }

        expandContour(new_contour_points);

        contour_.search_tree.build();

        return has_grown_shell;
    }

    std::tuple<std::vector<Point_3>, std::vector<Point_3>>
        evaluateGrowing(const std::vector<std::shared_ptr<GrowingPointsCloud>>& points_clouds, const CGAL::Bbox_3& boundaries, const std::vector<Vector_3>& grow_deltas)
    {
        const std::vector<Point_3>& outer_shell = grown_shells_.back().points;
        std::vector<Point_3> new_shell;
        std::vector<Point_3> new_contour_points;

        spdlog::info("Start evaluating {} points cloud candidates for ex {}", (outer_shell.size() * grow_deltas.size()), getExtruderNr());

        std::mutex mutex;
        cura::parallel_for(
            outer_shell,
            [&](auto iterator)
            {
                const Point_3& outer_shell_point = *iterator;
                std::vector<GrowingCandidate> growing_candidates;

                for (const Vector_3& grow_delta : grow_deltas)
                {
                    GrowingCandidate grow_candidate;
                    grow_candidate.position = outer_shell_point + grow_delta;
                    grow_candidate.capacity = evaluateGrowCandidate(grow_candidate.position, points_clouds, boundaries);
                    growing_candidates.push_back(grow_candidate);
                }

                bool point_is_contour = false;
                for (const GrowingCandidate& grow_candidate : growing_candidates)
                {
                    switch (grow_candidate.capacity)
                    {
                    case GrowCapacity::CanGrow:
                        mutex.lock();
                        new_shell.push_back(grow_candidate.position);
                        mutex.unlock();
                        break;

                    case GrowCapacity::BlockedBySimilar:
                        // Point has grown inside or is too close to other shell points, just discard it
                        break;

                    case GrowCapacity::BlockedByDifferent:
                        // Point has reached a different points cloud, discard it and tag origin point as being part of the contour
                        point_is_contour = true;
                        break;

                    case GrowCapacity::Unknown:
                        // This should never happen
                        break;
                    }
                }

                if (point_is_contour)
                {
                    mutex.lock();
                    new_contour_points.push_back(outer_shell_point);
                    mutex.unlock();
                }
            });

        return std::make_tuple(new_shell, new_contour_points);

        // cura::parallel_for(
        //     growable_points_,
        //     [&](auto iterator)
        //     {
        //         GrowingPoint& growing_point = *iterator;
        //         for (auto [index, grow_candidate] : growing_point.grow_candidates | ranges::views::enumerate)
        //         {
        //             grow_candidate.capacity = evaluateGrowCandidate(grow_candidate, index, points_clouds, boundaries);
        //         }
        //     });
    }

public:
    static constexpr coord_t grow_radius = 100;
    static constexpr coord_t grow_radius_squared = grow_radius * grow_radius;

private:
    const size_t extruder_nr_;
    std::deque<Shell> grown_shells_;
    Shell contour_;
};

void exportPointsClouds(const std::vector<std::shared_ptr<GrowingPointsCloud>>& points_clouds, const std::string& filename)
{
    PolygonMesh exported_mesh;
    for (const std::shared_ptr<GrowingPointsCloud>& points_cloud : points_clouds)
    {
        for (const Point_3& point : points_cloud->getAllShellPoints())
        {
            exported_mesh.add_vertex(point);
        }
    }
    exportMesh(exported_mesh, filename);
}

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

void makeModifierMeshWatershed(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image, PolygonMesh& output_mesh)
{
    std::vector<std::shared_ptr<GrowingPointsCloud>> points_clouds = makeInitialPointsCloudsFromTexture2(mesh, image);
    exportPointsClouds(points_clouds, "initial_points_cloud");

    const CGAL::Bbox_3 expanded_bounding_box = expand(CGAL::Polygon_mesh_processing::bbox(mesh), GrowingPointsCloud::grow_radius * 1.5);
    GrowingPointsCloud::doWatershed(points_clouds, expanded_bounding_box);

    const std::vector<Point_3>& contour = points_clouds[1]->getContour();
    exportPointsCloud(contour, "final_contour");
    makeMeshFromPointsCloud(contour, output_mesh, GrowingPointsCloud::grow_radius);
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
    png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/texture-high.png");
    // png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/dino-texture.png");

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
    makeModifierMeshWatershed(converted_mesh, image, output_mesh);
    registerModifiedMesh(meshgroup, output_mesh);

    exportMesh(converted_mesh, "converted_mesh");
    exportMesh(output_mesh, "output_mesh");
}

} // namespace cura::MeshMaterialSplitter