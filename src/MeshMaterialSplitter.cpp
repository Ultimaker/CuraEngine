// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <CGAL/config.h>
#define CGAL_CAN_USE_CXX20_FORMAT 0

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <execution>

#include <boost/geometry.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/unordered/concurrent_flat_map.hpp>
#include <boost/unordered/concurrent_flat_set.hpp>
#include <boost/unordered/unordered_flat_set.hpp>
#include <range/v3/algorithm/max.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/map.hpp>
#include <spdlog/spdlog.h>

#include "MeshGroup.h"
#include "Slice.h"
#include "TextureDataProvider.h"
#include "geometry/Shape.h"
#include "mesh.h"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"


using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Triangle_3 = Kernel::Triangle_3;

using Point_2 = Kernel::Point_2;
using Triangle_2 = Kernel::Triangle_2;

using PolygonMesh = CGAL::Surface_mesh<Kernel::Point_3>;

using Kernel_U32 = CGAL::Simple_cartesian<uint32_t>;
using Point_2U32 = Kernel_U32::Point_2;

struct Pixel3D
{
    Point_3 position;
    uint8_t occupation;

    double x() const
    {
        return position.x();
    }
    double y() const
    {
        return position.y();
    }
    double z() const
    {
        return position.z();
    }

    void dummySetPos(double)
    {
    }
};

BOOST_GEOMETRY_REGISTER_POINT_3D_GET_SET(Pixel3D, double, boost::geometry::cs::cartesian, x, y, z, dummySetPos, dummySetPos, dummySetPos);

using Boost_Point3D = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;
using Boost_RTree = boost::geometry::index::rtree<Pixel3D, boost::geometry::index::quadratic<8>>;

namespace cura::MeshMaterialSplitter
{

Point_2U32 getPixelCoordinates(const Point_2& uv_coordinates, const std::shared_ptr<Image>& image)
{
    std::pair<size_t, size_t> pixel_coordinates = image->getPixelCoordinates(Point2F(uv_coordinates.x(), uv_coordinates.y()));
    return Point_2U32(pixel_coordinates.first, pixel_coordinates.second);
}

std::optional<Point_3> getBarycentricCoordinates(const Point_3& point, const Point_3& p0, const Point_3& p1, const Point_3& p2)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const Vector_3 v0(p1 - p0);
    const Vector_3 v1(p2 - p0);
    const Vector_3 v2(point - p0);

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

    // Return as a Point_3 where x/y/z represent the barycentric coordinates u/v/w
    return Point_3(u, v, w);
}

Point_2 getUVCoordinates(const Point_3& barycentric_coordinates, const Triangle_2& face_uv_coordinates)
{
    return Point_2(
        (face_uv_coordinates[2].x() * barycentric_coordinates.x()) + (face_uv_coordinates[0].x() * barycentric_coordinates.y())
            + (face_uv_coordinates[1].x() * barycentric_coordinates.z()),
        (face_uv_coordinates[2].y() * barycentric_coordinates.x()) + (face_uv_coordinates[0].y() * barycentric_coordinates.y())
            + (face_uv_coordinates[1].y() * barycentric_coordinates.z()));
}

Triangle_3 getFaceTriangle(const PolygonMesh& mesh, CGAL::SM_Face_index face)
{
    CGAL::SM_Halfedge_index h = mesh.halfedge(face);
    Point_3 point_A = mesh.point(mesh.source(h));
    h = mesh.next(h);
    Point_3 point_B = mesh.point(mesh.source(h));
    Point_3 point_C = mesh.point(mesh.target(h));

    return Triangle_3(point_A, point_B, point_C);
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

/*!
 * The ParameterizedSegment is a helper to quickly calculate the voxels traversed by a triangle. Is allows intersecting the segment with two planes in the X or Y direction.
 */
class ParameterizedSegment
{
public:
    ParameterizedSegment(const Point_3& start, const Point_3& end)
        : direction_(end - start)
        , start_(start)
        , end_(end)
    {
    }

    const Point_3& start() const
    {
        return start_;
    }

    const Point_3& end() const
    {
        return end_;
    }

    Point_3 pointAtX(const double x) const
    {
        const double factor = (x - start_.x()) / (direction_.x());
        return Point_3(x, start_.y() + factor * direction_.y(), start_.z() + factor * direction_.z());
    }

    Point_3 pointAtY(const double y) const
    {
        const double factor = (y - start_.y()) / (direction_.y());
        return Point_3(start_.x() + factor * direction_.x(), y, start_.z() + factor * direction_.z());
    }

    std::optional<ParameterizedSegment> croppedSegmentX(const double layer_start, const double layer_end, const Point_3& p1, const Point_3& p2) const
    {
        if (p1.x() <= layer_end && p2.x() >= layer_start)
        {
            return ParameterizedSegment(p1.x() < layer_start ? pointAtX(layer_start) : p1, p2.x() > layer_end ? pointAtX(layer_end) : p2);
        }

        return std::nullopt;
    }

    std::optional<ParameterizedSegment> intersectionWithXLayer(const double layer_start, const double layer_end) const
    {
        if (direction_.x() > 0)
        {
            return croppedSegmentX(layer_start, layer_end, start_, end_);
        }

        if (direction_.x() < 0)
        {
            return croppedSegmentX(layer_start, layer_end, end_, start_);
        }

        if (start_.x() >= layer_start && start_.x() <= layer_end)
        {
            return *this;
        }

        return std::nullopt;
    }

    std::optional<ParameterizedSegment> croppedSegmentY(const double layer_start, const double layer_end, const Point_3& p1, const Point_3& p2) const
    {
        if (p1.y() <= layer_end && p2.y() >= layer_start)
        {
            return ParameterizedSegment(p1.y() < layer_start ? pointAtY(layer_start) : p1, p2.y() > layer_end ? pointAtY(layer_end) : p2);
        }

        return std::nullopt;
    }

    std::optional<ParameterizedSegment> intersectionWithYLayer(const double layer_start, const double layer_end) const
    {
        if (direction_.y() > 0)
        {
            return croppedSegmentY(layer_start, layer_end, start_, end_);
        }

        if (direction_.y() < 0)
        {
            return croppedSegmentY(layer_start, layer_end, end_, start_);
        }

        if (start_.y() >= layer_start && start_.y() <= layer_end)
        {
            return *this;
        }

        return std::nullopt;
    }

private:
    Vector_3 direction_;
    Point_3 start_;
    Point_3 end_;
};

class VoxelGrid
{
public:
    struct SimplePoint_3U16
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
        uint16_t black_hole{ 0 }; // Don't place anything in there, or it would be lost forever (it exists only to properly set the 4th byte of the key)
    };

    struct SimplePoint_3S16
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    union LocalCoordinates
    {
        // This union allows to store a position in the voxels grid by XYZ position stored on UINT16 values, and also to use this position as a key that can
        // be stored in a map, without having to separately hash the X, Y and Z components, which saves some time when heavily dealing with large maps.
        uint64_t key{ 0 };
        SimplePoint_3U16 position;

        LocalCoordinates(const uint16_t x, const uint16_t y, const uint16_t z)
            : position{ x, y, z }
        {
        }

        bool operator==(const LocalCoordinates& other) const
        {
            return key == other.key;
        }
        bool operator!=(const LocalCoordinates& other) const
        {
            return key != other.key;
        }
        bool operator<(const LocalCoordinates& other) const
        {
            return key < other.key;
        }
    };

    using KeyType = uint64_t;
    static constexpr uint8_t nb_voxels_around = 3 * 3 * 3 - 1;

public:
    explicit VoxelGrid(const CGAL::Bbox_3& bounding_box, const coord_t max_resolution)
    {
        const CGAL::Bbox_3 expanded_bounding_box = expand(bounding_box, max_resolution * 2);
        origin_ = Vector_3(expanded_bounding_box.xmin(), expanded_bounding_box.ymin(), expanded_bounding_box.zmin());
        resolution_ = Point_3(expanded_bounding_box.x_span(), expanded_bounding_box.y_span(), expanded_bounding_box.z_span());

        double actual_max_resolution;
        max_coordinate_ = 1;
        do
        {
            max_coordinate_ <<= 1;
            resolution_ = Point_3(resolution_.x() / 2.0, resolution_.y() / 2.0, resolution_.z() / 2.0);
            actual_max_resolution = std::max({ resolution_.x(), resolution_.y(), resolution_.z() });
        } while (actual_max_resolution > max_resolution);
    }

    const Point_3& getResolution() const
    {
        return resolution_;
    }

    Point_3 toGlobalCoordinates(const LocalCoordinates& position, const bool at_center = false) const
    {
        return Point_3(toGlobalX(position.position.x, at_center), toGlobalY(position.position.y, at_center), toGlobalZ(position.position.z, at_center));
    }

    void setOccupation(const LocalCoordinates& position, const uint8_t extruder_nr)
    {
        occupied_voxels_.insert_or_assign(position, extruder_nr);
    }

    void setOrUpdateOccupation(const LocalCoordinates& position, const uint8_t extruder_nr)
    {
        occupied_voxels_.insert_or_visit(
            { position, extruder_nr },
            [extruder_nr](auto& voxel)
            {
                voxel.second = std::min(voxel.second, extruder_nr);
            });
    }

    std::optional<uint8_t> getOccupation(const LocalCoordinates& local_position) const
    {
        std::optional<uint8_t> result = std::nullopt;
        occupied_voxels_.cvisit(
            local_position,
            [&result](const auto& occupation)
            {
                result = occupation.second;
            });
        return result;
    }

    template<class... Args>
    void visitOccupiedVoxels(Args&&... args)
    {
        occupied_voxels_.visit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            args...);
    }

    template<class... Args>
    void visitOccupiedVoxels(Args&&... args) const
    {
        occupied_voxels_.cvisit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            args...);
    }

    std::vector<LocalCoordinates> getVoxelsAround(const LocalCoordinates& point) const
    {
        const SimplePoint_3U16& position = point.position;
        std::vector<LocalCoordinates> voxels_around;
        voxels_around.reserve(nb_voxels_around);

        for (int8_t delta_x = -1; delta_x < 2; ++delta_x)
        {
            const int64_t pos_x = position.x + delta_x;
            if (pos_x < 0 || pos_x >= max_coordinate_)
            {
                continue;
            }

            for (int8_t delta_y = -1; delta_y < 2; ++delta_y)
            {
                const int64_t pos_y = position.y + delta_y;
                if (pos_y < 0 || pos_y >= max_coordinate_)
                {
                    continue;
                }

                for (int8_t delta_z = -1; delta_z < 2; ++delta_z)
                {
                    const int64_t pos_z = position.z + delta_z;
                    if (pos_z < 0 || pos_z >= max_coordinate_)
                    {
                        continue;
                    }

                    if (delta_x || delta_y || delta_z)
                    {
                        voxels_around.push_back(LocalCoordinates(pos_x, pos_y, pos_z));
                    }
                }
            }
        }

        return voxels_around;
    }

    LocalCoordinates toLocalCoordinates(const Point_3& position) const
    {
        return LocalCoordinates(toLocalX(position.x()), toLocalY(position.y()), toLocalZ(position.z()));
    }

    uint16_t toLocalX(const double x) const
    {
        return (x - origin_.x()) / resolution_.x();
    }

    double toGlobalX(const uint16_t x, const bool at_center = false) const
    {
        return (x * resolution_.x()) + origin_.x() + (at_center ? resolution_.x() / 2.0 : 0.0);
    }

    uint16_t toLocalY(const double y) const
    {
        return (y - origin_.y()) / resolution_.y();
    }

    double toGlobalY(const uint16_t y, const bool at_center = false) const
    {
        return (y * resolution_.y()) + origin_.y() + (at_center ? resolution_.y() / 2.0 : 0.0);
    }

    uint16_t toLocalZ(const double z) const
    {
        return (z - origin_.z()) / resolution_.z();
    }

    double toGlobalZ(const uint16_t z, const bool at_center = false) const
    {
        return (z * resolution_.z()) + origin_.z() + (at_center ? resolution_.z() / 2.0 : 0.0);
    }

    /*!
     * @brief Gets all the voxels traversed by the given triangle, which is similar to rasterizing the triangle, but in 3D
     * @param triangle The 3D triangle we want to rasterize
     * @return The list of voxels traversed by the triangle
     *
     * To do a fast (enough) rasterization of the triangle, we calculate its bounding box in the voxels grid on the X axis, then we iterate on all the YZ columns
     * in the X direction. For each column we crop the edges of the triangle to fit inside the width of the column, which gives a sub-triangle or a quad shape. Then for this
     * sub-shape, we calculate its bounding box in the Y axis, and iterate on all the Z "square tubes". For each cube we crop the edges again, and iterate over the Z bounding boxes
     * of the sub-sub-shape. Each iterated position is then considered as being traversed by the triangle.
     */
    std::vector<LocalCoordinates> getTraversedVoxels(const Triangle_3& triangle) const
    {
        const SimplePoint_3U16 p0 = toLocalCoordinates(triangle[0]).position;
        const SimplePoint_3U16 p1 = toLocalCoordinates(triangle[1]).position;
        const SimplePoint_3U16 p2 = toLocalCoordinates(triangle[2]).position;

        const ParameterizedSegment s1(triangle[0], triangle[1]);
        const ParameterizedSegment s2(triangle[1], triangle[2]);
        const ParameterizedSegment s3(triangle[2], triangle[0]);

        std::vector<LocalCoordinates> traversed_voxels;

        const uint16_t xmin = std::min({ p0.x, p1.x, p2.x });
        const uint16_t xmax = std::max({ p0.x, p1.x, p2.x });
        for (uint16_t x = xmin; x <= xmax; ++x)
        {
            const double layer_start_x = toGlobalX(x);
            const double layer_end_x = toGlobalX(x + 1);
            const std::optional<ParameterizedSegment> s1_inter_x = s1.intersectionWithXLayer(layer_start_x, layer_end_x);
            const std::optional<ParameterizedSegment> s2_inter_x = s2.intersectionWithXLayer(layer_start_x, layer_end_x);
            const std::optional<ParameterizedSegment> s3_inter_x = s3.intersectionWithXLayer(layer_start_x, layer_end_x);

            std::vector<double> y_values;
            for (const std::optional<ParameterizedSegment>& inter_x : { s1_inter_x, s2_inter_x, s3_inter_x })
            {
                if (inter_x.has_value())
                {
                    y_values.push_back(inter_x.value().start().y());
                    y_values.push_back(inter_x.value().end().y());
                }
            }

            if (y_values.empty())
            {
                continue;
            }

            const uint16_t ymin = toLocalY(ranges::min(y_values));
            const uint16_t ymax = toLocalY(ranges::max(y_values));

            for (uint16_t y = ymin; y <= ymax; ++y)
            {
                const double layer_start_y = toGlobalY(y);
                const double layer_end_y = toGlobalY(y + 1);
                const std::optional<ParameterizedSegment> s1_inter_y = s1.intersectionWithYLayer(layer_start_y, layer_end_y);
                const std::optional<ParameterizedSegment> s2_inter_y = s2.intersectionWithYLayer(layer_start_y, layer_end_y);
                const std::optional<ParameterizedSegment> s3_inter_y = s3.intersectionWithYLayer(layer_start_y, layer_end_y);

                std::vector<double> z_values;
                for (const std::optional<ParameterizedSegment>& inter_y : { s1_inter_y, s2_inter_y, s3_inter_y })
                {
                    if (inter_y.has_value())
                    {
                        z_values.push_back(inter_y.value().start().z());
                        z_values.push_back(inter_y.value().end().z());
                    }
                }

                if (z_values.empty())
                {
                    continue;
                }

                const uint16_t zmin = toLocalZ(ranges::min(z_values));
                const uint16_t zmax = toLocalZ(ranges::max(z_values));

                for (uint16_t z = zmin; z <= zmax; ++z)
                {
                    traversed_voxels.push_back(LocalCoordinates(x, y, z));
                }
            }
        }

        return traversed_voxels;
    }

private:
    Point_3 resolution_;
    Vector_3 origin_;
    uint32_t max_coordinate_;
    boost::concurrent_flat_map<LocalCoordinates, uint8_t> occupied_voxels_;
};

std::size_t hash_value(VoxelGrid::LocalCoordinates const& position)
{
    return boost::hash<uint64_t>()(position.key);
}

bool makeInitialVoxelSpaceFromTexture(const PolygonMesh& mesh, const std::shared_ptr<TextureDataProvider>& texture_data_provider, VoxelGrid& voxel_space)
{
    const auto faces = mesh.faces();
    const auto uv_coords = mesh.property_map<CGAL::SM_Face_index, std::array<Point_2, 3>>("f:uv_coords").value();
    boost::concurrent_flat_set<uint8_t> found_extruders;

    run_multiple_producers_ordered_consumer(
        0,
        faces.size(),
        [&](const size_t face_index)
        {
            const CGAL::SM_Face_index face = *std::next(faces.begin(), face_index);
            const Triangle_2 face_uvs(uv_coords[face][0], uv_coords[face][1], uv_coords[face][2]);
            const Triangle_3 triangle = getFaceTriangle(mesh, face);

            for (const VoxelGrid::LocalCoordinates& traversed_voxel : voxel_space.getTraversedVoxels(triangle))
            {
                const Point_3 global_position = voxel_space.toGlobalCoordinates(traversed_voxel);
                const std::optional<Point_3> barycentric_coordinates = getBarycentricCoordinates(global_position, triangle[0], triangle[1], triangle[2]);
                if (! barycentric_coordinates.has_value() || barycentric_coordinates.value().x() < 0 || barycentric_coordinates.value().y() < 0
                    || barycentric_coordinates.value().z() < 0)
                {
                    // Triangle is invalid, or point is outside the triangle
                    continue;
                }

                const Point_2 point_uv_coords = getUVCoordinates(barycentric_coordinates.value(), face_uvs);
                const Point_2U32 pixel = getPixelCoordinates(point_uv_coords, texture_data_provider->getTexture());
                const std::optional<uint32_t> extruder_nr = texture_data_provider->getValue(pixel.x(), pixel.y(), "extruder");
                if (extruder_nr.has_value())
                {
                    voxel_space.setOrUpdateOccupation(traversed_voxel, extruder_nr.value());
                    found_extruders.insert(extruder_nr.value());
                }
            }

            return true;
        },
        [](bool result) {});

    if (found_extruders.size() == 1)
    {
        // We have found only one extruder in the texture, so return true only if this extruder is not 0, otherwise the whole splitting is useless
        bool is_non_zero = true;
        found_extruders.visit_all(
            [&is_non_zero](const uint8_t extruder_nr)
            {
                is_non_zero = extruder_nr != 0;
            });
        return is_non_zero;
    }

    return true;
}

struct LookupTree
{
    std::vector<Pixel3D> pixels_cloud;
    Boost_RTree boost_tree;
};

LookupTree makeLookupTreeFromVoxelGrid(VoxelGrid& voxel_grid)
{
    LookupTree lookup_tree;

    std::mutex mutex;
    voxel_grid.visitOccupiedVoxels(
        [&lookup_tree, &voxel_grid, &mutex](const auto& voxel)
        {
            const Point_3 global_coordinates = voxel_grid.toGlobalCoordinates(voxel.first);
            mutex.lock();
            lookup_tree.pixels_cloud.push_back(Pixel3D{ global_coordinates, voxel.second });
            mutex.unlock();
        });

    lookup_tree.boost_tree = Boost_RTree(lookup_tree.pixels_cloud.begin(), lookup_tree.pixels_cloud.end());

    return lookup_tree;
}

std::vector<Mesh> makeMeshesFromPointsClouds2(const VoxelGrid& voxel_grid)
{
    std::map<uint8_t, Mesh> meshes;

    using FaceIndices = std::array<size_t, 3>;
    using AddedFace = std::array<FaceIndices, 2>;
    std::vector<std::pair<VoxelGrid::SimplePoint_3S16, AddedFace>> faces_to_add;

    const double half_res_x = voxel_grid.getResolution().x() / 2;
    const double half_res_y = voxel_grid.getResolution().y() / 2;
    const double half_res_z = voxel_grid.getResolution().z() / 2;

    std::array<Point3LL, 8> cube_points;
    cube_points[0] = Point3LL(half_res_x, -half_res_y, -half_res_z);
    cube_points[1] = Point3LL(half_res_x, half_res_y, -half_res_z);
    cube_points[2] = Point3LL(half_res_x, half_res_y, half_res_z);
    cube_points[3] = Point3LL(half_res_x, -half_res_y, half_res_z);
    cube_points[4] = Point3LL(-half_res_x, -half_res_y, -half_res_z);
    cube_points[5] = Point3LL(-half_res_x, half_res_y, -half_res_z);
    cube_points[6] = Point3LL(-half_res_x, half_res_y, half_res_z);
    cube_points[7] = Point3LL(-half_res_x, -half_res_y, half_res_z);

    faces_to_add.push_back({ VoxelGrid::SimplePoint_3S16(1, 0, 0), { FaceIndices{ 0, 1, 2 }, FaceIndices{ 2, 3, 0 } } });
    faces_to_add.push_back({ VoxelGrid::SimplePoint_3S16(-1, 0, 0), { FaceIndices{ 5, 4, 7 }, FaceIndices{ 7, 6, 5 } } });
    faces_to_add.push_back({ VoxelGrid::SimplePoint_3S16(0, 1, 0), { FaceIndices{ 1, 5, 6 }, FaceIndices{ 6, 2, 1 } } });
    faces_to_add.push_back({ VoxelGrid::SimplePoint_3S16(0, -1, 0), { FaceIndices{ 4, 0, 3 }, FaceIndices{ 3, 7, 4 } } });

    boost::concurrent_flat_map<VoxelGrid::LocalCoordinates, uint8_t> occupied_voxels;
    voxel_grid.visitOccupiedVoxels(
        [&occupied_voxels](const auto& occupied_voxel)
        {
            if (occupied_voxel.second > 0)
            {
                occupied_voxels.emplace(occupied_voxel.first, occupied_voxel.second);
            }
        });

    std::mutex mutex;
    occupied_voxels.cvisit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&faces_to_add, &voxel_grid, &meshes, &mutex, &cube_points](const auto& occupied_voxel)
        {
            const Point_3 current_position = voxel_grid.toGlobalCoordinates(occupied_voxel.first, true);
            const Point3LL current_position_ll(current_position.x(), current_position.y(), current_position.z());

            for (const auto& face_to_add : faces_to_add)
            {
                int32_t x = static_cast<int32_t>(occupied_voxel.first.position.x) + face_to_add.first.x;
                int32_t y = static_cast<int32_t>(occupied_voxel.first.position.y) + face_to_add.first.y;
                int32_t z = static_cast<int32_t>(occupied_voxel.first.position.z) + face_to_add.first.z;

                uint8_t outer_occupation;
                if (x < 0 || x > std::numeric_limits<uint16_t>::max() || y < 0 || y > std::numeric_limits<uint16_t>::max() || z < 0 || z > std::numeric_limits<uint16_t>::max())
                {
                    outer_occupation = 0;
                }
                else
                {
                    outer_occupation = voxel_grid.getOccupation(VoxelGrid::LocalCoordinates(x, y, z)).value_or(occupied_voxel.second);
                }

                if (outer_occupation != occupied_voxel.second)
                {
                    mutex.lock();
                    auto mesh_iterator = meshes.find(occupied_voxel.second);
                    if (mesh_iterator == meshes.end())
                    {
                        Mesh mesh(Application::getInstance().current_slice_->scene.extruders.at(occupied_voxel.second).settings_);
                        mesh.settings_.add("cutting_mesh", "true");
                        mesh.settings_.add("extruder_nr", std::to_string(occupied_voxel.second));
                        meshes.insert({ occupied_voxel.second, mesh });
                    }

                    Mesh& mesh = meshes[occupied_voxel.second];
                    for (const FaceIndices& face_indices : face_to_add.second)
                    {
                        mesh.addFace(
                            current_position_ll + cube_points[face_indices[0]],
                            current_position_ll + cube_points[face_indices[1]],
                            current_position_ll + cube_points[face_indices[2]]);
                    }

                    mutex.unlock();
                }
            }
        });

    std::vector<Mesh> meshes_vec;
    for (Mesh& mesh : meshes | ranges::views::values)
    {
        meshes_vec.push_back(std::move(mesh));
    }
    return meshes_vec;
}

std::vector<Mesh> makeModifierMeshes(const PolygonMesh& mesh, const std::shared_ptr<TextureDataProvider>& texture_data_provider)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;

    // Fill a first voxel grid by rasterizing the triangles of the mesh in 3D, and assign the extruders according to the texture. This way we can later evaluate which extruder to
    // assign any point in 3D space just by finding the closest outside point and see what extruder it is assigned to.
    spdlog::debug("Fill original voxels based on texture data");
    CGAL::Bbox_3 bounding_box = CGAL::Polygon_mesh_processing::bbox(mesh);
    auto resolution = settings.get<coord_t>("multi_material_paint_resolution");
    VoxelGrid voxel_space(bounding_box, resolution);
    if (! makeInitialVoxelSpaceFromTexture(mesh, texture_data_provider, voxel_space))
    {
        // Texture is filled with 0s, don't bother doing anything
        return {};
    }

    spdlog::debug("Prepare AABB trees for fast look-up");
    LookupTree tree = makeLookupTreeFromVoxelGrid(voxel_space);

    const auto deepness = settings.get<coord_t>("multi_material_paint_deepness");
    const Point_3& spatial_resolution = voxel_space.getResolution();
    const coord_t deepness_squared = deepness * deepness;

    // Generate a clean and approximate version of the mesh by alpha-wrapping it, so that we can do proper and fast inside-mesh checking
    spdlog::debug("prepare alpha mesh");
    PolygonMesh alpha_mesh;
    constexpr double alpha = 2000.0;
    const double offset = resolution * 2.0;
    CGAL::alpha_wrap_3(mesh, alpha, offset, alpha_mesh);
    CGAL::Side_of_triangle_mesh<PolygonMesh, Kernel> inside_mesh(alpha_mesh);
    bool check_inside = true;

    spdlog::debug("Get initially filled voxels");
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> previously_evaluated_voxels;
    voxel_space.visitOccupiedVoxels(
        [&previously_evaluated_voxels](const auto& voxel)
        {
            if (voxel.second > 0)
            {
                previously_evaluated_voxels.insert(voxel.first);
            };
        });

    while (! previously_evaluated_voxels.empty())
    {
        boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> voxels_to_evaluate;
        std::atomic_bool keep_checking_inside(false);

        spdlog::debug("Finding voxels around {} voxels", previously_evaluated_voxels.size());

        // For each already-filled voxel, gather the voxels around it and evaluate them
        previously_evaluated_voxels.visit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            [&](const VoxelGrid::LocalCoordinates& previously_evaluated_voxel)
            {
                for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_space.getVoxelsAround(previously_evaluated_voxel))
                {
                    if (voxels_to_evaluate.contains(voxel_around))
                    {
                        // This voxel has already been registered for evaluation
                        continue;
                    }

                    const std::optional<uint8_t> occupation = voxel_space.getOccupation(voxel_around);
                    if (occupation.has_value())
                    {
                        // Voxel is already filled, don't evaluate it anyhow
                        continue;
                    }

                    bool evaluate_voxel;
                    if (check_inside)
                    {
                        evaluate_voxel = inside_mesh(voxel_space.toGlobalCoordinates(voxel_around)) != CGAL::ON_UNBOUNDED_SIDE;
                        if (! evaluate_voxel)
                        {
                            voxel_space.setOccupation(voxel_around, 0);

                            // As long as we find voxels outside the mesh, keep checking for it. Once we have no single candidate outside, this means the outer shell
                            // is complete and we are only growing inside, thus we can skip checking for insideness
                            keep_checking_inside.store(true);
                        }
                    }
                    else
                    {
                        evaluate_voxel = true;
                    }

                    if (evaluate_voxel)
                    {
                        voxels_to_evaluate.emplace(voxel_around);
                    }
                }
            });

        if (check_inside && ! keep_checking_inside.load())
        {
            spdlog::debug("Stop checking for voxels insideness");
            check_inside = false;
        }

        // Now actually evaluate the candidate voxels, i.e. find their closest outside point and set the according occupation
        spdlog::debug("Evaluating {} voxels", voxels_to_evaluate.size());
        voxels_to_evaluate.visit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            [&voxel_space, &tree, &deepness_squared](const VoxelGrid::LocalCoordinates& voxel_to_evaluate)
            {
                const Point_3 position = voxel_space.toGlobalCoordinates(voxel_to_evaluate);

                // Define a query point
                Boost_Point3D query_point = { position.x(), position.y(), position.z() };

                // Find the nearest neighbor
                std::vector<Pixel3D> nearest_neighbors;
                tree.boost_tree.query(boost::geometry::index::nearest(query_point, 1), std::back_inserter(nearest_neighbors));

                if (! nearest_neighbors.empty())
                {
                    const Pixel3D& closest_neighbor = nearest_neighbors.front();
                    const Vector_3 diff = position - closest_neighbor.position;
                    const uint8_t new_occupation = diff.squared_length() <= deepness_squared ? closest_neighbor.occupation : 0;
                    voxel_space.setOccupation(voxel_to_evaluate, new_occupation);
                }
                else
                {
                    voxel_space.setOccupation(voxel_to_evaluate, 0);
                }
            });

        // Now we have evaluated the candidates, check which of them are to be processed next. We skip all the voxels that have only voxels with similar occupations around
        // them, because they are obviously not part of the boundaries we are looking for. This avoids filling the inside of the points cloud and speeds up calculation a lot.
        spdlog::debug("Find boundary voxels for next round");
        previously_evaluated_voxels.clear();
        voxels_to_evaluate.visit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            [&voxel_space, &previously_evaluated_voxels](const VoxelGrid::LocalCoordinates& evaluated_voxel)
            {
                bool has_various_voxels_around = false;
                uint8_t actual_occupation = voxel_space.getOccupation(evaluated_voxel).value();
                for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_space.getVoxelsAround(evaluated_voxel))
                {
                    std::optional<uint8_t> around_occupation = voxel_space.getOccupation(voxel_around);
                    if (around_occupation.has_value() && around_occupation.value() != actual_occupation)
                    {
                        has_various_voxels_around = true;
                        break;
                    }
                }

                if (has_various_voxels_around)
                {
                    previously_evaluated_voxels.emplace(evaluated_voxel);
                }
            });
    }

    spdlog::debug("Make meshes from points clouds");
    return makeMeshesFromPointsClouds2(voxel_space);
}

void makeMaterialModifierMeshes(Mesh& mesh, MeshGroup* meshgroup)
{
    if (mesh.texture_ == nullptr || mesh.texture_data_mapping_ == nullptr || ! mesh.texture_data_mapping_->contains("extruder"))
    {
        return;
    }

    spdlog::stopwatch timer;

    PolygonMesh converted_mesh;
    for (const MeshVertex& vertex : mesh.vertices_)
    {
        converted_mesh.add_vertex(Point_3(vertex.p_.x_, vertex.p_.y_, vertex.p_.z_));
    }

    bool has_uvs = false;
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
                has_uvs = true;
            }
            else
            {
                face_uvs[j] = Point_2(-1.0, -1.0);
            }
        }

        uv_coords[face_index] = face_uvs;
    }

    if (! has_uvs)
    {
        return;
    }

    spdlog::info("Start multi-material mesh generation");

    const auto texture_data_provider = std::make_shared<TextureDataProvider>(nullptr, mesh.texture_, mesh.texture_data_mapping_);

    for (const Mesh& mesh : makeModifierMeshes(converted_mesh, texture_data_provider))
    {
        meshgroup->meshes.push_back(mesh);
    }

    spdlog::info("Multi-material mesh generation took {} seconds", timer.elapsed().count());
}

} // namespace cura::MeshMaterialSplitter
