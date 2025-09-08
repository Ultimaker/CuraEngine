// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <execution>
#include <unordered_set>

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
#include "geometry/ClosedLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Shape.h"
#include "geometry/Triangle2F.h"
#include "geometry/Triangle3D.h"
#include "mesh.h"
#include "slicer.h"
#include "utils/OpenPolylineStitcher.h"
#include "utils/Point3D.h"
#include "utils/Simplify.h"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"


struct Pixel3D
{
    cura::Point3D position;
    uint8_t occupation;

    double x() const
    {
        return position.x_;
    }
    double y() const
    {
        return position.y_;
    }
    double z() const
    {
        return position.z_;
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

std::optional<Point3D> getBarycentricCoordinates(const Point3D& point, const Point3D& p0, const Point3D& p1, const Point3D& p2)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const Point3D v0(p1 - p0);
    const Point3D v1(p2 - p0);
    const Point3D v2(point - p0);

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

Point2F getUVCoordinates(const Point3D& barycentric_coordinates, const Triangle2F& face_uv_coordinates)
{
    return Point2F(
        (face_uv_coordinates[0].x_ * barycentric_coordinates.x_) + (face_uv_coordinates[1].x_ * barycentric_coordinates.y_)
            + (face_uv_coordinates[2].x_ * barycentric_coordinates.z_),
        (face_uv_coordinates[0].y_ * barycentric_coordinates.x_) + (face_uv_coordinates[1].y_ * barycentric_coordinates.y_)
            + (face_uv_coordinates[2].y_ * barycentric_coordinates.z_));
}

/*!
 * The ParameterizedSegment is a helper to quickly calculate the voxels traversed by a triangle. Is allows intersecting the segment with two planes in the X or Y direction.
 */
class ParameterizedSegment
{
public:
    ParameterizedSegment(const Point3D& start, const Point3D& end)
        : direction_(end - start)
        , start_(start)
        , end_(end)
    {
    }

    const Point3D& start() const
    {
        return start_;
    }

    const Point3D& end() const
    {
        return end_;
    }

    Point3D pointAtX(const double x) const
    {
        const double factor = (x - start_.x_) / (direction_.x_);
        return Point3D(x, start_.y_ + factor * direction_.y_, start_.z_ + factor * direction_.z_);
    }

    Point3D pointAtY(const double y) const
    {
        const double factor = (y - start_.y_) / (direction_.y_);
        return Point3D(start_.x_ + factor * direction_.x_, y, start_.z_ + factor * direction_.z_);
    }

    std::optional<ParameterizedSegment> croppedSegmentX(const double layer_start, const double layer_end, const Point3D& p1, const Point3D& p2) const
    {
        if (p1.x_ <= layer_end && p2.x_ >= layer_start)
        {
            return ParameterizedSegment(p1.x_ < layer_start ? pointAtX(layer_start) : p1, p2.x_ > layer_end ? pointAtX(layer_end) : p2);
        }

        return std::nullopt;
    }

    std::optional<ParameterizedSegment> intersectionWithXLayer(const double layer_start, const double layer_end) const
    {
        if (direction_.x_ > 0)
        {
            return croppedSegmentX(layer_start, layer_end, start_, end_);
        }

        if (direction_.x_ < 0)
        {
            return croppedSegmentX(layer_start, layer_end, end_, start_);
        }

        if (start_.x_ >= layer_start && start_.x_ <= layer_end)
        {
            return *this;
        }

        return std::nullopt;
    }

    std::optional<ParameterizedSegment> croppedSegmentY(const double layer_start, const double layer_end, const Point3D& p1, const Point3D& p2) const
    {
        if (p1.y_ <= layer_end && p2.y_ >= layer_start)
        {
            return ParameterizedSegment(p1.y_ < layer_start ? pointAtY(layer_start) : p1, p2.y_ > layer_end ? pointAtY(layer_end) : p2);
        }

        return std::nullopt;
    }

    std::optional<ParameterizedSegment> intersectionWithYLayer(const double layer_start, const double layer_end) const
    {
        if (direction_.y_ > 0)
        {
            return croppedSegmentY(layer_start, layer_end, start_, end_);
        }

        if (direction_.y_ < 0)
        {
            return croppedSegmentY(layer_start, layer_end, end_, start_);
        }

        if (start_.y_ >= layer_start && start_.y_ <= layer_end)
        {
            return *this;
        }

        return std::nullopt;
    }

private:
    Point3D direction_;
    Point3D start_;
    Point3D end_;
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
    explicit VoxelGrid(const AABB3D& bounding_box, const coord_t max_resolution)
    {
        origin_ = Point3D(bounding_box.min_.x_, bounding_box.min_.y_, bounding_box.min_.z_);
        resolution_ = Point3D(bounding_box.spanX(), bounding_box.spanY(), bounding_box.spanZ());

        double actual_max_resolution;
        max_coordinate_ = 1;
        do
        {
            max_coordinate_ <<= 1;
            resolution_ = Point3D(resolution_.x_ / 2.0, resolution_.y_ / 2.0, resolution_.z_ / 2.0);
            actual_max_resolution = std::max({ resolution_.x_, resolution_.y_, resolution_.z_ });
        } while (actual_max_resolution > max_resolution);
    }

    const Point3D& getResolution() const
    {
        return resolution_;
    }

    uint32_t getMaxCoordinates() const
    {
        return max_coordinate_;
    }

    Point3D toGlobalCoordinates(const LocalCoordinates& position, const bool at_center = true) const
    {
        return Point3D(toGlobalX(position.position.x, at_center), toGlobalY(position.position.y, at_center), toGlobalZ(position.position.z, at_center));
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

    LocalCoordinates toLocalCoordinates(const Point3D& position) const
    {
        return LocalCoordinates(toLocalX(position.x_), toLocalY(position.y_), toLocalZ(position.z_));
    }

    uint16_t toLocalX(const double x) const
    {
        return (x - origin_.x_) / resolution_.x_;
    }

    double toGlobalX(const uint16_t x, const bool at_center = true) const
    {
        return (x * resolution_.x_) + origin_.x_ + (at_center ? resolution_.x_ / 2.0 : 0.0);
    }

    uint16_t toLocalY(const double y) const
    {
        return (y - origin_.y_) / resolution_.y_;
    }

    double toGlobalY(const uint16_t y, const bool at_center = true) const
    {
        return (y * resolution_.y_) + origin_.y_ + (at_center ? resolution_.y_ / 2.0 : 0.0);
    }

    uint16_t toLocalZ(const double z) const
    {
        return (z - origin_.z_) / resolution_.z_;
    }

    double toGlobalZ(const uint16_t z, const bool at_center = true) const
    {
        return (z * resolution_.z_) + origin_.z_ + (at_center ? resolution_.z_ / 2.0 : 0.0);
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
    std::vector<LocalCoordinates> getTraversedVoxels(const Triangle3D& triangle) const
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
            const double layer_start_x = toGlobalX(x, false);
            const double layer_end_x = toGlobalX(x + 1, false);
            const std::optional<ParameterizedSegment> s1_inter_x = s1.intersectionWithXLayer(layer_start_x, layer_end_x);
            const std::optional<ParameterizedSegment> s2_inter_x = s2.intersectionWithXLayer(layer_start_x, layer_end_x);
            const std::optional<ParameterizedSegment> s3_inter_x = s3.intersectionWithXLayer(layer_start_x, layer_end_x);

            std::vector<double> y_values;
            for (const std::optional<ParameterizedSegment>& inter_x : { s1_inter_x, s2_inter_x, s3_inter_x })
            {
                if (inter_x.has_value())
                {
                    y_values.push_back(inter_x.value().start().y_);
                    y_values.push_back(inter_x.value().end().y_);
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
                const double layer_start_y = toGlobalY(y, false);
                const double layer_end_y = toGlobalY(y + 1, false);
                const std::optional<ParameterizedSegment> s1_inter_y = s1.intersectionWithYLayer(layer_start_y, layer_end_y);
                const std::optional<ParameterizedSegment> s2_inter_y = s2.intersectionWithYLayer(layer_start_y, layer_end_y);
                const std::optional<ParameterizedSegment> s3_inter_y = s3.intersectionWithYLayer(layer_start_y, layer_end_y);

                std::vector<double> z_values;
                for (const std::optional<ParameterizedSegment>& inter_y : { s1_inter_y, s2_inter_y, s3_inter_y })
                {
                    if (inter_y.has_value())
                    {
                        z_values.push_back(inter_y.value().start().z_);
                        z_values.push_back(inter_y.value().end().z_);
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
    Point3D resolution_;
    Point3D origin_;
    uint32_t max_coordinate_;
    boost::concurrent_flat_map<LocalCoordinates, uint8_t> occupied_voxels_;
};

std::size_t hash_value(VoxelGrid::LocalCoordinates const& position)
{
    return boost::hash<uint64_t>()(position.key);
}

bool makeInitialVoxelSpaceFromTexture(const Mesh& mesh, const std::shared_ptr<TextureDataProvider>& texture_data_provider, VoxelGrid& voxel_space)
{
    boost::concurrent_flat_set<uint8_t> found_extruders;

    cura::parallel_for(
        mesh.faces_,
        [&](const auto& iterator)
        {
            const MeshFace& face = *iterator;
            const std::optional<Point2F> uv0 = face.uv_coordinates_[0];
            const std::optional<Point2F> uv1 = face.uv_coordinates_[1];
            const std::optional<Point2F> uv2 = face.uv_coordinates_[2];

            if (! uv0.has_value() || ! uv1.has_value() || ! uv2.has_value())
            {
                return;
            }

            const Triangle2F face_uvs{ uv0.value(), uv1.value(), uv2.value() };

            constexpr double scale = 1.0;
            const Triangle3D triangle{ Point3D(mesh.vertices_[face.vertex_index_[0]].p_, scale),
                                       Point3D(mesh.vertices_[face.vertex_index_[1]].p_, scale),
                                       Point3D(mesh.vertices_[face.vertex_index_[2]].p_, scale) };

            for (const VoxelGrid::LocalCoordinates& traversed_voxel : voxel_space.getTraversedVoxels(triangle))
            {
                const Point3D global_position = voxel_space.toGlobalCoordinates(traversed_voxel);
                const std::optional<Point3D> barycentric_coordinates = getBarycentricCoordinates(global_position, triangle[0], triangle[1], triangle[2]);
                if (! barycentric_coordinates.has_value() || barycentric_coordinates.value().x_ < 0 || barycentric_coordinates.value().y_ < 0
                    || barycentric_coordinates.value().z_ < 0)
                {
                    // Triangle is invalid, or point is outside the triangle
                    continue;
                }

                const Point2F point_uv_coords = getUVCoordinates(barycentric_coordinates.value(), face_uvs);
                const std::pair<size_t, size_t> pixel = texture_data_provider->getTexture()->getPixelCoordinates(Point2F(point_uv_coords.x_, point_uv_coords.y_));
                const std::optional<uint32_t> extruder_nr = texture_data_provider->getValue(std::get<0>(pixel), std::get<1>(pixel), "extruder");
                if (extruder_nr.has_value())
                {
                    voxel_space.setOrUpdateOccupation(traversed_voxel, extruder_nr.value());
                    found_extruders.insert(extruder_nr.value());
                }
            }
        });

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
            const Point3D global_coordinates = voxel_grid.toGlobalCoordinates(voxel.first);
            mutex.lock();
            lookup_tree.pixels_cloud.push_back(Pixel3D{ global_coordinates, voxel.second });
            mutex.unlock();
        });

    lookup_tree.boost_tree = Boost_RTree(lookup_tree.pixels_cloud.begin(), lookup_tree.pixels_cloud.end());

    return lookup_tree;
}

union ContourKey
{
    uint32_t key;
    struct
    {
        uint16_t z;
        uint8_t extruder;
        uint8_t black_hole{ 0 }; // Don't place anything in there, or it would be lost forever (it exists only to properly set the 4th byte of the key)
    } definition;
};

std::size_t hash_value(ContourKey const& contour_key)
{
    return boost::hash<uint64_t>()(contour_key.key);
}

bool operator==(const ContourKey& key1, const ContourKey& key2)
{
    return key1.key == key2.key;
}

std::vector<Mesh> makeMeshesFromPointsClouds(const VoxelGrid& voxel_grid)
{
    spdlog::debug("Make meshes from points clouds");

    // Gather all positions that should be considered for a marching square, e.g. all that have a non-null extruder and around them
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> marching_squares;
    voxel_grid.visitOccupiedVoxels(
        [&marching_squares](const auto& occupied_voxel)
        {
            if (occupied_voxel.second > 0)
            {
                marching_squares.insert(occupied_voxel.first);
                if (occupied_voxel.first.position.x > 0)
                {
                    marching_squares.insert(VoxelGrid::LocalCoordinates(occupied_voxel.first.position.x - 1, occupied_voxel.first.position.y, occupied_voxel.first.position.z));
                }
                if (occupied_voxel.first.position.y > 0)
                {
                    marching_squares.insert(VoxelGrid::LocalCoordinates(occupied_voxel.first.position.x, occupied_voxel.first.position.y - 1, occupied_voxel.first.position.z));
                }
                if (occupied_voxel.first.position.x > 0 && occupied_voxel.first.position.y > 0)
                {
                    marching_squares.insert(VoxelGrid::LocalCoordinates(occupied_voxel.first.position.x - 1, occupied_voxel.first.position.y - 1, occupied_voxel.first.position.z));
                }
            }
        });

    // Build the cached list of segments to be added
    const double half_res_x = voxel_grid.getResolution().x_ / 2;
    const double half_res_y = voxel_grid.getResolution().y_ / 2;
    std::array<OpenLinesSet, 16> marching_segments;
    // marching_segments[0] = OpenLinesSet(); // This case is empty
    marching_segments[1] = OpenLinesSet{ OpenPolyline({ Point2LL(0, half_res_y), Point2LL(half_res_x, 0) }) };
    marching_segments[2] = OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(0, half_res_y) }) };
    marching_segments[3] = OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(half_res_x, 0) }) };
    marching_segments[4] = OpenLinesSet{ OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(0, -half_res_y) }) };
    marching_segments[5] = OpenLinesSet{ OpenPolyline({ Point2LL(0, half_res_y), Point2LL(0, -half_res_y) }) };
    marching_segments[6] = OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(0, half_res_y) }), OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(0, -half_res_y) }) };
    marching_segments[7] = OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(0, -half_res_y) }) };
    marching_segments[8] = OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(-half_res_x, 0) }) };
    marching_segments[9] = OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(-half_res_x, 0) }), OpenPolyline({ Point2LL(0, half_res_y), Point2LL(half_res_x, 0) }) };
    marching_segments[10] = OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(0, half_res_y) }) };
    marching_segments[11] = OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(half_res_x, 0) }) };
    marching_segments[12] = OpenLinesSet{ OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(-half_res_x, 0) }) };
    marching_segments[13] = OpenLinesSet{ OpenPolyline({ Point2LL(0, half_res_y), Point2LL(-half_res_x, 0) }) };
    marching_segments[14] = OpenLinesSet{ OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(0, half_res_y) }) };
    // marching_segments[15] = OpenLinesSet(); // This case is empty

    // Now visit all the squares and generate the appropriate outer segments
    struct Contour
    {
        OpenLinesSet segments; // Single segments before stitching
        Shape polygons; // Assembled polygons
    };

    boost::concurrent_flat_map<ContourKey, Contour> raw_contours;
    marching_squares.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&voxel_grid, &raw_contours, &half_res_x, &half_res_y, &marching_segments](const VoxelGrid::LocalCoordinates square_start)
        {
            const int32_t x_plus1 = static_cast<int32_t>(square_start.position.x) + 1;
            const bool x_plus1_valid = x_plus1 <= std::numeric_limits<uint16_t>::max();
            const int32_t y_plus1 = static_cast<int32_t>(square_start.position.y) + 1;
            const bool y_plus1_valid = y_plus1 <= std::numeric_limits<uint16_t>::max();

            const uint8_t occupation_bit0
                = x_plus1_valid && y_plus1_valid ? voxel_grid.getOccupation(VoxelGrid::LocalCoordinates(x_plus1, y_plus1, square_start.position.z)).value_or(0) : 0;
            const uint8_t occupation_bit1
                = y_plus1_valid ? voxel_grid.getOccupation(VoxelGrid::LocalCoordinates(square_start.position.x, y_plus1, square_start.position.z)).value_or(0) : 0;
            const uint8_t occupation_bit2
                = x_plus1_valid ? voxel_grid.getOccupation(VoxelGrid::LocalCoordinates(x_plus1, square_start.position.y, square_start.position.z)).value_or(0) : 0;
            const uint8_t occupation_bit3 = voxel_grid.getOccupation(square_start).value_or(0);

            std::unordered_set<uint8_t> extruders = { occupation_bit0, occupation_bit1, occupation_bit2, occupation_bit3 };
            for (const uint8_t extruder : extruders)
            {
                if (extruder == 0)
                {
                    continue;
                }

                size_t segments_index = (occupation_bit0 == extruder ? 1 : 0) + ((occupation_bit1 == extruder ? 1 : 0) << 1) + ((occupation_bit2 == extruder ? 1 : 0) << 2)
                                      + ((occupation_bit3 == extruder ? 1 : 0) << 3);
                OpenLinesSet translated_segments = marching_segments[segments_index];

                Point3D center_position = voxel_grid.toGlobalCoordinates(square_start);
                center_position += Point3D(half_res_x, half_res_y, 0);
                Point2LL center_position_ll(center_position.x_, center_position.y_);
                for (OpenPolyline& lines_set : translated_segments)
                {
                    for (Point2LL& point : lines_set)
                    {
                        point += center_position_ll;
                    }
                }

                raw_contours.emplace_or_visit(
                    std::make_pair(ContourKey{ .definition = { square_start.position.z, extruder } }, Contour{ .segments = translated_segments }),
                    [&translated_segments](auto& plane_contour)
                    {
                        plane_contour.second.segments.push_back(translated_segments);
                    });
            }
        });

    raw_contours.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [](auto& raw_contour)
        {
            OpenLinesSet result_lines;
            OpenPolylineStitcher::stitch(raw_contour.second.segments, result_lines, raw_contour.second.polygons);
        });

    const double min_distance = std::min({ voxel_grid.getResolution().x_, voxel_grid.getResolution().y_, voxel_grid.getResolution().z_ }) / 2.0;
    Simplify simplifier(min_distance, min_distance / 2, std::numeric_limits<coord_t>::max());

    std::map<uint8_t, Mesh> meshes;
    std::mutex mutex;
    raw_contours.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&simplifier, &voxel_grid, &meshes, &mutex](const auto& contour)
        {
            const uint16_t z = contour.first.definition.z;
            const uint8_t extruder = contour.first.definition.extruder;
            const coord_t z_low = voxel_grid.toGlobalZ(z, false);
            const coord_t z_high = voxel_grid.toGlobalZ(z + 1, false);

            for (const Polygon& polygon : contour.second.polygons)
            {
                // Do not export holes, only outer contours
                if (polygon.area() > 0)
                {
                    const Polygon simplified_polygon = simplifier.polygon(polygon);

                    mutex.lock();
                    const auto mesh_iterator = meshes.find(extruder);
                    if (mesh_iterator == meshes.end())
                    {
                        Mesh mesh(Application::getInstance().current_slice_->scene.extruders.at(extruder).settings_);
                        mesh.settings_.add("cutting_mesh", "true");
                        mesh.settings_.add("extruder_nr", std::to_string(extruder));
                        meshes.insert({ extruder, mesh });
                    }

                    Mesh& mesh = meshes[extruder];
                    for (auto iterator = simplified_polygon.beginSegments(); iterator != simplified_polygon.endSegments(); ++iterator)
                    {
                        const Point2LL& start = (*iterator).start;
                        const Point2LL& end = (*iterator).end;
                        mesh.addFace(Point3LL(start, z_low), Point3LL(end, z_low), Point3LL(end, z_high));
                        mesh.addFace(Point3LL(end, z_high), Point3LL(start, z_high), Point3LL(start, z_low));
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

std::vector<Shape> sliceMesh(const Mesh& base_mesh, const VoxelGrid& rasterized_mesh)
{
    const coord_t thickness = rasterized_mesh.getResolution().z_;
    const coord_t initial_layer_thickness = thickness;
    constexpr bool use_variable_layer_heights = false;
    constexpr std::vector<AdaptiveLayer>* adaptive_layers = nullptr;
    constexpr SlicingTolerance slicing_tolerance = SlicingTolerance::INCLUSIVE;

    // There is some margin in the voxel grid around the mesh, so get the actual mesh bounding box and see how many layers are actually covered
    AABB3D mesh_bounding_box = base_mesh.getAABB();
    const size_t margin_below_mesh = rasterized_mesh.toLocalZ(mesh_bounding_box.min_.z_);
    const size_t slice_layer_count = rasterized_mesh.toLocalZ(mesh_bounding_box.max_.z_) - margin_below_mesh + 1;

    const double xy_offset = INT2MM(std::min(rasterized_mesh.getResolution().x_, rasterized_mesh.getResolution().y_) * 2);
    Mesh sliced_mesh = base_mesh;
    sliced_mesh.settings_.add("xy_offset", std::to_string(xy_offset));
    sliced_mesh.settings_.add("xy_offset_layer_0", "0");
    sliced_mesh.settings_.add("hole_xy_offset", "0");
    sliced_mesh.settings_.add("hole_xy_offset_max_diameter", "0");

    Slicer slicer(&sliced_mesh, thickness, slice_layer_count, use_variable_layer_heights, adaptive_layers, slicing_tolerance, initial_layer_thickness);

    // In order to re-create an offset on the Z direction, union the sliced shapes over a few layers so that we get an approximate outer shell of it
    std::vector<Shape> slices;
    for (std::ptrdiff_t layer_index = 0; layer_index < rasterized_mesh.getMaxCoordinates(); ++layer_index)
    {
        Shape expanded_shape;
        for (std::ptrdiff_t delta = -2; delta <= 2; ++delta)
        {
            std::ptrdiff_t union_layer_index = layer_index + delta - margin_below_mesh;
            if (union_layer_index >= 0 && static_cast<size_t>(union_layer_index) < slicer.layers.size())
            {
                expanded_shape = expanded_shape.unionPolygons(slicer.layers[union_layer_index].polygons_);
            }
        }
        slices.push_back(expanded_shape);
    }
    return slices;
}

bool isInside(const VoxelGrid& voxel_grid, const VoxelGrid::LocalCoordinates& position, const std::vector<Shape>& sliced_mesh)
{
    const Point3D global_position = voxel_grid.toGlobalCoordinates(position);
    constexpr bool border_result = true;
    return sliced_mesh.at(position.position.z).inside(Point2LL(global_position.x_, global_position.y_), border_result);
}

std::vector<Mesh> makeModifierMeshes(const Mesh& mesh, const std::shared_ptr<TextureDataProvider>& texture_data_provider)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;

    // Fill a first voxel grid by rasterizing the triangles of the mesh in 3D, and assign the extruders according to the texture. This way we can later evaluate which extruder
    // to assign any point in 3D space just by finding the closest outside point and see what extruder it is assigned to.
    spdlog::debug("Fill original voxels based on texture data");
    auto resolution = settings.get<coord_t>("multi_material_paint_resolution");
    AABB3D bounding_box;
    for (const MeshVertex& vertex : mesh.vertices_)
    {
        bounding_box.include(vertex.p_);
    }
    bounding_box.expand(resolution * 4);

    VoxelGrid voxel_grid(bounding_box, resolution);
    if (! makeInitialVoxelSpaceFromTexture(mesh, texture_data_provider, voxel_grid))
    {
        // Texture is filled with 0s, don't bother doing anything
        return {};
    }

    spdlog::debug("Prepare AABB trees for fast look-up");
    LookupTree tree = makeLookupTreeFromVoxelGrid(voxel_grid);

    const auto deepness = settings.get<coord_t>("multi_material_paint_deepness");
    const coord_t deepness_squared = deepness * deepness;

    // Create a slice of the mesh so that we can quickly check for points insideness
    std::vector<Shape> sliced_mesh = sliceMesh(mesh, voxel_grid);
    bool check_inside = true;

    spdlog::debug("Get initially filled voxels");
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> previously_evaluated_voxels;
    voxel_grid.visitOccupiedVoxels(
        [&previously_evaluated_voxels](const auto& voxel)
        {
            if (voxel.second > 0)
            {
                previously_evaluated_voxels.insert(voxel.first);
            };
        });

    uint32_t iteration = 0;

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
                for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_grid.getVoxelsAround(previously_evaluated_voxel))
                {
                    if (voxels_to_evaluate.contains(voxel_around))
                    {
                        // This voxel has already been registered for evaluation
                        continue;
                    }

                    const std::optional<uint8_t> occupation = voxel_grid.getOccupation(voxel_around);
                    if (occupation.has_value())
                    {
                        // Voxel is already filled, don't evaluate it anyhow
                        continue;
                    }

                    bool evaluate_voxel;
                    if (check_inside)
                    {
                        evaluate_voxel = isInside(voxel_grid, voxel_around, sliced_mesh);
                        if (! evaluate_voxel)
                        {
                            voxel_grid.setOccupation(voxel_around, 0);

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
            spdlog::debug("Stop checking for voxels insideness at iteration {}", iteration);
            check_inside = false;
        }

        // Now actually evaluate the candidate voxels, i.e. find their closest outside point and set the according occupation
        spdlog::debug("Evaluating {} voxels", voxels_to_evaluate.size());
        voxels_to_evaluate.visit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            [&voxel_grid, &tree, &deepness_squared](const VoxelGrid::LocalCoordinates& voxel_to_evaluate)
            {
                const Point3D position = voxel_grid.toGlobalCoordinates(voxel_to_evaluate);

                // Define a query point
                const Boost_Point3D query_point = { position.x_, position.y_, position.z_ };

                // Find the nearest neighbor
                std::vector<Pixel3D> nearest_neighbors;
                tree.boost_tree.query(boost::geometry::index::nearest(query_point, 1), std::back_inserter(nearest_neighbors));

                if (! nearest_neighbors.empty())
                {
                    const Pixel3D& closest_neighbor = nearest_neighbors.front();
                    const Point3D diff = position - closest_neighbor.position;
                    const uint8_t new_occupation = diff.vSize2() <= deepness_squared ? closest_neighbor.occupation : 0;
                    voxel_grid.setOccupation(voxel_to_evaluate, new_occupation);
                }
                else
                {
                    voxel_grid.setOccupation(voxel_to_evaluate, 0);
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
            [&voxel_grid, &previously_evaluated_voxels](const VoxelGrid::LocalCoordinates& evaluated_voxel)
            {
                bool has_various_voxels_around = false;
                uint8_t actual_occupation = voxel_grid.getOccupation(evaluated_voxel).value();
                for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_grid.getVoxelsAround(evaluated_voxel))
                {
                    std::optional<uint8_t> around_occupation = voxel_grid.getOccupation(voxel_around);
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

        ++iteration;
    }

    return makeMeshesFromPointsClouds(voxel_grid);
}

void makeMaterialModifierMeshes(Mesh& mesh, MeshGroup* meshgroup)
{
    if (mesh.texture_ == nullptr || mesh.texture_data_mapping_ == nullptr || ! mesh.texture_data_mapping_->contains("extruder"))
    {
        return;
    }

    const spdlog::stopwatch timer;
    spdlog::info("Start multi-material mesh generation");

    const auto texture_data_provider = std::make_shared<TextureDataProvider>(nullptr, mesh.texture_, mesh.texture_data_mapping_);

    for (const Mesh& modifier_mesh : makeModifierMeshes(mesh, texture_data_provider))
    {
        meshgroup->meshes.push_back(modifier_mesh);
    }

    spdlog::info("Multi-material mesh generation took {} seconds", timer.elapsed().count());
}

} // namespace cura::MeshMaterialSplitter
