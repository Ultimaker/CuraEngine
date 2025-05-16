// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

// #include <CGAL/AABB_face_graph_triangle_primitive.h>
// #include <CGAL/AABB_traits_3.h>
// #include <CGAL/Boolean_set_operations_2.h>
// #include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/number_type_basic.h>

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered_map.hpp>
#include <range/v3/algorithm/all_of.hpp>
#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/algorithm/remove.hpp>
#include <range/v3/algorithm/remove_if.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/map.hpp>
#include <spdlog/spdlog.h>

#include "MeshGroup.h"
#include "Slice.h"
#include "geometry/Shape.h"
#include "mesh.h"
#include "png++/image.hpp"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"

namespace CGAL
{
// Declare missing traits for supporting uint16 with CGAL
template<>
struct Real_embeddable_traits<uint16_t> : public INTERN_RET::Real_embeddable_traits_base<uint16_t, std::integral_constant<bool, true>>
{
    struct Abs
    {
        typedef uint16_t result_type;
        uint16_t operator()(const uint16_t& x) const
        {
            return x;
        }
    };

    struct Sgn
    {
        typedef ::CGAL::Sign result_type;
        ::CGAL::Sign operator()(const uint16_t& x) const
        {
            return (x == 0) ? CGAL::ZERO : CGAL::POSITIVE;
        }
    };

    struct Compare
    {
        typedef ::CGAL::Comparison_result result_type;
        ::CGAL::Comparison_result operator()(const uint16_t& a, const uint16_t& b) const
        {
            if (a < b)
                return CGAL::SMALLER;
            if (a > b)
                return CGAL::LARGER;
            return CGAL::EQUAL;
        }
    };

    struct To_double
    {
        typedef double result_type;
        double operator()(const uint16_t& x) const
        {
            return static_cast<double>(x);
        }
    };

    struct To_interval
    {
        typedef std::pair<double, double> result_type;
        std::pair<double, double> operator()(const uint16_t& x) const
        {
            double d = static_cast<double>(x);
            return std::make_pair(d, d);
        }
    };
};
} // namespace CGAL

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

using Kernel_S64 = CGAL::Simple_cartesian<int64_t>;
using Point_3S64 = Kernel_S64::Point_3;

using Kernel_U32 = CGAL::Simple_cartesian<uint32_t>;
using Point_3U32 = Kernel_U32::Point_3;
using Point_2U32 = Kernel_U32::Point_2;

using Kernel_U16 = CGAL::Simple_cartesian<uint16_t>;
using Point_3U16 = Kernel_U16::Point_3;
using Point_2U16 = Kernel_U16::Point_2;

using Kernel_S8 = CGAL::Simple_cartesian<int8_t>;
using Point_3S8 = Kernel_S8::Point_3;
using Vector_3S8 = Kernel_S8::Vector_3;

using Kernel_U8 = CGAL::Simple_cartesian<uint8_t>;
using Point_3U8 = Kernel_U8::Point_3;


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

Point_2 getPixelCoordinates(const Point_2& uv_coordinates, const png::image<png::rgb_pixel>& image)
{
    const uint32_t width = image.get_width();
    const uint32_t height = image.get_height();
    return Point_2(
        std::clamp(static_cast<uint32_t>(uv_coordinates.x() * width), static_cast<uint32_t>(0), width - 1),
        std::clamp(static_cast<uint32_t>(height - uv_coordinates.y() * height), static_cast<uint32_t>(0), height - 1));
}

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

template<typename PointsContainer>
void makeMeshFromPointsCloud(const PointsContainer& points_cloud, PolygonMesh& output_mesh, const coord_t points_grid_resolution)
{
    const double alpha = points_grid_resolution * 5.0;
    const double offset = alpha / 50.0;

    CGAL::alpha_wrap_3(points_cloud, alpha, offset, output_mesh);
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

    explicit OctreeNode(OctreeNode* parent = nullptr, const ExtruderOccupation& occupation = ExtruderOccupation::Unknown)
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

class VoxelOctree
{
public:
    using LocalCoordinates = Point_3U16;
    using KeyType = uint64_t;
    using DepthType = uint8_t;

public:
    explicit VoxelOctree(const PolygonMesh& mesh)
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

        nodes_[toKey(LocalCoordinates(0, 0, 0), 0)] = OctreeNode::ExtruderOccupation::Unknown;
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
        auto [depth, iterator] = getNode(position);
        if (iterator->second != occupation)
        {
            if (depth == max_depth_)
            {
                // This is already the leaf node, just change its value
                iterator->second = occupation;
            }
            else
            {
                // We don't have a leaf node for this voxel yet, create it
                nodes_[toKey(position)] = occupation;
            }

            // if (! children_split)
            // {
            //     // We have just changed the occupation value of an existing node, so try and check whether the tree can be compressed now
            //     OctreeNode* node_ptr = node.get();
            //     do
            //     {
            //         node_ptr = node_ptr->getParent();
            //     } while (node_ptr && node_ptr->compressIfPossible());
            // }

            return true;
        }

        return false;
    }

    uint8_t getExtruderNr(const LocalCoordinates& local_position) const
    {
        return static_cast<uint8_t>(getOccupation(local_position)) - static_cast<uint8_t>(OctreeNode::ExtruderOccupation::Occupied);
    }

    OctreeNode::ExtruderOccupation getOccupation(const LocalCoordinates& local_position) const
    {
        return std::get<1>(getNode(local_position))->second;
    }

    OctreeNode::ExtruderOccupation getOccupation(const Point_3& position) const
    {
        return getOccupation(toLocalCoordinates(position));
    }

    std::vector<LocalCoordinates> getFilledVoxels() const
    {
        std::vector<LocalCoordinates> filled_voxels;

        for (auto iterator = nodes_.begin(); iterator != nodes_.end(); ++iterator)
        {
            const auto [depth, coordinates] = toCoordinates(iterator->first);
            if (depth == max_depth_)
            {
                filled_voxels.push_back(coordinates);
            }
        }

#if 0
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
#endif

        return filled_voxels;
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
    KeyType toKey(const LocalCoordinates& coordinate, const uint8_t depth) const
    {
        const uint8_t depth_delta = max_depth_ - depth;
        return (static_cast<KeyType>(coordinate.x()) >> depth_delta) | ((static_cast<KeyType>(coordinate.y()) >> depth_delta) << 16)
             | ((static_cast<KeyType>(coordinate.z()) >> depth_delta) << 32) | (static_cast<KeyType>(depth) << 48);
    }

    KeyType toKey(const LocalCoordinates& coordinate) const
    {
        return static_cast<KeyType>(coordinate.x()) | (static_cast<KeyType>(coordinate.y()) << 16) | (static_cast<KeyType>(coordinate.z()) << 32)
             | (static_cast<KeyType>(max_depth_) << 48);
    }

    std::tuple<DepthType, LocalCoordinates> toCoordinates(const uint64_t key) const
    {
        const uint8_t depth = key >> 48;
        const uint8_t depth_delta = max_depth_ - depth;
        return std::make_tuple(depth, LocalCoordinates(key << depth_delta, (key >> 16) << depth_delta, (key >> 32) << depth_delta));
    }

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

    std::tuple<DepthType, boost::unordered_flat_map<KeyType, OctreeNode::ExtruderOccupation>::iterator> getNode(const LocalCoordinates& local_position)
    {
        boost::unordered_flat_map<KeyType, OctreeNode::ExtruderOccupation>::iterator iterator;
        DepthType actual_depth = max_depth_ + 1;
        do
        {
            actual_depth--;
            iterator = nodes_.find(toKey(local_position, actual_depth));
        } while (iterator == nodes_.end() && actual_depth > 0);

        return std::make_tuple(actual_depth, iterator);
    }

    std::tuple<DepthType, boost::unordered_flat_map<KeyType, OctreeNode::ExtruderOccupation>::const_iterator> getNode(const LocalCoordinates& local_position) const
    {
        // Reuse the non-const version and cast the iterator to const_iterator
        auto [depth, it] = const_cast<VoxelOctree*>(this)->getNode(local_position);
        return std::make_tuple(depth, static_cast<boost::unordered_flat_map<KeyType, OctreeNode::ExtruderOccupation>::const_iterator>(it));
    }

private:
    DepthType max_depth_{ 0 };
    Point_3 definition_;
    Vector_3 origin_;
    uint32_t max_coordinate_;
    boost::unordered_flat_map<KeyType, OctreeNode::ExtruderOccupation> nodes_;
};

void makeInitialVoxelSpaceFromTexture(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image, VoxelOctree& voxel_space)
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

void registerModifiedMesh(MeshGroup* meshgroup, const PolygonMesh& output_mesh)
{
    ExtruderTrain& extruder = Application::getInstance().current_slice_->scene.extruders.at(1);
    Mesh modifier_mesh(extruder.settings_);
    for (const CGAL::SM_Face_index face : output_mesh.faces())
    {
        const Triangle_3 triangle = getFaceTriangle(output_mesh, face);
        modifier_mesh.addFace(
            Point3LL(triangle[0].x(), triangle[0].y(), triangle[0].z()),
            Point3LL(triangle[1].x(), triangle[1].y(), triangle[1].z()),
            Point3LL(triangle[2].x(), triangle[2].y(), triangle[2].z()));
    }

    modifier_mesh.settings_.add("cutting_mesh", "true");
    modifier_mesh.settings_.add("extruder_nr", "1");

    meshgroup->meshes.push_back(modifier_mesh);
}

void makeModifierMeshVoxelSpace(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image, PolygonMesh& output_mesh)
{
    spdlog::info("Fill original voxels based on texture data");
    VoxelOctree voxel_space(mesh);
    makeInitialVoxelSpaceFromTexture(mesh, image, voxel_space);

    spdlog::info("Get initially filled voxels");
    std::vector<VoxelOctree::LocalCoordinates> filled_voxels = voxel_space.getFilledVoxels();

    spdlog::info("Export initial meshes");
    std::map<size_t, PolygonMesh> initial_meshes;
    for (const VoxelOctree::LocalCoordinates& local_coord : filled_voxels)
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
        std::map<VoxelOctree::LocalCoordinates, std::map<OctreeNode::ExtruderOccupation, uint8_t>> voxels_votes;

        cura::parallel_for(
            filled_voxels,
            [&](auto iterator)
            {
                const VoxelOctree::LocalCoordinates& filled_voxel = *iterator;
                mutex.lock();
                const OctreeNode::ExtruderOccupation actual_filled_occupation = voxel_space.getOccupation(filled_voxel);
                mutex.unlock();
                std::map<VoxelOctree::LocalCoordinates, std::map<uint8_t, uint8_t>> voxels_votes_local;

                for (const VoxelOctree::LocalCoordinates& voxel_around : voxel_space.getVoxelsAround(filled_voxel))
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

        spdlog::info("Apply growing with {} voted voxels", voxels_votes.size());

        std::vector<VoxelOctree::LocalCoordinates> new_filled_voxels;
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
    for (const VoxelOctree::LocalCoordinates& local_coord : voxel_space.getFilledVoxels())
    {
        if (voxel_space.getExtruderNr(local_coord) == 1)
        {
            points_cloud.push_back(voxel_space.toGlobalCoordinates(local_coord));
        }
    }
    exportPointsCloud(points_cloud, "final_contour");

    const Point_3& definition = voxel_space.getDefinition();
    spdlog::info("Making mesh from points cloud");
    makeMeshFromPointsCloud(points_cloud, output_mesh, std::max({ definition.x(), definition.y(), definition.z() }));
}

void splitMesh(Mesh& mesh, MeshGroup* meshgroup)
{
    png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/texture.png");
    // png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/texture-high.png");
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
    makeModifierMeshVoxelSpace(converted_mesh, image, output_mesh);
    registerModifiedMesh(meshgroup, output_mesh);

    exportMesh(converted_mesh, "converted_mesh");
    exportMesh(output_mesh, "output_mesh");
}

} // namespace cura::MeshMaterialSplitter
