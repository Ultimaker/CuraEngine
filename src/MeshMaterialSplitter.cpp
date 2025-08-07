// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>

#include <boost/unordered/concurrent_flat_map.hpp>
#include <boost/unordered/concurrent_flat_set.hpp>
#include <boost/unordered/unordered_flat_set.hpp>
#include <range/v3/algorithm/max_element.hpp>
#include <spdlog/spdlog.h>

#include "MeshGroup.h"
#include "Slice.h"
#include "TextureDataProvider.h"
#include "geometry/Shape.h"
#include "mesh.h"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"

#define EXPORT_DEBUG_MESHES 0
#define EXPORT_ITERATION_MESHES 0


namespace cura::MeshMaterialSplitter
{
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Direction_3 = Kernel::Direction_3;
using Triangle_3 = Kernel::Triangle_3;

using Point_2 = Kernel::Point_2;
using Triangle_2 = Kernel::Triangle_2;
using Vector_2 = Kernel::Vector_2;

using PolygonMesh = CGAL::Surface_mesh<Kernel::Point_3>;

using Kernel_U32 = CGAL::Simple_cartesian<uint32_t>;
using Point_2U32 = Kernel_U32::Point_2;

struct PixelsCloudPrimitive;
using AABB_traits = CGAL::AABB_traits_3<Kernel, PixelsCloudPrimitive>;
using Tree = CGAL::AABB_tree<AABB_traits>;

void exportMesh(const PolygonMesh& mesh, const std::string& filename)
{
#if EXPORT_DEBUG_MESHES
    PolygonMesh exported_mesh = mesh;

    std::ofstream out(fmt::format("/home/erwan/test/CURA-12449_handling-painted-models/{}.obj", filename));
    for (CGAL::SM_Vertex_index vertex : vertices(exported_mesh))
    {
        Point_3& p = exported_mesh.point(vertex);
        p = Point_3(p.x() * 0.001, p.y() * 0.001, p.z() * 0.001);
    }
    CGAL::IO::write_OBJ(out, exported_mesh);
#endif
}

template<typename PointContainer>
void exportPointsCloud(const PointContainer& points_cloud, const std::string& filename)
{
#if EXPORT_DEBUG_MESHES
    PolygonMesh exported_mesh;
    for (const Point_3& point : points_cloud)
    {
        exported_mesh.add_vertex(point);
    }
    exportMesh(exported_mesh, filename);
#endif
}

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

Point_2 getUVCoordinates(const Point_3& barycentric_coordinates, const Triangle_2& face_uv_coordinates)
{
    return Point_2(
        (face_uv_coordinates[2].x() * barycentric_coordinates.x()) + (face_uv_coordinates[0].x() * barycentric_coordinates.y())
            + (face_uv_coordinates[1].x() * barycentric_coordinates.z()),
        (face_uv_coordinates[2].y() * barycentric_coordinates.x()) + (face_uv_coordinates[0].y() * barycentric_coordinates.y())
            + (face_uv_coordinates[1].y() * barycentric_coordinates.z()));
}

template<typename PointsContainer>
void makeMeshFromPointsCloud(const PointsContainer& points_cloud, PolygonMesh& output_mesh, const coord_t points_grid_resolution)
{
    if (points_cloud.empty())
    {
        return;
    }

    const double alpha = points_grid_resolution * 2.0;
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

struct Pixel3D
{
    Point_3 position;
    uint8_t occupation;
};

using PixelsCloud = std::vector<Pixel3D>;

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

    std::optional<ParameterizedSegment> intersectionWithXLayer(const double layer_start, const double layer_end) const
    {
        if (direction_.x() > 0)
        {
            if (start_.x() <= layer_end && end_.x() >= layer_start)
            {
                return ParameterizedSegment(start_.x() < layer_start ? pointAtX(layer_start) : start_, end_.x() > layer_end ? pointAtX(layer_end) : end_);
            }

            return std::nullopt;
        }
        else if (direction_.x() < 0)
        {
            if (end_.x() <= layer_end && start_.x() >= layer_start)
            {
                return ParameterizedSegment(end_.x() < layer_start ? pointAtX(layer_start) : end_, start_.x() > layer_end ? pointAtX(layer_end) : start_);
            }

            return std::nullopt;
        }
        else
        {
            if (start_.x() >= layer_start && start_.x() <= layer_end)
            {
                return *this;
            }

            return std::nullopt;
        }
    }

    std::optional<ParameterizedSegment> intersectionWithYLayer(const double layer_start, const double layer_end) const
    {
        if (direction_.y() > 0)
        {
            if (start_.y() <= layer_end && end_.y() >= layer_start)
            {
                return ParameterizedSegment(start_.y() < layer_start ? pointAtY(layer_start) : start_, end_.y() > layer_end ? pointAtY(layer_end) : end_);
            }

            return std::nullopt;
        }
        else if (direction_.y() < 0)
        {
            if (end_.y() <= layer_end && start_.y() >= layer_start)
            {
                return ParameterizedSegment(end_.y() < layer_start ? pointAtY(layer_start) : end_, start_.y() > layer_end ? pointAtY(layer_end) : start_);
            }

            return std::nullopt;
        }
        else
        {
            if (start_.y() >= layer_start && start_.y() <= layer_end)
            {
                return *this;
            }

            return std::nullopt;
        }
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

    union LocalCoordinates
    {
        uint64_t key{ 0 };
        SimplePoint_3U16 position;

        LocalCoordinates(uint16_t x, uint16_t y, uint16_t z)
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
    explicit VoxelGrid(const CGAL::Bbox_3& bounding_box, const double max_resolution)
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

    Point_3 toGlobalCoordinates(const LocalCoordinates& position) const
    {
        return Point_3(toGlobalX(position.position.x), toGlobalY(position.position.y), toGlobalZ(position.position.z));
    }

    void setOccupation(const LocalCoordinates& position, const uint8_t extruder_nr)
    {
        occupied_voxels_.insert_or_assign(position, extruder_nr);
    }

    void setOrUpdateOccupation(const LocalCoordinates& position, const uint8_t extruder_nr)
    {
        occupied_voxels_.insert_or_visit(
            std::make_pair(position, extruder_nr),
            [extruder_nr](auto& voxel)
            {
                voxel.second = std::min(voxel.second, extruder_nr);
            });
    }

    std::optional<uint8_t> getOccupation(const LocalCoordinates& local_position) const
    {
        std::optional<uint8_t> result = std::nullopt;
        occupied_voxels_.visit(
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
        occupied_voxels_.visit_all(std::execution::par, args...);
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
                        voxels_around.emplace_back(pos_x, pos_y, pos_z);
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

    double toGlobalX(const uint16_t x) const
    {
        return (x * resolution_.x()) + origin_.x();
    }

    uint16_t toLocalY(const double y) const
    {
        return (y - origin_.y()) / resolution_.y();
    }

    double toGlobalY(const uint16_t y) const
    {
        return (y * resolution_.y()) + origin_.y();
    }

    uint16_t toLocalZ(const double z) const
    {
        return (z - origin_.z()) / resolution_.z();
    }

    double toGlobalZ(const uint16_t z) const
    {
        return (z * resolution_.z()) + origin_.z();
    }

    void exportToFile(const std::string& basename) const
    {
#if EXPORT_DEBUG_MESHES
        std::map<uint8_t, PolygonMesh> meshes;

        occupied_voxels_.visit_all(
            [&meshes, this](const auto& voxel)
            {
                meshes[voxel.second].add_vertex(toGlobalCoordinates(voxel.first));
            });

        for (auto iterator = meshes.begin(); iterator != meshes.end(); ++iterator)
        {
            exportMesh(iterator->second, fmt::format("{}_{}", basename, iterator->first));
        }
#endif
    }

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

            const uint16_t ymin = toLocalY(std::ranges::min(y_values));
            const uint16_t ymax = toLocalY(std::ranges::max(y_values));

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

                const uint16_t zmin = toLocalZ(std::ranges::min(z_values));
                const uint16_t zmax = toLocalZ(std::ranges::max(z_values));

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
    const auto& faces = mesh.faces();
    const auto& uv_coords = mesh.property_map<CGAL::SM_Face_index, std::array<Point_2, 3>>("f:uv_coords").value();
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

void registerModifiedMesh(MeshGroup* meshgroup, const PolygonMesh& output_mesh, const uint8_t extruder_nr)
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
    modifier_mesh.settings_.add("extruder_nr", std::to_string(extruder_nr));

    meshgroup->meshes.push_back(modifier_mesh);
}


struct PixelsCloudPrimitive
{
public:
    typedef const Pixel3D* Id;
    typedef Point_3 Point;
    typedef Point_3 Datum;

private:
    Id m_pt;

public:
    explicit PixelsCloudPrimitive(PixelsCloud::const_iterator it)
        : m_pt(&(*it))
    {
    }

    const Id& id() const
    {
        return m_pt;
    }

    Point convert(const Pixel3D* p) const
    {
        return p->position;
    }

    Datum datum() const
    {
        return m_pt->position;
    }

    Point reference_point() const
    {
        return m_pt->position;
    }
};

struct LookupTree
{
    PixelsCloud pixels_cloud;
    Tree tree;
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
            lookup_tree.pixels_cloud.emplace_back(global_coordinates, voxel.second);
            mutex.unlock();
        });

    lookup_tree.tree = Tree(lookup_tree.pixels_cloud.begin(), lookup_tree.pixels_cloud.end());
    lookup_tree.tree.accelerate_distance_queries();
    return lookup_tree;
}

void makeModifierMeshVoxelSpace(const PolygonMesh& mesh, const std::shared_ptr<TextureDataProvider>& texture_data_provider, std::map<uint8_t, PolygonMesh>& output_meshes)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;

    // Fill a first voxel grid by rasterizing the triangles of the mesh in 3D, and assign the extruders according to the texture. This way we can later evaluate which extruder to
    // assign any point in 3D space just by finding the closest outside point and see what extruder it is assigned to.
    spdlog::info("Fill original voxels based on texture data");
    CGAL::Bbox_3 bounding_box = CGAL::Polygon_mesh_processing::bbox(mesh);
    double resolution = settings.get<double>("multi_material_paint_resolution") * 1000.0;
    VoxelGrid voxel_space(bounding_box, resolution);
    if (! makeInitialVoxelSpaceFromTexture(mesh, texture_data_provider, voxel_space))
    {
        // Texture is filled with 0s, don't bother doing anything
        return;
    }
    // Create 2 AABB trees for efficient spatial queries. Points lookup is very fast as long as there is a close point, so the deeper we go inside the mesh, the longer the lookup
    // time will be. However, we don't require a high precision inside the mesh because it won't be visible, so use 2 lookup trees, one with high resolution for the outside, and
    // a second one with very low resolution for when we are far enough from the outside.
    VoxelGrid low_res_texture_data(bounding_box, std::max(1000.0, resolution));
    makeInitialVoxelSpaceFromTexture(mesh, texture_data_provider, low_res_texture_data);

    spdlog::info("Prepare AABB trees for fast look-up");
    LookupTree tree = makeLookupTreeFromVoxelGrid(voxel_space);
    LookupTree tree_lowres = makeLookupTreeFromVoxelGrid(low_res_texture_data);

    spdlog::info("Export initial points clouds");
    voxel_space.exportToFile("initial_points_cloud");

    const double deepness = settings.get<double>("multi_material_paint_deepness") * 1000.0;
    const Point_3& spatial_resolution = voxel_space.getResolution();
    const float deepness_squared = deepness * deepness;

    // Generate a clean and approximate version of the mesh by alpha-wrapping it, so that we can do proper and fast inside-mesh checking
    spdlog::info("prepare alpha mesh");
    PolygonMesh alpha_mesh;
    constexpr double alpha = 2000.0;
    constexpr double offset = 10.0;
    CGAL::alpha_wrap_3(mesh, alpha, offset, alpha_mesh);
    exportMesh(alpha_mesh, "alpha_mesh");
    CGAL::Side_of_triangle_mesh<PolygonMesh, Kernel> inside_mesh(alpha_mesh);
    bool check_inside = true;

    spdlog::info("Get initially filled voxels");
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> previously_evaluated_voxels;
    voxel_space.visitOccupiedVoxels(
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

        spdlog::info("Finding voxels around {} voxels", previously_evaluated_voxels.size());

        // For each already-filled voxel, gather the voxels around it and evaluate them
        previously_evaluated_voxels.visit_all(
            std::execution::par,
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

        // As soon as we are deep enough in the mesh, use the low-resolution lookup tree which makes finding the closest outside point much faster
        const Tree& actual_lookup_tree = iteration < 12 ? tree.tree : tree_lowres.tree;

        if (check_inside && ! keep_checking_inside.load())
        {
            spdlog::info("Stop checking for voxels insideness");
            check_inside = false;
        }

        // Now actually evaluate the candidate voxels, i.e. find their closest outside point and set the according occupation
        spdlog::info("Evaluating {} voxels", voxels_to_evaluate.size());
        voxels_to_evaluate.visit_all(
            std::execution::par,
            [&voxel_space, &actual_lookup_tree, &deepness_squared](const VoxelGrid::LocalCoordinates& voxel_to_evaluate)
            {
                const Point_3 position = voxel_space.toGlobalCoordinates(voxel_to_evaluate);
                const std::pair<Point_3, PixelsCloudPrimitive::Id> closest_point_and_primitive = actual_lookup_tree.closest_point_and_primitive(position);
                const Vector_3 diff = position - closest_point_and_primitive.first;
                const uint8_t new_occupation = diff.squared_length() <= deepness_squared ? closest_point_and_primitive.second->occupation : 0;
                voxel_space.setOccupation(voxel_to_evaluate, new_occupation);
            });

        // Now we have evaluated the candidates, check which of them are to be processed next. We skip all the voxels that have only voxels with similar occupations around
        // them, because they are obviously not part of the boundaries we are looking for. This avoids filling the inside of the points cloud and speeds up calculation a lot.
        spdlog::info("Find boundary voxels for next round");
        previously_evaluated_voxels.clear();
        voxels_to_evaluate.visit_all(
            std::execution::par,
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

        iteration++;
#if EXPORT_ITERATION_MESHES
        voxel_space.exportToFile(fmt::format("points_cloud_iteration_{}", iteration++));
#endif
    }

    spdlog::info("Make final points clouds");
    voxel_space.exportToFile("final_grid");
    boost::concurrent_flat_map<uint8_t, boost::concurrent_flat_set<Point_3>> points_clouds;
    voxel_space.visitOccupiedVoxels(
        [&points_clouds, &voxel_space](const auto& filled_voxel)
        {
            if (filled_voxel.second > 0)
            {
                Point_3 position = voxel_space.toGlobalCoordinates(filled_voxel.first);
                points_clouds.insert_or_visit(
                    std::make_pair(filled_voxel.second, boost::concurrent_flat_set({ position })),
                    [&position](auto& points_cloud)
                    {
                        points_cloud.second.insert(position);
                    });
            }
        });

    spdlog::info("Make mesh from points cloud");
    std::mutex mutex;
    points_clouds.visit_all(
        [&spatial_resolution, &mutex, &output_meshes](auto& points_cloud_it)
        {
            boost::concurrent_flat_set<Point_3>& points_cloud = points_cloud_it.second;
            boost::unordered_flat_set<Point_3> points_cloud_non_concurrent = std::move(points_cloud);

            PolygonMesh output_mesh;
            makeMeshFromPointsCloud(points_cloud_non_concurrent, output_mesh, std::max({ spatial_resolution.x(), spatial_resolution.y(), spatial_resolution.z() }));

            mutex.lock();
            output_meshes[points_cloud_it.first] = std::move(output_mesh);
            mutex.unlock();
        });
}

void splitMesh(Mesh& mesh, MeshGroup* meshgroup)
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

    exportMesh(converted_mesh, "converted_mesh");

    if (! has_uvs)
    {
        return;
    }

    const auto texture_data_provider = std::make_shared<TextureDataProvider>(nullptr, mesh.texture_, mesh.texture_data_mapping_);

    std::map<uint8_t, PolygonMesh> output_meshes;
    makeModifierMeshVoxelSpace(converted_mesh, texture_data_provider, output_meshes);
    for (auto& [extruder_nr, output_mesh] : output_meshes)
    {
        if (output_mesh.faces().empty())
        {
            continue;
        }

        registerModifiedMesh(meshgroup, output_mesh, extruder_nr);
        exportMesh(output_mesh, fmt::format("output_mesh_{}", extruder_nr));
    }

    spdlog::info("Multi-material mesh splitting took {} seconds", timer.elapsed().count());
}

} // namespace cura::MeshMaterialSplitter
