// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>

#include <boost/unordered/concurrent_flat_map.hpp>
#include <boost/unordered/concurrent_flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>
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
#include "TextureDataProvider.h"
#include "geometry/Shape.h"
#include "mesh.h"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"

namespace cura::MeshMaterialSplitter
{
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
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

Point_2 getPixelCoordinates(const Point_2& uv_coordinates, const std::shared_ptr<Image>& image)
{
    std::pair<size_t, size_t> pixel_coordinates = image->getPixelCoordinates(Point2F(uv_coordinates.x(), uv_coordinates.y()));
    return Point_2(pixel_coordinates.first, pixel_coordinates.second);
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

template<typename PointType, typename VectorType>
std::optional<Point_3> getBarycentricCoordinates(const PointType& point, const PointType& p0, const PointType& p1, const PointType& p2)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const VectorType v0(p1 - p0);
    const VectorType v1(p2 - p0);
    const VectorType v2(point - p0);

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

struct Pixel3D
{
    Point_3 position;
    uint8_t occupation;
};

using PixelsCloud = std::vector<Pixel3D>;

class VoxelGrid
{
public:
    struct SimplePoint_3U16
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
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
    using DepthType = uint8_t;
    static constexpr uint8_t nb_voxels_around = 3 * 3 * 3 - 1;

public:
    explicit VoxelGrid(const PolygonMesh& mesh, const double max_resolution)
    {
        const CGAL::Bbox_3 expanded_bounding_box = expand(CGAL::Polygon_mesh_processing::bbox(mesh), max_resolution);
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
        return Point_3(position.position.x * resolution_.x(), position.position.y * resolution_.y(), position.position.z * resolution_.z()) + origin_;
    }

    void setOccupation(const Point_3& position, const uint8_t extruder_nr)
    {
        setOccupation(toLocalCoordinates(position), extruder_nr);
    }

    void setOccupation(const LocalCoordinates& position, const uint8_t extruder_nr)
    {
        nodes_.insert_or_assign(position, extruder_nr);
    }

    bool isFilled(const LocalCoordinates& local_position) const
    {
        return nodes_.contains(local_position);
    }

    std::optional<uint8_t> getOccupation(const LocalCoordinates& local_position) const
    {
        std::optional<uint8_t> result = std::nullopt;
        nodes_.visit(
            local_position,
            [&result](const auto& occupation)
            {
                result = occupation.second;
            });
        return result;
    }

    template<class... Args>
    void setOccupationIfUnknown(const LocalCoordinates& position, Args&&... args)
    {
        nodes_.try_emplace(position, std::forward<Args>(args)...);
    }

    template<class... Args>
    void visitFilledVoxels(Args&&... args)
    {
        nodes_.visit_all(std::execution::par, args...);
    }

    std::vector<LocalCoordinates> getFilledVoxels(const std::function<bool(const std::pair<LocalCoordinates, uint8_t>&)>& condition = nullptr) const
    {
        std::vector<LocalCoordinates> filled_voxels;

        nodes_.visit_all(
            [&filled_voxels, &condition](const auto& voxel)
            {
                if (! condition || condition(voxel))
                {
                    filled_voxels.push_back(voxel.first);
                }
            });

        return filled_voxels;
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
        const Point_3 position_in_space = position - origin_;
        return LocalCoordinates(position_in_space.x() / resolution_.x(), position_in_space.y() / resolution_.y(), position_in_space.z() / resolution_.z());
    }

    void exportToFile(const std::string& basename) const
    {
        std::map<uint8_t, PolygonMesh> meshes;

        nodes_.visit_all(
            [&meshes, this](const auto& voxel)
            {
                meshes[voxel.second].add_vertex(toGlobalCoordinates(voxel.first));
            });

        for (auto iterator = meshes.begin(); iterator != meshes.end(); ++iterator)
        {
            exportMesh(iterator->second, fmt::format("{}_{}", basename, iterator->first));
        }
    }


private:
    Point_3 resolution_;
    Vector_3 origin_;
    uint32_t max_coordinate_;
    boost::concurrent_flat_map<LocalCoordinates, uint8_t> nodes_;
};

std::size_t hash_value(VoxelGrid::LocalCoordinates const& position)
{
    return boost::hash<uint64_t>()(position.key);
}

void makeInitialVoxelSpaceFromTexture(const PolygonMesh& mesh, const std::shared_ptr<TextureDataProvider>& texture_data_provider, VoxelGrid& voxel_space, PixelsCloud& pixels_cloud)
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
            const Triangle_2 face_pixel_coordinates(
                getPixelCoordinates(face_uvs[2], texture_data_provider->getTexture()),
                getPixelCoordinates(face_uvs[0], texture_data_provider->getTexture()),
                getPixelCoordinates(face_uvs[1], texture_data_provider->getTexture()));
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
                        = getBarycentricCoordinates<Point_2, Vector_2>(pixel_center, face_pixel_coordinates[0], face_pixel_coordinates[1], face_pixel_coordinates[2]);

                    if (! barycentric_coordinates.has_value() || barycentric_coordinates.value().x() < 0 || barycentric_coordinates.value().y() < 0
                        || barycentric_coordinates.value().z() < 0)
                    {
                        // Triangle is invalid, or point is outside the triangle
                        continue;
                    }

                    const Point_3 pixel_3d = getSpaceCoordinates(pixel_center, face_pixel_coordinates, triangle);
                    const std::optional<uint32_t> extruder_nr = texture_data_provider->getValue(x, y, "extruder");

                    if (extruder_nr.has_value())
                    {
                        mutex.lock();
                        voxel_space.setOccupation(pixel_3d, extruder_nr.value());
                        pixels_cloud.emplace_back(pixel_3d, extruder_nr.value());
                        mutex.unlock();
                    }
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


struct PixelsCloudPrimitive
{
public:
    typedef const Pixel3D* Id;
    typedef Point_3 Point;
    typedef Point_3 Datum;

private:
    Id m_pt;

public:
    PixelsCloudPrimitive()
    {
    } // default constructor needed
    // the following constructor is the one that receives the iterators from the
    // iterator range given as input to the AABB_tree
    PixelsCloudPrimitive(PixelsCloud::const_iterator it)
        : m_pt(&(*it))
    {
    }
    const Id& id() const
    {
        return m_pt;
    }

    // utility function to convert a custom
    // point type to CGAL point type.
    Point convert(const Pixel3D* p) const
    {
        return p->position;
    }

    // on the fly conversion from the internal data to the CGAL types
    Datum datum() const
    {
        return m_pt->position;
    }

    // returns a reference point which must be on the primitive
    Point reference_point() const
    {
        return m_pt->position;
    }
};

void makeModifierMeshVoxelSpace(const PolygonMesh& mesh, const std::shared_ptr<TextureDataProvider>& texture_data_provider, PolygonMesh& output_mesh)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;

    spdlog::info("Fill original voxels based on texture data");
    VoxelGrid voxel_space(mesh, settings.get<double>("multi_material_paint_resolution") * 1000.0);
    PixelsCloud pixels_cloud;
    makeInitialVoxelSpaceFromTexture(mesh, texture_data_provider, voxel_space, pixels_cloud);

    const double deepness = settings.get<double>("multi_material_paint_deepness") * 1000.0;
    const Point_3& resolution = voxel_space.getResolution();
    const float deepness_squared = deepness * deepness;

    spdlog::info("Get initially filled voxels");
    const std::vector<VoxelGrid::LocalCoordinates> filled_voxels = voxel_space.getFilledVoxels(
        [](const std::pair<VoxelGrid::LocalCoordinates, uint8_t>& voxel)
        {
            return voxel.second > 0;
        });

    spdlog::info("Export initial meshes");
    voxel_space.exportToFile("initial_points_cloud");

    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> previously_evaluated_voxels;
    previously_evaluated_voxels.insert(filled_voxels.begin(), filled_voxels.end());

    // Create an AABB tree for efficient spatial queries
    spdlog::info("Prepare AABB tree for fast look-up");
    Tree tree(pixels_cloud.begin(), pixels_cloud.end());
    tree.accelerate_distance_queries();

    while (! previously_evaluated_voxels.empty())
    {
        boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> voxels_to_evaluate;
        boost::concurrent_flat_map<VoxelGrid::LocalCoordinates, std::vector<VoxelGrid::LocalCoordinates>> origin_voxels; // target_voxel: [origin_voxels]
        boost::concurrent_flat_map<VoxelGrid::LocalCoordinates, std::vector<VoxelGrid::LocalCoordinates>> target_voxels; // origin_voxel: [target_voxels]

        const auto register_voxel_relations = [&origin_voxels, &target_voxels](const VoxelGrid::LocalCoordinates& origin_voxel, const VoxelGrid::LocalCoordinates& target_voxel)
        {
            origin_voxels.insert_or_visit(
                std::make_pair(target_voxel, std::vector{ origin_voxel }),
                [&origin_voxel](auto& voxel)
                {
                    voxel.second.push_back(origin_voxel);
                });
            target_voxels.insert_or_visit(
                std::make_pair(origin_voxel, std::vector{ target_voxel }),
                [&target_voxel](auto& voxel)
                {
                    voxel.second.push_back(target_voxel);
                });
        };

        spdlog::info("Finding voxels around {} voxels", previously_evaluated_voxels.size());

        previously_evaluated_voxels.visit_all(
            std::execution::par,
            [&](const VoxelGrid::LocalCoordinates& previously_evaluated_voxel)
            {
                for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_space.getVoxelsAround(previously_evaluated_voxel))
                {
                    if (voxels_to_evaluate.contains(voxel_around))
                    {
                        register_voxel_relations(previously_evaluated_voxel, voxel_around);
                        continue;
                    }

                    if (! voxel_space.isFilled(voxel_around))
                    {
                        // Voxel is not occupied yet, so evaluate it
                        voxels_to_evaluate.emplace(voxel_around);
                        register_voxel_relations(previously_evaluated_voxel, voxel_around);
                    }
                }
            });

        spdlog::info("Evaluating {} voxels", voxels_to_evaluate.size());

        voxels_to_evaluate.visit_all(
            std::execution::par,
            [&voxel_space, &tree, &deepness_squared](const VoxelGrid::LocalCoordinates& voxel_to_evaluate)
            {
                // Find closest outside point and its occupation
                const Point_3 position = voxel_space.toGlobalCoordinates(voxel_to_evaluate);
                const std::pair<Point_3, PixelsCloudPrimitive::Id> closest_point_and_primitive = tree.closest_point_and_primitive(position);
                const Vector_3 diff = position - closest_point_and_primitive.first;
                if (diff.squared_length() <= deepness_squared)
                {
                    const uint8_t occupation = closest_point_and_primitive.second->occupation;
                    voxel_space.setOccupation(voxel_to_evaluate, occupation);
                }
                else
                {
                    voxel_space.setOccupation(voxel_to_evaluate, 0);
                }
            });

        // Now we have updated the occupations, check which evaluated voxels are to be processed next
        previously_evaluated_voxels.clear();
        voxels_to_evaluate.visit_all(
            std::execution::par,
            [&origin_voxels, &target_voxels, &voxel_space, &previously_evaluated_voxels](const VoxelGrid::LocalCoordinates& evaluated_voxel)
            {
                // First optional is for indicating that it has been set once, second is for whether the occupation actually contains an extruder nr
                std::optional<std::optional<uint8_t>> first_target_occupation;
                bool add_to_evaluate = false;
                origin_voxels.visit(
                    evaluated_voxel,
                    [&target_voxels, &add_to_evaluate, &voxel_space, &first_target_occupation](const auto& origin_voxel_value)
                    {
                        for (const VoxelGrid::LocalCoordinates& origin_voxel : origin_voxel_value.second)
                        {
                            target_voxels.visit(
                                origin_voxel,
                                [&voxel_space, &first_target_occupation, &add_to_evaluate](const auto& target_voxel_value)
                                {
                                    for (const VoxelGrid::LocalCoordinates& target_voxel : target_voxel_value.second)
                                    {
                                        const std::optional<uint8_t> target_occupation = voxel_space.getOccupation(target_voxel);
                                        if (! first_target_occupation.has_value())
                                        {
                                            first_target_occupation = target_occupation;
                                        }
                                        else if (first_target_occupation.value() != target_occupation)
                                        {
                                            add_to_evaluate = true;
                                            break;
                                        }
                                    }
                                });

                            if (add_to_evaluate)
                            {
                                break;
                            }
                        }
                    });

                if (add_to_evaluate)
                {
                    previously_evaluated_voxels.emplace(evaluated_voxel);
                }
            });

        // voxel_space.exportToFile(fmt::format("points_cloud_iteration_{}", iteration++));
    }

    spdlog::info("Making final points cloud");
    std::list<Point_3> points_cloud;
    voxel_space.visitFilledVoxels(
        [&points_cloud, &voxel_space](const auto& filled_voxel)
        {
            if (filled_voxel.second == 1)
            {
                points_cloud.push_back(voxel_space.toGlobalCoordinates(filled_voxel.first));
            }
        });
    exportPointsCloud(points_cloud, "final_contour");

    spdlog::info("Making mesh from points cloud");
    makeMeshFromPointsCloud(points_cloud, output_mesh, std::max({ resolution.x(), resolution.y(), resolution.z() }));
}

void splitMesh(Mesh& mesh, MeshGroup* meshgroup)
{
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

    const auto texture_data_provider = std::make_shared<TextureDataProvider>(nullptr, mesh.texture_, mesh.texture_data_mapping_);

    PolygonMesh output_mesh;
    makeModifierMeshVoxelSpace(converted_mesh, texture_data_provider, output_mesh);
    registerModifiedMesh(meshgroup, output_mesh);

    exportMesh(converted_mesh, "converted_mesh");
    exportMesh(output_mesh, "output_mesh");
}

} // namespace cura::MeshMaterialSplitter
