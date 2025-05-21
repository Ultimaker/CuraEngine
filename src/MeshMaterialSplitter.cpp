// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>

#include <boost/unordered/concurrent_flat_map.hpp>
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
using Triangle_2 = Kernel::Triangle_2;
using Triangle_3 = Kernel::Triangle_3;
using Vector_3 = Kernel::Vector_3;
using PolygonMesh = CGAL::Surface_mesh<Kernel::Point_3>;

using Kernel_U32 = CGAL::Simple_cartesian<uint32_t>;
using Point_2U32 = Kernel_U32::Point_2;


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

enum class ExtruderOccupation : uint8_t
{
    Unknown = 0,
    InsideMesh = 1, // Voxel is tagged as being inside the mesh, but not assigned yet
    OutsideMesh = 2, // Voxel is tagged as being outside the mesh
    Occupied = 3, // When occupied, the actual extruder value is the value - Occupied
};

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

    static ExtruderOccupation makeOccupation(const size_t extruder_nr)
    {
        return static_cast<ExtruderOccupation>(static_cast<uint8_t>(ExtruderOccupation::Occupied) + extruder_nr);
    }

    void setExtruderNr(const Point_3& position, const size_t extruder_nr)
    {
        setOccupation(toLocalCoordinates(position), makeOccupation(extruder_nr));
    }

    void setOccupation(const LocalCoordinates& position, const ExtruderOccupation& occupation)
    {
        nodes_.insert_or_assign(position, occupation);
    }

    uint8_t getExtruderNr(const LocalCoordinates& local_position) const
    {
        return static_cast<uint8_t>(getOccupation(local_position)) - static_cast<uint8_t>(ExtruderOccupation::Occupied);
    }

    bool isFilled(const LocalCoordinates& local_position) const
    {
        return nodes_.contains(local_position);
    }

    ExtruderOccupation getOccupation(const LocalCoordinates& local_position) const
    {
        ExtruderOccupation result = ExtruderOccupation::Unknown;
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
        nodes_.visit_all(args...);
    }

    std::vector<LocalCoordinates> getFilledVoxels() const
    {
        std::vector<LocalCoordinates> filled_voxels;

        nodes_.visit_all(
            [&filled_voxels](const auto& voxel)
            {
                filled_voxels.push_back(voxel.first);
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
                        voxels_around.emplace_back(LocalCoordinates(pos_x, pos_y, pos_z));
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

private:
    Point_3 resolution_;
    Vector_3 origin_;
    uint32_t max_coordinate_;
    boost::concurrent_flat_map<LocalCoordinates, ExtruderOccupation> nodes_;
};

std::size_t hash_value(VoxelGrid::LocalCoordinates const& position)
{
    return boost::hash<uint64_t>()(position.key);
}

void makeInitialVoxelSpaceFromTexture(const PolygonMesh& mesh, const png::image<png::rgb_pixel>& image, VoxelGrid& voxel_space)
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
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;

    spdlog::info("Fill original voxels based on texture data");
    VoxelGrid voxel_space(mesh, settings.get<double>("multi_material_paint_resolution") * 1000.0);
    makeInitialVoxelSpaceFromTexture(mesh, image, voxel_space);

    const double deepness = settings.get<double>("multi_material_paint_deepness") * 1000.0;
    const Point_3& resolution = voxel_space.getResolution();
    const uint16_t max_shells = deepness / ((resolution.x() + resolution.y() + resolution.z()) / 3.0);

    spdlog::info("Get initially filled voxels");
    std::vector<VoxelGrid::LocalCoordinates> filled_voxels = voxel_space.getFilledVoxels();

    spdlog::info("Export initial meshes");
    std::map<size_t, PolygonMesh> initial_meshes;
    for (const VoxelGrid::LocalCoordinates& local_coord : filled_voxels)
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
    bool check_inside = true;
    uint16_t shells = 0;

    while (! filled_voxels.empty() && shells < max_shells)
    {
        spdlog::info("Voting for {} voxels", filled_voxels.size());

        boost::concurrent_flat_map<VoxelGrid::LocalCoordinates, std::map<ExtruderOccupation, uint8_t>> voxels_votes;

        std::atomic_bool keep_checking_inside(false);

        cura::parallel_for(
            filled_voxels,
            [&](auto iterator)
            {
                const VoxelGrid::LocalCoordinates& filled_voxel = *iterator;
                const ExtruderOccupation actual_filled_occupation = voxel_space.getOccupation(filled_voxel);

                for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_space.getVoxelsAround(filled_voxel))
                {
                    bool vote_for_voxel;
                    if (check_inside)
                    {
                        voxel_space.setOccupationIfUnknown(
                            voxel_around,
                            inside_mesh(voxel_space.toGlobalCoordinates(voxel_around)) != CGAL::ON_UNBOUNDED_SIDE ? ExtruderOccupation::InsideMesh
                                                                                                                  : ExtruderOccupation::OutsideMesh);
                        const ExtruderOccupation occupation = voxel_space.getOccupation(voxel_around);
                        vote_for_voxel = (occupation == ExtruderOccupation::InsideMesh);
                        if (occupation == ExtruderOccupation::OutsideMesh)
                        {
                            // As long as we find voxels outside the mesh, keep checking for it. Once we have no single candidate outside, this means the outer shell
                            // is complete and we are only growing inside, thus we can skip checking for insideness
                            keep_checking_inside.store(true);
                        }
                    }
                    else
                    {
                        vote_for_voxel = ! voxel_space.isFilled(voxel_around);
                    }

                    if (vote_for_voxel)
                    {
                        // Voxel is not occupied yet, so vote to fill it
                        voxels_votes.emplace_or_visit(
                            voxel_around,
                            std::map<ExtruderOccupation, uint8_t>{ { actual_filled_occupation, 1 } },
                            [&actual_filled_occupation](auto& voxel_votes)
                            {
                                ++voxel_votes.second[actual_filled_occupation];
                            });
                    }
                }
            });

        if (check_inside && ! keep_checking_inside.load())
        {
            spdlog::info("Stop checking for voxels insideness");
            check_inside = false;
        }
        if (! check_inside)
        {
            shells++;
        }

        spdlog::info("Apply growing with {} voted voxels", voxels_votes.size());

        std::vector<VoxelGrid::LocalCoordinates> new_filled_voxels;
        new_filled_voxels.reserve(voxels_votes.size());

        voxels_votes.visit_all(
            [&voxel_space, &new_filled_voxels](const auto& voxel_votes)
            {
                const std::map<ExtruderOccupation, uint8_t>& occupation_votes = voxel_votes.second;
                ExtruderOccupation most_voted_occupation;
                if (occupation_votes.size() == 1)
                {
                    most_voted_occupation = occupation_votes.begin()->first;
                }
                else
                {
                    const auto max_vote = ranges::max_element(
                        occupation_votes,
                        [](const auto& vote_a, const auto& vote_b)
                        {
                            return vote_a.second < vote_b.second;
                        });
                    most_voted_occupation = max_vote->first;
                }
                voxel_space.setOccupation(voxel_votes.first, most_voted_occupation);
                new_filled_voxels.push_back(voxel_votes.first);
            });

        filled_voxels = std::move(new_filled_voxels);
    }

    spdlog::info("Making final points cloud");
    std::list<Point_3> points_cloud;
    ExtruderOccupation occupation_extruder_1 = VoxelGrid::makeOccupation(1);
    voxel_space.visitFilledVoxels(
        [&occupation_extruder_1, &points_cloud, &voxel_space](const auto& filled_voxel)
        {
            if (filled_voxel.second == occupation_extruder_1)
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
    // png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/texture.png");
    // png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/texture-high.png");
    png::image<png::rgb_pixel> image("/home/erwan/test/CURA-12449_handling-painted-models/dino-texture.png");

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
