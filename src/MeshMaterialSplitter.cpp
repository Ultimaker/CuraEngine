// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <execution>
#include <unordered_set>

#include <boost/unordered/concurrent_flat_map.hpp>
#include <boost/unordered/concurrent_flat_set.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/map.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "MeshGroup.h"
#include "Slice.h"
#include "TextureDataProvider.h"
#include "geometry/ClosedLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Shape.h"
#include "geometry/Triangle2F.h"
#include "geometry/Triangle3LL.h"
#include "mesh.h"
#include "progress/Progress.h"
#include "slicer.h"
#include "utils/MeshUtils.h"
#include "utils/OpenPolylineStitcher.h"
#include "utils/Point3D.h"
#include "utils/Simplify.h"
#include "utils/SpatialLookup.h"
#include "utils/ThreadPool.h"
#include "utils/VoxelGrid.h"


namespace cura::MeshMaterialSplitter
{

/*!
 * Utility structure that contains pre-calculated data to generate the multi-material modifiers of a mesh
 */
struct MeshGeneratorData
{
    const Mesh& mesh; //!< The mesh for which a material modifier is to be generated
    AABB3D bounding_box; //!< The bounding box of the mesh
    size_t estimated_material_iterations{ 0 }; //!< The roughly estimated number of multi-material voxels-propagation iterations (for progress reporting)
    size_t estimated_support_iterations{ 0 }; //!< The roughly estimated number of support-mesh voxels-propagation iterations (for progress reporting)
    coord_t depth; //!< The propagation depth retrieved from the mesh settings
    coord_t resolution; //!< The points cloud resolution retrieved from the mesh settings
};

/*!
 * Utility structure to store values in a map that are grouped by unique z-height and extruder number combinations
 */
union ContourKey
{
    uint32_t key;
    struct
    {
        uint16_t z;
        uint8_t value;
        uint8_t black_hole{ 0 }; // Don't place anything in there, or it would be lost forever (it exists only to properly set the 4th byte of the key)
    } definition;
};

inline std::size_t hash_value(ContourKey const& contour_key)
{
    return boost::hash<uint32_t>()(contour_key.key);
}

bool operator==(const ContourKey& key1, const ContourKey& key2)
{
    return key1.key == key2.key;
}

/*!
 * Fills the given voxels grid by setting an occupation everywhere the triangles of the mesh cross voxels. The extruder number is set according to the texture data
 * @param mesh The mesh to fill the voxels grid with
 * @param texture_data_provider The provider containing the painted texture data
 * @param voxel_grid The voxels grid to be filled with mesh data
 * @param default_value The main mesh extruder number
 * @return True if this generated relevant data for multi-extruder, otherwise this means the mesh is completely filled with only extruder 0 and there is no need to go further on
 *         trying to calculate the modified meshes.
 */
boost::concurrent_flat_set<uint8_t> makeVoxelGridFromTexture(
    const Mesh& mesh,
    const std::shared_ptr<TextureDataProvider>& texture_data_provider,
    const std::string& texture_feature,
    VoxelGrid& voxel_grid,
    const uint8_t default_value,
    const std::unordered_set<size_t>& authorized_values)
{
    spdlog::stopwatch timer;
    boost::concurrent_flat_set<uint8_t> found_values;

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
            const Triangle3LL triangle{ mesh.vertices_[face.vertex_index_[0]].p_, mesh.vertices_[face.vertex_index_[1]].p_, mesh.vertices_[face.vertex_index_[2]].p_ };

            for (const VoxelGrid::LocalCoordinates& traversed_voxel : voxel_grid.getTraversedVoxels(triangle))
            {
                const Point3D global_position = voxel_grid.toGlobalCoordinates(traversed_voxel);
                const Point3LL global_position_ll = Point3LL(std::llrint(global_position.x_), std::llrint(global_position.y_), std::llrint(global_position.z_));
                const std::optional<Point3D> barycentric_coordinates = MeshUtils::getBarycentricCoordinates(global_position_ll, triangle);
                if (! barycentric_coordinates.has_value() || barycentric_coordinates.value().x_ < 0 || barycentric_coordinates.value().y_ < 0
                    || barycentric_coordinates.value().z_ < 0)
                {
                    // Triangle is invalid, or point is outside the triangle
                    continue;
                }

                const Point2F point_uv_coords = MeshUtils::getUVCoordinates(barycentric_coordinates.value(), face_uvs);
                const std::pair<size_t, size_t> pixel = texture_data_provider->getTexture()->getPixelCoordinates(Point2F(point_uv_coords.x_, point_uv_coords.y_));
                std::optional<uint32_t> texture_value = texture_data_provider->getValue(std::get<0>(pixel), std::get<1>(pixel), texture_feature);
                if (! texture_value.has_value() || ! authorized_values.contains(texture_value.value()))
                {
                    texture_value = default_value;
                }

                voxel_grid.setOrUpdateOccupation(traversed_voxel, texture_value.value());
                found_values.insert(texture_value.value());
            }
        });

    spdlog::info("Total voxel grid creation time from texture {}ms", timer.elapsed_ms().count());
    return found_values;
}

/*!
 * Create modifier meshes from the given voxels grid, filled with the contours of the areas that should be processed by the different valuess.
 * @param voxel_grid The voxels grid containing the values occupations
 * @param ignore_value The "default" value for which no mesh should be created
 * @param mesh_settings The mesh settings to be applied to the generated meshes. If not provided, the occupation values will be considered as extruder number and we will
 *                      use the settings of the associated extruder.
 * @param is_hollow Indicates whether the occupied voxels have a hollow core and are surrounded by a solid shell of ignored values (which is the case for materials voxels grid)
 *                  or if they just form a solid block themselves (which is the case for support voxels grid)
 * @return A list of modifier meshes to be registered
 *
 * This function works by treating each horizontal plane separately of the voxels grid. For each plane, we apply a marching squares algorithm in order to generate 2D polygons.
 * Then we just have to extrude those polygons vertically. The final mesh has no horizontal face, thus it is not watertight at all. However, since it will subsequently
 * be re-sliced on XY planes, this is good enough.
 */
std::map<uint8_t, Mesh> makeMeshesFromVoxelsGrid(const VoxelGrid& voxel_grid, const uint8_t ignore_value, const std::optional<Settings>& mesh_settings, const bool is_hollow)
{
    spdlog::debug("Make modifier meshes from voxels grid");

    // First, gather all positions that should be considered for the marching square, e.g. all that have a specific value and around them
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> marching_squares;
    voxel_grid.visitOccupiedVoxels(
        [&marching_squares, &ignore_value](const auto& occupied_voxel)
        {
            if (occupied_voxel.second != ignore_value)
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

    // Build the list of segments to be added when running the marching squares
    const double half_res_x = voxel_grid.getResolution().x_ / 2;
    const double half_res_y = voxel_grid.getResolution().y_ / 2;
    std::array<OpenLinesSet, 16> marching_segments = {
        OpenLinesSet(), // This case is empty
        OpenLinesSet{ OpenPolyline({ Point2LL(0, half_res_y), Point2LL(half_res_x, 0) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(0, half_res_y) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(half_res_x, 0) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(0, -half_res_y) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(0, half_res_y), Point2LL(0, -half_res_y) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(0, half_res_y) }), OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(0, -half_res_y) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(-half_res_x, 0), Point2LL(0, -half_res_y) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(-half_res_x, 0) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(-half_res_x, 0) }), OpenPolyline({ Point2LL(0, half_res_y), Point2LL(half_res_x, 0) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(0, half_res_y) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(0, -half_res_y), Point2LL(half_res_x, 0) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(-half_res_x, 0) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(0, half_res_y), Point2LL(-half_res_x, 0) }) },
        OpenLinesSet{ OpenPolyline({ Point2LL(half_res_x, 0), Point2LL(0, half_res_y) }) },
        OpenLinesSet(), // This case is empty
    };

    // Now visit all the squares and generate the appropriate outer segments
    struct Contour
    {
        OpenLinesSet segments; // Single segments before stitching
        Shape polygons; // Assembled polygons
    };

    const Point3D position_delta_center(half_res_x, half_res_y, 0);
    boost::concurrent_flat_map<ContourKey, Contour> raw_contours;
    marching_squares.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&voxel_grid, &raw_contours, &position_delta_center, &marching_segments, &ignore_value, &is_hollow](const VoxelGrid::LocalCoordinates square_start)
        {
            const int32_t x_plus1 = static_cast<int32_t>(square_start.position.x) + 1;
            const bool x_plus1_valid = x_plus1 <= std::numeric_limits<uint16_t>::max();
            const int32_t y_plus1 = static_cast<int32_t>(square_start.position.y) + 1;
            const bool y_plus1_valid = y_plus1 <= std::numeric_limits<uint16_t>::max();

            std::unordered_set<uint8_t> filled_values;
            std::array<uint8_t, 4> occupation_bits;
            auto add_occupied_value = [&voxel_grid, &ignore_value, &filled_values, &occupation_bits, &square_start, &is_hollow](
                                          const int32_t x,
                                          const int32_t y,
                                          const bool position_valid,
                                          const size_t occupation_bit_index) -> void
            {
                if (position_valid)
                {
                    const std::optional<uint8_t> occupation = voxel_grid.getOccupation(VoxelGrid::LocalCoordinates(x, y, square_start.position.z));
                    if (occupation.has_value())
                    {
                        filled_values.insert(occupation.value());
                        occupation_bits[occupation_bit_index] = occupation.value();
                        return;
                    }
                }

                if (! is_hollow)
                {
                    filled_values.insert(ignore_value);
                }
                occupation_bits[occupation_bit_index] = ignore_value;
            };

            add_occupied_value(x_plus1, y_plus1, x_plus1_valid && y_plus1_valid, 0);
            add_occupied_value(square_start.position.x, y_plus1, y_plus1_valid, 1);
            add_occupied_value(x_plus1, square_start.position.y, x_plus1_valid, 2);
            add_occupied_value(square_start.position.x, square_start.position.y, true, 3);

            if (filled_values.size() < 2)
            {
                // Early-out, since this is not going to generate any segment
                return;
            }

            for (const uint8_t value : filled_values)
            {
                if (value == ignore_value)
                {
                    continue;
                }

                // Apply the marching squares base principle: calculate the index of the segments list to be added according to the occupations of the 4 positions
                const size_t segments_index = (occupation_bits[0] == value ? 1 : 0) + ((occupation_bits[1] == value ? 1 : 0) << 1) + ((occupation_bits[2] == value ? 1 : 0) << 2)
                                            + ((occupation_bits[3] == value ? 1 : 0) << 3);
                OpenLinesSet translated_segments = marching_segments[segments_index];

                if (translated_segments.empty())
                {
                    // Some cases don't generate segments, so don't bother doing any further calculation
                    continue;
                }

                // Now translate the segments according to the current position
                const Point3D center_position = voxel_grid.toGlobalCoordinates(square_start) + position_delta_center;
                const Point2LL center_position_ll(center_position.x_, center_position.y_);
                for (OpenPolyline& lines_set : translated_segments)
                {
                    for (Point2LL& point : lines_set)
                    {
                        point += center_position_ll;
                    }
                }

                // And finally, add the segments to the proper Z-plane and extruder
                raw_contours.emplace_or_visit(
                    std::make_pair(ContourKey{ .definition = { square_start.position.z, value } }, Contour{ .segments = translated_segments }),
                    [&translated_segments](auto& plane_contour)
                    {
                        plane_contour.second.segments.push_back(translated_segments);
                    });
            }
        });

    // Now we have added separate segments, stitch them to proper closed polygons
    raw_contours.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [](auto& raw_contour)
        {
            OpenLinesSet result_lines;
            OpenPolylineStitcher::stitch(raw_contour.second.segments, result_lines, raw_contour.second.polygons);
        });

    // Finally, simplify the polygons and extrude them vertically
    const double min_distance = std::min({ voxel_grid.getResolution().x_, voxel_grid.getResolution().y_, voxel_grid.getResolution().z_ }) / 2.0;
    Simplify simplifier(min_distance, min_distance / 2, std::numeric_limits<coord_t>::max());

    std::map<uint8_t, Mesh> meshes;
    std::mutex mutex;
    raw_contours.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&simplifier, &voxel_grid, &meshes, &mutex, &mesh_settings](const auto& contour)
        {
            const uint16_t z = contour.first.definition.z;
            const uint8_t value = contour.first.definition.value;
            const coord_t z_low = voxel_grid.toGlobalZ(z, false);
            const coord_t z_high = voxel_grid.toGlobalZ(z + 1, false);

            // Add a small offset to make sure overlapping edges won't let any space in between
            constexpr int offset_overlapping = 5;
            const Shape simplified_polygons = simplifier.polygon(contour.second.polygons).offset(offset_overlapping);
            for (const Polygon& simplified_polygon : simplified_polygons)
            {
                const std::lock_guard lock(mutex);
                const auto mesh_iterator = meshes.find(value);
                if (mesh_iterator == meshes.end())
                {
                    const Settings settings = mesh_settings.value_or(Application::getInstance().current_slice_->scene.extruders[value].settings_);
                    meshes.insert({ value, Mesh(settings) });
                }

                Mesh& mesh = meshes[value];
                for (auto iterator = simplified_polygon.beginSegments(); iterator != simplified_polygon.endSegments(); ++iterator)
                {
                    const Point2LL& start = (*iterator).start;
                    const Point2LL& end = (*iterator).end;
                    mesh.addFace(Point3LL(start, z_low), Point3LL(end, z_low), Point3LL(end, z_high));
                    mesh.addFace(Point3LL(end, z_high), Point3LL(start, z_high), Point3LL(start, z_low));
                }
            }
        });

    return meshes;
}

std::vector<Mesh> applyMeshExtruders(std::map<uint8_t, Mesh>& meshes)
{
    std::vector<Mesh> output_meshes;
    for (auto& [extruder_nr, mesh] : meshes)
    {
        mesh.settings_.add("cutting_mesh", "true");
        mesh.settings_.add("extruder_nr", std::to_string(extruder_nr));
        output_meshes.push_back(std::move(mesh));
    }

    return output_meshes;
}

std::vector<Mesh> applyMeshSupport(std::map<uint8_t, Mesh>& meshes, const size_t support_extruder_nr)
{
    std::vector<Mesh> output_meshes;
    for (auto& [support_value, mesh] : meshes)
    {
        if (support_value == 1)
        {
            mesh.settings_.add("force_support_overhang_mesh", "true");
            output_meshes.push_back(std::move(mesh));
        }
        else if (support_value == 2)
        {
            mesh.settings_.add("anti_overhang_mesh", "true");
            output_meshes.push_back(std::move(mesh));
        }
    }

    return output_meshes;
}

/*!
 * Pre-slice the mesh to match the given voxels grid, so that we have a matching Shape for each XY plane of the grid.
 * @param mesh The mesh to be pre-sliced
 * @param rasterized_mesh The voxels grid containing the rasterized mesh
 * @return A vector of shapes, that has as many elements as Z planes in the voxels grid
 */
std::vector<Shape> sliceMesh(const Mesh& mesh, const VoxelGrid& rasterized_mesh)
{
    const coord_t thickness = rasterized_mesh.getResolution().z_;
    const coord_t initial_layer_thickness = thickness;
    constexpr bool use_variable_layer_heights = false;
    constexpr std::vector<AdaptiveLayer>* adaptive_layers = nullptr;
    constexpr SlicingTolerance slicing_tolerance = SlicingTolerance::INCLUSIVE;

    // There is some margin in the voxel grid around the mesh, so get the actual mesh bounding box and see how many layers are actually covered
    AABB3D mesh_bounding_box = mesh.getAABB();
    const size_t margin_below_mesh = rasterized_mesh.toLocalZ(std::max(mesh_bounding_box.min_.z_, static_cast<coord_t>(0)));
    const size_t slice_layer_count = rasterized_mesh.toLocalZ(mesh_bounding_box.max_.z_) - margin_below_mesh + 1;

    const double xy_offset = INT2MM(std::max(rasterized_mesh.getResolution().x_, rasterized_mesh.getResolution().y_) * 2);
    Mesh sliced_mesh = mesh;
    sliced_mesh.settings_.add("xy_offset", std::to_string(xy_offset));
    sliced_mesh.settings_.add("xy_offset_layer_0", std::to_string(xy_offset));
    sliced_mesh.settings_.add("hole_xy_offset", "0");
    sliced_mesh.settings_.add("hole_xy_offset_max_diameter", "0");

    Slicer slicer(&sliced_mesh, thickness, slice_layer_count, use_variable_layer_heights, adaptive_layers, slicing_tolerance, initial_layer_thickness);

    // In order to re-create an offset on the Z direction, union the sliced shapes over a few layers so that we get an approximate outer shell of it
    std::vector<Shape> slices;
    for (std::ptrdiff_t layer_index = 0; layer_index < rasterized_mesh.getSlicesCount().z_; ++layer_index)
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

/*!
 * Checks whether the given position is inside the mesh, given the pre-sliced mesh
 * @param voxel_grid The voxels grid containing the rasterized mesh
 * @param position The position to be checked for insideness
 * @param sliced_mesh The pre-sliced mesh, containing a Shape for each Z-plane of the voxels grid
 * @return True if the position is inside the mesh or on the border, false if is outside
 */
bool isInside(const VoxelGrid& voxel_grid, const VoxelGrid::LocalCoordinates& position, const std::vector<Shape>& sliced_mesh)
{
    const Point3D global_position = voxel_grid.toGlobalCoordinates(position);
    constexpr bool border_result = true;
    return sliced_mesh.at(position.position.z).inside(Point2LL(global_position.x_, global_position.y_), border_result);
}

/*!
 * Find the voxels to be evaluated next, given the ones that have been previously evaluated
 * @param voxel_grid The current voxel grid to be checked
 * @param previously_evaluated_voxels The list of voxels that were just evaluated
 * @return The list of new voxels to be evaluated
 */
boost::concurrent_flat_set<VoxelGrid::LocalCoordinates>
    findVoxelsToEvaluate(const VoxelGrid& voxel_grid, const boost::concurrent_flat_set<VoxelGrid::LocalCoordinates>& previously_evaluated_voxels)
{
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> voxels_to_evaluate;
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> voxels_considered;

    previously_evaluated_voxels.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&](const VoxelGrid::LocalCoordinates& previously_evaluated_voxel)
        {
            for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_grid.getVoxelsAround(previously_evaluated_voxel))
            {
                if (voxels_considered.insert(voxel_around) && ! voxel_grid.hasOccupation(voxel_around))
                {
                    // Voxel has not been considered yet and is not filled, evaluate it now
                    voxels_to_evaluate.emplace(voxel_around);
                }
            }
        });

    return voxels_to_evaluate;
}

/*!
 * Evaluate the given voxels list, i.e. set their proper occupation
 * @param voxel_grid The voxel grid to be filled
 * @param voxels_to_evaluate The voxels to be evaluated
 * @param texture_data The lookup containing the rasterized texture data
 * @param sliced_mesh The pre-sliced mesh matching the voxel grid, or an empty list to skip the insideness check
 * @param depth_squared The maximum depth, squared
 * @param mesh_extruder_nr The main mesh extruder number
 */
void evaluateVoxels(
    VoxelGrid& voxel_grid,
    const boost::concurrent_flat_set<VoxelGrid::LocalCoordinates>& voxels_to_evaluate,
    const SpatialLookup& texture_data,
    const std::vector<Shape>& sliced_mesh,
    const coord_t depth_squared,
    const uint8_t mesh_extruder_nr)
{
    voxels_to_evaluate.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&voxel_grid, &texture_data, &sliced_mesh, &depth_squared, &mesh_extruder_nr](const VoxelGrid::LocalCoordinates& voxel_to_evaluate)
        {
            const Point3D position = voxel_grid.toGlobalCoordinates(voxel_to_evaluate);

            if (! sliced_mesh.empty() && ! isInside(voxel_grid, voxel_to_evaluate, sliced_mesh))
            {
                voxel_grid.setOccupation(voxel_to_evaluate, mesh_extruder_nr);
            }
            else
            {
                // Find the nearest neighbor
                const std::optional<OccupiedPosition> nearest_occupation = texture_data.findClosestOccupation(position);
                if (nearest_occupation.has_value())
                {
                    const Point3D diff = position - nearest_occupation.value().position;
                    const uint8_t new_occupation = diff.vSize2() <= depth_squared ? nearest_occupation.value().occupation : mesh_extruder_nr;
                    voxel_grid.setOccupation(voxel_to_evaluate, new_occupation);
                }
                else
                {
                    voxel_grid.setOccupation(voxel_to_evaluate, mesh_extruder_nr);
                }
            }
        });
}

/*!
 * From the given evaluated voxels list, keep only those that have various extruder values around them, so that we will only evaluate voxels on the borders
 * and skip those that grow inside the modifier meshes
 * @param evaluated_voxels The previously evaluated voxels
 * @param voxel_grid The voxel grid being filled
 */
void findBoundaryVoxels(boost::concurrent_flat_set<VoxelGrid::LocalCoordinates>& evaluated_voxels, const VoxelGrid& voxel_grid)
{
    evaluated_voxels.erase_if(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&voxel_grid](const VoxelGrid::LocalCoordinates& evaluated_voxel)
        {
            bool has_various_voxels_around = false;
            const uint8_t actual_occupation = voxel_grid.getOccupation(evaluated_voxel).value();
            for (const VoxelGrid::LocalCoordinates& voxel_around : voxel_grid.getVoxelsAround(evaluated_voxel))
            {
                const std::optional<uint8_t> around_occupation = voxel_grid.getOccupation(voxel_around);
                if (around_occupation.has_value() && around_occupation.value() != actual_occupation)
                {
                    has_various_voxels_around = true;
                    break;
                }
            }

            return ! has_various_voxels_around;
        });
}

/*!
 * Propagates the voxels occupations around the boundaries, according to the initially built grid
 * @param voxel_grid The voxel grid to be filled, initially containing the rasterized mesh data based on texture
 * @param evaluated_voxels The initial voxels list to be propagated
 * @param estimated_iterations The roughly estimated number of iterations that will be processed
 * @param sliced_mesh The pre-sliced mesh matching the voxel grid
 * @param texture_data The lookup containing the rasterized texture data
 * @param depth_squared The maximum propagation depth, squared
 * @param mesh_extruder_nr The main mesh extruder number
 * @param delta_iterations The number of already processed iterations over the total, for progress reporting
 * @param total_estimated_iterations The total number of iterations to be processed, for progress reporting
 */
void propagateVoxels(
    VoxelGrid& voxel_grid,
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates>& evaluated_voxels,
    const size_t estimated_iterations,
    const std::vector<Shape>& sliced_mesh,
    const SpatialLookup& texture_data,
    const coord_t depth_squared,
    const uint8_t mesh_extruder_nr,
    const size_t delta_iterations,
    const size_t total_estimated_iterations)
{
    size_t iteration = 0;

    while (! evaluated_voxels.empty())
    {
        Progress::messageProgress(Progress::Stage::SPLIT_MULTIMATERIAL, std::min(iteration, estimated_iterations) + delta_iterations, total_estimated_iterations);

        // Make the list of new voxels to be evaluated, based on which were evaluated before
        spdlog::debug("Finding voxels around {} voxels for iteration {}", evaluated_voxels.size(), iteration);
        evaluated_voxels = findVoxelsToEvaluate(voxel_grid, evaluated_voxels);

        // Now actually evaluate the candidate voxels, i.e. find their closest outside point and set the according occupation
        spdlog::debug("Evaluating {} voxels", evaluated_voxels.size());
        evaluateVoxels(voxel_grid, evaluated_voxels, texture_data, sliced_mesh, depth_squared, mesh_extruder_nr);

        // Now we have evaluated the candidates, check which of them are to be processed next. We skip all the voxels that have only voxels with similar occupations around
        // them, because they are obviously not part of the boundaries we are looking for. This avoids filling the inside of the points clouds and speeds up calculation a lot.
        spdlog::debug("Find boundary voxels for next round");
        findBoundaryVoxels(evaluated_voxels, voxel_grid);

        ++iteration;
    }
}

/*!
 * Generate a modifier mesh for every extruder other than 0, that has some user-painted texture data
 * @param mesh_data The generation data for the mesh to be processed
 * @param texture_data_provider The provider containing the texture painted data
 * @param delta_iterations The number of already processed iterations over the total, for progress reporting
 * @param total_estimated_iterations The total number of iterations to be processed, for progress reporting
 * @return A list of modifier meshes to be added to the slicing process
 */
std::vector<Mesh> makeMaterialModifierMeshes(
    const MeshGeneratorData& mesh_data,
    const std::shared_ptr<TextureDataProvider>& texture_data_provider,
    const size_t delta_iterations,
    const size_t total_estimated_iterations)
{
    const Settings& settings = mesh_data.mesh.settings_;
    const uint8_t mesh_extruder_nr = static_cast<uint8_t>(settings.get<size_t>("extruder_nr"));

    // Fill a first voxel grid by rasterizing the triangles of the mesh in 3D, and assign the extruders according to the texture. This way we can later evaluate which extruder
    // to assign any point in 3D space just by finding the closest outside point and see what extruder it is assigned to.
    spdlog::debug("Fill original voxels based on texture data");
    AABB3D bounding_box = mesh_data.bounding_box;
    bounding_box.expand(mesh_data.resolution * 8);

    // Create the voxel grid and initially fill it with the rasterized mesh triangles, which will be used as spatial reference for the texture data
    VoxelGrid voxel_grid(bounding_box, mesh_data.resolution);

    std::unordered_set<size_t> active_extruders;
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice_->scene.extruders)
    {
        active_extruders.insert(extruder.extruder_nr_);
    }

    const boost::concurrent_flat_set<uint8_t> found_extruders
        = makeVoxelGridFromTexture(mesh_data.mesh, texture_data_provider, "extruder", voxel_grid, mesh_extruder_nr, active_extruders);
    bool texture_contains_multimaterial_data = true;
    if (found_extruders.size() == 1)
    {
        // We have found only one extruder in the texture, so return true only if this extruder is not the mesh extruder, otherwise the rest is useless
        found_extruders.visit_all(
            [&texture_contains_multimaterial_data, &mesh_extruder_nr](const uint8_t extruder_nr)
            {
                texture_contains_multimaterial_data = extruder_nr != mesh_extruder_nr;
            });
    }
    if (! texture_contains_multimaterial_data)
    {
        // Texture is filled with the main extruder, don't bother doing anything
        return {};
    }

    spdlog::debug("Prepare spatial lookup for texture data");
    const SpatialLookup texture_data = SpatialLookup::makeSpatialLookupFromVoxelGrid(voxel_grid);

    const coord_t depth_squared = mesh_data.depth * mesh_data.depth;

    // Create a slice of the mesh so that we can quickly check for points insideness
    const std::vector<Shape> sliced_mesh = sliceMesh(mesh_data.mesh, voxel_grid);

    spdlog::debug("Get initially filled voxels");
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> previously_evaluated_voxels;
    voxel_grid.visitOccupiedVoxels(
        [&previously_evaluated_voxels, &mesh_extruder_nr](const auto& voxel)
        {
            if (voxel.second != mesh_extruder_nr)
            {
                previously_evaluated_voxels.insert(voxel.first);
            };
        });

    propagateVoxels(
        voxel_grid,
        previously_evaluated_voxels,
        mesh_data.estimated_material_iterations,
        sliced_mesh,
        texture_data,
        depth_squared,
        mesh_extruder_nr,
        delta_iterations,
        total_estimated_iterations);

    constexpr std::optional<Settings> mesh_settings = std::nullopt;
    constexpr bool is_hollow = true;
    std::map<uint8_t, Mesh> meshes = makeMeshesFromVoxelsGrid(voxel_grid, mesh_extruder_nr, mesh_settings, is_hollow);
    return applyMeshExtruders(meshes);
}

std::vector<Mesh> makeSupportModifierMeshes(const Mesh& mesh, const AABB3D& mesh_bounding_box, const std::shared_ptr<TextureDataProvider>& texture_data_provider)
{
    const Settings& settings = mesh.settings_;
    const auto resolution = settings.get<coord_t>("support_paint_resolution");
    constexpr uint8_t ignore_value = 0;
    const std::unordered_set<size_t>& authorized_values{ 1, 2 };

    // Create the voxel grid and initially fill it with the rasterized mesh triangles
    VoxelGrid voxel_grid(mesh_bounding_box, resolution);
    const boost::concurrent_flat_set<uint8_t> found_support_values = makeVoxelGridFromTexture(mesh, texture_data_provider, "support", voxel_grid, ignore_value, authorized_values);
    if (found_support_values.empty() || (found_support_values.size() == 1 && found_support_values.contains(ignore_value)))
    {
        // Texture is filled with only automatic support, don't bother doing anything
        return {};
    }

    // At this point, the voxel grid contains a 1-voxel-wide version of the textured mesh. In order to make proper modifier meshes that will cover the painted area, we will expand
    // it by 1 voxel in all direction, so it will end up being 3-voxels wide

    // Create the lookup structure that will help us expand only where necessary
    const SpatialLookup texture_data = SpatialLookup::makeSpatialLookupFromVoxelGrid(voxel_grid);

    // Extract the voxels have an actual support value. Others don't need to be expanded since they won't generate a mesh.
    boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> valued_voxels;
    voxel_grid.visitOccupiedVoxels(
        [&valued_voxels, &ignore_value](const auto& voxel)
        {
            if (voxel.second != ignore_value)
            {
                valued_voxels.insert(voxel.first);
            }
        });

    // Now get all the voxels around the valued voxels that have no occupation yet
    const boost::concurrent_flat_set<VoxelGrid::LocalCoordinates> outer_voxels = findVoxelsToEvaluate(voxel_grid, valued_voxels);

    // Finally, evaluate all the voxels around and set their proper occupation
    outer_voxels.visit_all(
#ifdef __cpp_lib_execution
        std::execution::par,
#endif
        [&voxel_grid, &texture_data, &ignore_value](const VoxelGrid::LocalCoordinates& voxel_to_evaluate)
        {
            const Point3D position = voxel_grid.toGlobalCoordinates(voxel_to_evaluate);

            // Find the nearest neighbor
            const std::optional<OccupiedPosition> nearest_occupation = texture_data.findClosestOccupation(position);
            if (nearest_occupation.has_value() && nearest_occupation.value().occupation != ignore_value)
            {
                voxel_grid.setOccupation(voxel_to_evaluate, nearest_occupation.value().occupation);
            }
        });

    constexpr bool is_hollow = false;
    const auto support_extruder_nr = settings.get<size_t>("support_extruder_nr");
    std::map<uint8_t, Mesh> meshes = makeMeshesFromVoxelsGrid(voxel_grid, ignore_value, mesh.settings_, is_hollow);
    return applyMeshSupport(meshes, support_extruder_nr);
}

/*!
 * Pre-calculate multi-material mesh generation data for the meshes in the given group
 * @param meshgroup The group containing the meshes to be processed
 * @return The list of mesh generation data for meshes that contain relevant information
 */
std::vector<MeshGeneratorData> makeInitialMeshesGenerationData(const MeshGroup* meshgroup)
{
    std::vector<MeshGeneratorData> result;

    for (const Mesh& mesh : meshgroup->meshes)
    {
        if (mesh.texture_ == nullptr || mesh.texture_data_mapping_ == nullptr)
        {
            continue;
        }

        const bool has_extruder_data = mesh.texture_data_mapping_->contains("extruder");
        const bool has_support_data = mesh.texture_data_mapping_->contains("support");
        if (! has_extruder_data && ! has_support_data)
        {
            continue;
        }

        const Settings& settings = mesh.settings_;
        MeshGeneratorData mesh_data{ .mesh = mesh,
                                     .depth = settings.get<coord_t>("multi_material_paint_depth"),
                                     .resolution = settings.get<coord_t>("multi_material_paint_resolution") };

        for (const MeshVertex& vertex : mesh.vertices_)
        {
            mesh_data.bounding_box.include(vertex.p_);
        }

        if (has_extruder_data)
        {
            // Make a rough estimation of the max number of iterations, by calculating how deep we may propagate inside the mesh
            const double bounding_box_max_depth = std::max({ mesh_data.bounding_box.spanX() / 2.0, mesh_data.bounding_box.spanY() / 2.0, mesh_data.bounding_box.spanZ() / 2.0 });
            const double estimated_min_depth = std::min(static_cast<double>(mesh_data.depth), bounding_box_max_depth);
            mesh_data.estimated_material_iterations = estimated_min_depth / mesh_data.resolution;
        }

        spdlog::debug("Estimated {} iterations for {}", mesh_data.estimated_material_iterations + mesh_data.estimated_support_iterations, mesh.mesh_name_);

        result.push_back(mesh_data);
    }

    return result;
}

void makePaintingModifierMeshes(MeshGroup* meshgroup)
{
    const std::vector<MeshGeneratorData> mesh_generation_data = makeInitialMeshesGenerationData(meshgroup);
    size_t delta_iterations = 0;
    const size_t total_estimated_iterations = ranges::accumulate(
        mesh_generation_data,
        0,
        [](const size_t total_iterations, const MeshGeneratorData& mesh_data)
        {
            return total_iterations + mesh_data.estimated_material_iterations;
        });

    std::vector<Mesh> modifier_meshes;
    for (const MeshGeneratorData& mesh_data : mesh_generation_data)
    {
        const Mesh& mesh = mesh_data.mesh;
        const spdlog::stopwatch timer;
        spdlog::info("Start painting mesh-modifier generation for {}", mesh.mesh_name_);

        const auto texture_data_provider = std::make_shared<TextureDataProvider>(nullptr, mesh.texture_, mesh.texture_data_mapping_);
        for (const Mesh& modifier_mesh : makeMaterialModifierMeshes(mesh_data, texture_data_provider, delta_iterations, total_estimated_iterations))
        {
            modifier_meshes.push_back(std::move(modifier_mesh));
        }

        for (const Mesh& modifier_mesh : makeSupportModifierMeshes(mesh_data.mesh, mesh_data.bounding_box, texture_data_provider))
        {
            modifier_meshes.push_back(std::move(modifier_mesh));
        }

        delta_iterations += mesh_data.estimated_material_iterations;
        spdlog::info("Painting mesh-modifier generation for {} took {} seconds", mesh.mesh_name_, timer.elapsed().count());
    }

    // Add meshes to group afterwards to avoid re-allocating the meshes in the vector
    for (Mesh& modifier_mesh : modifier_meshes)
    {
        meshgroup->meshes.push_back(std::move(modifier_mesh));
    }
}

} // namespace cura::MeshMaterialSplitter
