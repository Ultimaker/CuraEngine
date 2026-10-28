// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Slicer.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>

#include <fmt/chrono.h>
#include <scripta/logger.h>
#include <spdlog/spdlog.h>

#include "Mesh.h"
#include "Raft.h"
#include "geometry/Point3D.h"
#include "geometry/SingleShape.h"
#include "settings/AdaptiveLayer.h"
#include "settings/EPlatformAdhesion.h"
#include "settings/types/LayerIndex.h"
#include "utils/SectionType.h"
#include "utils/ThreadPool.h"
#include "utils/TimeKeeper.h"

namespace curaengine
{
Slicer::Slicer(
    Mesh* i_mesh,
    const coord_t thickness,
    const size_t slice_layer_count,
    bool use_variable_layer_heights,
    std::vector<AdaptiveLayer>* adaptive_layers,
    const SlicingTolerance slicing_tolerance,
    const coord_t initial_layer_thickness)
    : mesh(i_mesh)
{
    assert(slice_layer_count > 0);

    TimeKeeper slice_timer;

    layers = buildLayersWithHeight(slice_layer_count, slicing_tolerance, initial_layer_thickness, thickness, use_variable_layer_heights, adaptive_layers);
    scripta::setAll(
        layers,
        static_cast<int>(mesh->settings_.get<EPlatformAdhesion>("adhesion_type")),
        mesh->settings_.get<int>("raft_surface_layers"),
        mesh->settings_.get<coord_t>("raft_surface_thickness"),
        mesh->settings_.get<int>("raft_interface_layers"),
        mesh->settings_.get<coord_t>("raft_interface_thickness"),
        mesh->settings_.get<coord_t>("raft_base_thickness"),
        mesh->settings_.get<coord_t>("raft_airgap"),
        mesh->settings_.get<coord_t>("layer_0_z_overlap"),
        Raft::getFillerLayerCount());

    std::vector<std::pair<int32_t, int32_t>> zbbox = buildZHeightsForFaces(*mesh);

    buildSegments(*mesh, zbbox, slicing_tolerance, layers);

    spdlog::info("Slice of mesh took {}", std::chrono::duration<double>(slice_timer.restart()));

    makePolygons(*i_mesh, slicing_tolerance, layers);
    scripta::log("sliced_polygons", layers, SectionType::NA);
    spdlog::info("Make polygons took {}", std::chrono::duration<double>(slice_timer.restart()));
}

void Slicer::buildSegments(const Mesh& mesh, const std::vector<std::pair<int32_t, int32_t>>& zbbox, const SlicingTolerance& slicing_tolerance, std::vector<SlicerLayer>& layers)
{
    curaengine::parallel_for(
        layers,
        [&](auto layer_it)
        {
            SlicerLayer& layer = *layer_it;
            const int32_t& z = layer.z_;
            layer.segments_.reserve(100);

            // loop over all mesh faces
            for (unsigned int face_idx = 0; face_idx < mesh.faces_.size(); face_idx++)
            {
                if ((z < zbbox[face_idx].first) || (z > zbbox[face_idx].second))
                {
                    continue;
                }

                // get all vertices per face
                const MeshFace& face = mesh.faces_[face_idx];
                const MeshVertex& v0 = mesh.vertices_[face.vertex_index_[0]];
                const MeshVertex& v1 = mesh.vertices_[face.vertex_index_[1]];
                const MeshVertex& v2 = mesh.vertices_[face.vertex_index_[2]];
                const std::optional<Point2F> uv0 = face.uv_coordinates_[0];
                const std::optional<Point2F> uv1 = face.uv_coordinates_[1];
                const std::optional<Point2F> uv2 = face.uv_coordinates_[2];

                // get all vertices represented as 3D point
                Point3LL p0 = v0.p_;
                Point3LL p1 = v1.p_;
                Point3LL p2 = v2.p_;

                // Compensate for points exactly on the slice-boundary, except for 'inclusive', which already handles this correctly.
                if (slicing_tolerance != SlicingTolerance::INCLUSIVE)
                {
                    p0.z_ += static_cast<int>(p0.z_ == z) * -static_cast<int>(p0.z_ < 1);
                    p1.z_ += static_cast<int>(p1.z_ == z) * -static_cast<int>(p1.z_ < 1);
                    p2.z_ += static_cast<int>(p2.z_ == z) * -static_cast<int>(p2.z_ < 1);
                }

                SlicerSegment s;
                s.endVertex = nullptr;
                int end_edge_idx = -1;

                /*
                Now see if the triangle intersects the layer, and if so, where.

                Edge cases are important here:
                - If all three vertices of the triangle are exactly on the layer,
                  don't count the triangle at all, because if the model is
                  watertight, there will be adjacent triangles on all 3 sides that
                  are not flat on the layer.
                - If two of the vertices are exactly on the layer, only count the
                  triangle if the last vertex is going up. We can't count both
                  upwards and downwards triangles here, because if the model is
                  manifold there will always be an adjacent triangle that is going
                  the other way and you'd get double edges. You would also get one
                  layer too many if the total model height is an exact multiple of
                  the layer thickness. Between going up and going down, we need to
                  choose the triangles going up, because otherwise the first layer
                  of where the model starts will be empty and the model will float
                  in mid-air. We'd much rather let the last layer be empty in that
                  case.
                - If only one of the vertices is exactly on the layer, the
                  intersection between the triangle and the plane would be a point.
                  We can't print points and with a manifold model there would be
                  line segments adjacent to the point on both sides anyway, so we
                  need to discard this 0-length line segment then.
                - Vertices in ccw order if look from outside.
                */

                if (p0.z_ < z && p1.z_ > z && p2.z_ > z) //  1_______2
                { //   \     /
                    s = project2D(p0, p2, p1, uv0, uv2, uv1, z); //------------- z
                    end_edge_idx = 0; //     \ /
                } //      0

                else if (p0.z_ > z && p1.z_ <= z && p2.z_ <= z) //      0
                { //     / \      .
                    s = project2D(p0, p1, p2, uv0, uv1, uv2, z); //------------- z
                    end_edge_idx = 2; //   /     \    .
                    if (p2.z_ == z) //  1_______2
                    {
                        s.endVertex = &v2;
                    }
                }

                else if (p1.z_ < z && p0.z_ > z && p2.z_ > z) //  0_______2
                { //   \     /
                    s = project2D(p1, p0, p2, uv1, uv0, uv2, z); //------------- z
                    end_edge_idx = 1; //     \ /
                } //      1

                else if (p1.z_ > z && p0.z_ <= z && p2.z_ <= z) //      1
                { //     / \      .
                    s = project2D(p1, p2, p0, uv1, uv2, uv0, z); //------------- z
                    end_edge_idx = 0; //   /     \    .
                    if (p0.z_ == z) //  0_______2
                    {
                        s.endVertex = &v0;
                    }
                }

                else if (p2.z_ < z && p1.z_ > z && p0.z_ > z) //  0_______1
                { //   \     /
                    s = project2D(p2, p1, p0, uv2, uv1, uv0, z); //------------- z
                    end_edge_idx = 2; //     \ /
                } //      2

                else if (p2.z_ > z && p1.z_ <= z && p0.z_ <= z) //      2
                { //     / \      .
                    s = project2D(p2, p0, p1, uv2, uv0, uv1, z); //------------- z
                    end_edge_idx = 1; //   /     \    .
                    if (p1.z_ == z) //  0_______1
                    {
                        s.endVertex = &v1;
                    }
                }
                else
                {
                    // Not all cases create a segment, because a point of a face could create just a dot, and two touching faces
                    //   on the slice would create two segments
                    continue;
                }

                // store the segments per layer
                layer.face_idx_to_segment_idx_.insert(std::make_pair(face_idx, layer.segments_.size()));
                s.faceIndex = face_idx;
                s.endOtherFaceIdx = face.connected_face_index_[end_edge_idx];
                s.addedToPolygon = false;
                layer.segments_.push_back(s);
            }
        });
}

std::vector<SlicerLayer> Slicer::buildLayersWithHeight(
    size_t slice_layer_count,
    SlicingTolerance slicing_tolerance,
    coord_t initial_layer_thickness,
    coord_t thickness,
    bool use_variable_layer_heights,
    const std::vector<AdaptiveLayer>* adaptive_layers)
{
    std::vector<SlicerLayer> layers_res;

    layers_res.resize(slice_layer_count);

    // set (and initialize compensation for) initial layer, depending on slicing mode
    layers_res[0].z_ = slicing_tolerance == SlicingTolerance::INCLUSIVE ? 0 : std::max(0LL, initial_layer_thickness - thickness);
    coord_t adjusted_layer_offset = initial_layer_thickness;
    if (use_variable_layer_heights)
    {
        layers_res[0].z_ = (*adaptive_layers)[0].z_position_;
    }
    else if (slicing_tolerance == SlicingTolerance::MIDDLE)
    {
        layers_res[0].z_ = initial_layer_thickness / 2;
        adjusted_layer_offset = initial_layer_thickness + (thickness / 2);
    }

    // define all layer z positions (depending on slicing mode, see above)
    for (LayerIndex layer_nr = 1; layer_nr < slice_layer_count; layer_nr++)
    {
        if (use_variable_layer_heights)
        {
            layers_res[layer_nr].z_ = (*adaptive_layers)[layer_nr].z_position_;
        }
        else
        {
            layers_res[layer_nr].z_ = adjusted_layer_offset + (thickness * (layer_nr - 1));
        }
    }

    return layers_res;
}

void Slicer::makePolygons(Mesh& mesh, SlicingTolerance slicing_tolerance, std::vector<SlicerLayer>& layers)
{
    curaengine::parallel_for(
        layers,
        [&mesh](auto layer_it)
        {
            layer_it->makePolygons(&mesh);
        });

    switch (slicing_tolerance)
    {
    case SlicingTolerance::INCLUSIVE:
        for (size_t layer_nr = 0; layer_nr + 1 < layers.size(); layer_nr++)
        {
            layers[layer_nr].polygons_ = layers[layer_nr].polygons_.unionPolygons(layers[layer_nr + 1].polygons_);
        }
        break;
    case SlicingTolerance::EXCLUSIVE:
        for (size_t layer_nr = 0; layer_nr + 1 < layers.size(); layer_nr++)
        {
            layers[layer_nr].polygons_ = layers[layer_nr].polygons_.intersection(layers[layer_nr + 1].polygons_);
        }
        layers.back().polygons_.clear();
        break;
    case SlicingTolerance::MIDDLE:
    default:
        // do nothing
        ;
    }

    size_t layer_apply_initial_xy_offset = 0;
    if (layers.size() > 0 && layers[0].polygons_.size() == 0 && ! mesh.settings_.get<bool>("support_mesh") && mesh.isModelMesh())
    {
        layer_apply_initial_xy_offset = 1;
    }


    const coord_t xy_offset = mesh.settings_.get<coord_t>("xy_offset");
    const coord_t xy_offset_0 = mesh.settings_.get<coord_t>("xy_offset_layer_0");
    const coord_t xy_offset_hole = mesh.settings_.get<coord_t>("hole_xy_offset");
    const coord_t hole_offset_max_diameter = mesh.settings_.get<coord_t>("hole_xy_offset_max_diameter");

    const auto max_hole_area = std::numbers::pi / 4 * static_cast<double>(hole_offset_max_diameter * hole_offset_max_diameter);

    curaengine::parallel_for<size_t>(
        0,
        layers.size(),
        [&layers, layer_apply_initial_xy_offset, xy_offset, xy_offset_0, xy_offset_hole, hole_offset_max_diameter, max_hole_area](size_t layer_nr)
        {
            const auto xy_offset_local = (layer_nr <= layer_apply_initial_xy_offset) ? xy_offset_0 : xy_offset;
            if (xy_offset_local != 0)
            {
                layers[layer_nr].polygons_ = layers[layer_nr].polygons_.offset(xy_offset_local, ClipperLib::JoinType::jtRound);
            }
            if (xy_offset_hole != 0)
            {
                const auto parts = layers[layer_nr].polygons_.splitIntoParts();
                layers[layer_nr].polygons_.clear();

                for (const auto& part : parts)
                {
                    Shape holes;
                    Shape outline;
                    for (const Polygon& poly : part)
                    {
                        const auto area = poly.area();
                        const auto abs_area = std::abs(area);
                        const auto is_hole = area < 0;
                        if (is_hole)
                        {
                            if (hole_offset_max_diameter == 0)
                            {
                                holes.push_back(poly.offset(xy_offset_hole));
                            }
                            else if (abs_area < max_hole_area)
                            {
                                const auto distance = static_cast<int>(std::lerp(xy_offset_hole, 0, abs_area / max_hole_area));
                                holes.push_back(poly.offset(distance));
                            }
                            else
                            {
                                holes.push_back(poly);
                            }
                        }
                        else
                        {
                            outline.push_back(poly);
                        }
                    }

                    layers[layer_nr].polygons_.push_back(outline.difference(holes.unionPolygons()));
                }
            }
        });

    mesh.expandXY(xy_offset);
}


std::vector<std::pair<int32_t, int32_t>> Slicer::buildZHeightsForFaces(const Mesh& mesh)
{
    std::vector<std::pair<int32_t, int32_t>> zHeights;
    zHeights.reserve(mesh.faces_.size());
    for (const auto& face : mesh.faces_)
    {
        // const MeshFace& face = mesh.faces[mesh_idx];
        const MeshVertex& v0 = mesh.vertices_[face.vertex_index_[0]];
        const MeshVertex& v1 = mesh.vertices_[face.vertex_index_[1]];
        const MeshVertex& v2 = mesh.vertices_[face.vertex_index_[2]];

        // get all vertices represented as 3D point
        Point3LL p0 = v0.p_;
        Point3LL p1 = v1.p_;
        Point3LL p2 = v2.p_;

        // find the minimum and maximum z point
        int32_t minZ = p0.z_;
        if (p1.z_ < minZ)
        {
            minZ = p1.z_;
        }
        if (p2.z_ < minZ)
        {
            minZ = p2.z_;
        }

        int32_t maxZ = p0.z_;
        if (p1.z_ > maxZ)
        {
            maxZ = p1.z_;
        }
        if (p2.z_ > maxZ)
        {
            maxZ = p2.z_;
        }

        zHeights.emplace_back(std::make_pair(minZ, maxZ));
    }

    return zHeights;
}

SlicerSegment Slicer::project2D(
    const Point3LL& p0,
    const Point3LL& p1,
    const Point3LL& p2,
    const std::optional<Point2F>& uv0,
    const std::optional<Point2F>& uv1,
    const std::optional<Point2F>& uv2,
    const coord_t z)
{
    SlicerSegment seg;

    seg.start.X = interpolate(z, p0.z_, p1.z_, p0.x_, p1.x_);
    seg.start.Y = interpolate(z, p0.z_, p1.z_, p0.y_, p1.y_);
    seg.end.X = interpolate(z, p0.z_, p2.z_, p0.x_, p2.x_);
    seg.end.Y = interpolate(z, p0.z_, p2.z_, p0.y_, p2.y_);

    if (uv0.has_value() && uv1.has_value() && uv2.has_value())
    {
        const std::optional<Point3D> start_barycentric = getBarycentricCoordinates(Point3LL(seg.start, z), p0, p1, p2);
        const std::optional<Point3D> end_barycentric = getBarycentricCoordinates(Point3LL(seg.end, z), p0, p1, p2);

        if (start_barycentric.has_value() && end_barycentric.has_value())
        {
            seg.uv_start = interpolateUV(start_barycentric.value(), uv0.value(), uv1.value(), uv2.value());
            seg.uv_end = interpolateUV(end_barycentric.value(), uv0.value(), uv1.value(), uv2.value());
        }
    }

    return seg;
}

std::optional<Point3D> Slicer::getBarycentricCoordinates(const Point3LL& point, const Point3LL& p0, const Point3LL& p1, const Point3LL& p2)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const Point3LL v0(p1 - p0);
    const Point3LL v1(p2 - p0);
    const Point3LL v2(point - p0);

    // Compute dot products
    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);

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
    return Point3D(u, v, w);
}

Point2F Slicer::interpolateUV(const Point3D& barycentric_coordinates, const Point2F& uv0, const Point2F& uv1, const Point2F& uv2)
{
    return Point2F(
        barycentric_coordinates.x_ * uv0.x_ + barycentric_coordinates.y_ * uv1.x_ + barycentric_coordinates.z_ * uv2.x_,
        barycentric_coordinates.x_ * uv0.y_ + barycentric_coordinates.y_ * uv1.y_ + barycentric_coordinates.z_ * uv2.y_);
}

coord_t Slicer::interpolate(const coord_t x, const coord_t x0, const coord_t x1, const coord_t y0, const coord_t y1)
{
    const coord_t dx_01 = x1 - x0;
    coord_t num = (y1 - y0) * (x - x0);
    num += num > 0 ? dx_01 / 4 : -dx_01 / 4; // add in offset to round result
    return y0 + num / dx_01;
}

} // namespace curaengine
