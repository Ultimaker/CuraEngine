// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/AdaptiveLayerHeights.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "Application.h"
#include "Slice.h"
#include "settings/EnumSettings.h"
#include "settings/types/Angle.h"
#include "utils/Point3D.h"

namespace cura
{

AdaptiveLayer::AdaptiveLayer(const coord_t layer_height)
    : layer_height_{ layer_height }
{
}

AdaptiveLayerHeights::AdaptiveLayerHeights(const coord_t base_layer_height, const coord_t variation, const coord_t step_size, const coord_t threshold, const MeshGroup* meshgroup)
    : base_layer_height_{ base_layer_height }
    , max_variation_{ variation }
    , step_size_{ step_size }
    , threshold_{ threshold }
    , meshgroup_{ meshgroup }
{
    calculateAllowedLayerHeights();
    calculateMeshTriangleSlopes();
    calculateLayers();
}

size_t AdaptiveLayerHeights::getLayerCount() const
{
    return layers_.size();
}

std::vector<AdaptiveLayer>* AdaptiveLayerHeights::getLayers()
{
    return &layers_;
}

void AdaptiveLayerHeights::calculateAllowedLayerHeights()
{
    // calculate the allowed layer heights from variation and step size
    // note: the order is from thickest to thinnest height!
    for (coord_t allowed_layer_height = base_layer_height_ + max_variation_; allowed_layer_height >= base_layer_height_ - max_variation_; allowed_layer_height -= step_size_)
    {
        // we should only consider using layer_heights that are > 0
        if (allowed_layer_height <= 0)
        {
            break;
        }
        allowed_layer_heights_.push_back(allowed_layer_height);
    }
}

void AdaptiveLayerHeights::calculateLayers()
{
    const coord_t minimum_layer_height = *std::min_element(allowed_layer_heights_.begin(), allowed_layer_heights_.end());
    Settings const& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    auto slicing_tolerance = mesh_group_settings.get<SlicingTolerance>("slicing_tolerance");
    std::vector<size_t> triangles_of_interest;
    const coord_t model_max_z = meshgroup_->max().z_;
    coord_t z_level = 0;
    coord_t previous_layer_height = 0;

    // the first layer has it's own independent height set, so we always add that
    const auto initial_layer_height = mesh_group_settings.get<coord_t>("layer_height_0");
    z_level += initial_layer_height;

    AdaptiveLayer adaptive_layer(initial_layer_height);
    adaptive_layer.z_position_ = z_level;
    previous_layer_height = adaptive_layer.layer_height_;
    layers_.push_back(adaptive_layer);

    // loop while triangles are found
    while (z_level <= model_max_z || layers_.size() < 2)
    {
        double global_min_slope = std::numeric_limits<double>::max();
        // loop over all allowed layer heights starting with the largest
        bool has_added_layer = false;
        for (auto& layer_height : allowed_layer_heights_)
        {
            // use lower and upper bounds to filter on triangles that are interesting for this potential layer
            const coord_t lower_bound = z_level;
            // if slicing tolerance "middle" is used, a layer is interpreted as the middle of the upper and lower bounds.
            const coord_t upper_bound = z_level + ((slicing_tolerance == SlicingTolerance::MIDDLE) ? (layer_height / 2) : layer_height);

            if (layer_height == allowed_layer_heights_[0])
            {
                // this is the max layer thickness, search through all of the triangles in the mesh to find those
                // that intersect with a layer this thick
                triangles_of_interest.clear();

                for (size_t i = 0; i < face_min_z_values_.size(); ++i)
                {
                    if (face_min_z_values_[i] <= upper_bound && face_max_z_values_[i] >= lower_bound)
                    {
                        triangles_of_interest.push_back(i);
                    }
                }
            }
            else
            {
                // this is a reduced thickness layer, just search those triangles that intersected with the layer
                // in the previous iteration
                const std::vector<size_t> last_triangles_of_interest = triangles_of_interest;

                triangles_of_interest.clear();

                for (const auto& i : last_triangles_of_interest)
                {
                    if (face_min_z_values_[i] <= upper_bound)
                    {
                        triangles_of_interest.push_back(i);
                    }
                }
            }

            // when there not interesting triangles in this potential layer go to the next one
            if (triangles_of_interest.empty())
            {
                break;
            }

            // find the minimum slope of all the interesting triangles
            double minimum_slope = std::numeric_limits<double>::max();
            for (const size_t& triangle_index : triangles_of_interest)
            {
                const double slope = face_slopes_.at(triangle_index);
                if (minimum_slope > slope)
                {
                    minimum_slope = slope;
                }
            }
            if (global_min_slope > minimum_slope)
            {
                global_min_slope = minimum_slope;
            }

            // check if the maximum step size has been exceeded depending on layer height direction
            bool has_exceeded_step_size = false;
            if (previous_layer_height > layer_height && previous_layer_height - layer_height > step_size_)
            {
                has_exceeded_step_size = true;
            }
            else if (layer_height - previous_layer_height > step_size_ && layer_height > minimum_layer_height)
            {
                continue;
            }

            // we add the layer in the following cases:
            // 1) the layer angle is below the threshold and the layer height difference with the previous layer is the maximum allowed step size
            // 2) the layer height is the smallest it is allowed
            // 3) the layer is a flat surface (we can't divide by 0)
            const double minimum_slope_tan = std::tan(minimum_slope);
            if (minimum_slope_tan == 0.0 || (layer_height / minimum_slope_tan) <= threshold_ || layer_height == minimum_layer_height || has_exceeded_step_size)
            {
                z_level += layer_height;
                AdaptiveLayer adaptive_layer_add(layer_height);
                adaptive_layer_add.z_position_ = z_level;
                previous_layer_height = adaptive_layer_add.layer_height_;
                layers_.push_back(adaptive_layer_add);
                has_added_layer = true;
                break;
            }
        }

        // this means we cannot find a layer height that has an angle lower than the threshold.
        // in this case, we use the layer height with the lowest
        if (! has_added_layer)
        {
            const auto& min_layer_height = allowed_layer_heights_.back();
            AdaptiveLayer minimum_adaptive_layer(min_layer_height);
            z_level += min_layer_height;
            minimum_adaptive_layer.z_position_ = z_level;
            previous_layer_height = min_layer_height;
            layers_.push_back(minimum_adaptive_layer);
        }
    }
}

void AdaptiveLayerHeights::calculateMeshTriangleSlopes()
{
    // loop over all mesh faces (triangles) and find their slopes
    for (const Mesh& mesh : Application::getInstance().current_slice_->scene.current_mesh_group->meshes)
    {
        // Skip meshes that are not printable
        if (mesh.settings_.get<bool>("infill_mesh") || mesh.settings_.get<bool>("cutting_mesh") || mesh.settings_.get<bool>("anti_overhang_mesh"))
        {
            continue;
        }

        for (const MeshFace& face : mesh.faces_)
        {
            const MeshVertex& v0 = mesh.vertices_[face.vertex_index_[0]];
            const MeshVertex& v1 = mesh.vertices_[face.vertex_index_[1]];
            const MeshVertex& v2 = mesh.vertices_[face.vertex_index_[2]];

            const Point3D p0 = v0.p_;
            const Point3D p1 = v1.p_;
            const Point3D p2 = v2.p_;

            double min_z = p0.z_;
            min_z = std::min(min_z, p1.z_);
            min_z = std::min(min_z, p2.z_);
            double max_z = p0.z_;
            max_z = std::max(max_z, p1.z_);
            max_z = std::max(max_z, p2.z_);

            // calculate the angle of this triangle in the z direction
            const Point3D n = (p1 - p0).cross(p2 - p0);
            const Point3D normal = n.normalized();
            AngleRadians z_angle = std::acos(std::abs(normal.z_));

            // prevent flat surfaces from influencing the algorithm
            if (z_angle == 0)
            {
                z_angle = std::numbers::pi;
            }

            face_min_z_values_.push_back(MM2INT(min_z));
            face_max_z_values_.push_back(MM2INT(max_z));
            face_slopes_.push_back(z_angle);
        }
    }
}

} // namespace cura
