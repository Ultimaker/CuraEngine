// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/VoxelGrid.h"

#include <range/v3/algorithm/max.hpp>
#include <range/v3/algorithm/min.hpp>

#include "utils/AABB3D.h"
#include "utils/OBJ.h"
#include "utils/ParameterizedSegment.h"


namespace cura
{

VoxelGrid::VoxelGrid(const AABB3D& bounding_box, const coord_t max_resolution)
{
    origin_ = Point3D(bounding_box.min_.x_, bounding_box.min_.y_, bounding_box.min_.z_);

    auto set_resolution = [&max_resolution](coord_t& slices_count, double& resolution, const double span)
    {
        slices_count = static_cast<coord_t>(span / max_resolution) + 1;
        resolution = span / slices_count;
    };

    set_resolution(slices_count_.x_, resolution_.x_, bounding_box.spanX());
    set_resolution(slices_count_.y_, resolution_.y_, bounding_box.spanY());
    set_resolution(slices_count_.z_, resolution_.z_, bounding_box.spanZ());
}

Point3D VoxelGrid::toGlobalCoordinates(const LocalCoordinates& position, const bool at_center) const
{
    return Point3D(toGlobalX(position.position.x, at_center), toGlobalY(position.position.y, at_center), toGlobalZ(position.position.z, at_center));
}

void VoxelGrid::setOccupation(const LocalCoordinates& position, const uint8_t extruder_nr)
{
    occupied_voxels_.insert_or_assign(position, extruder_nr);
}

void VoxelGrid::setOrUpdateOccupation(const LocalCoordinates& position, const uint8_t extruder_nr)
{
    occupied_voxels_.insert_or_visit(
        { position, extruder_nr },
        [extruder_nr](auto& voxel)
        {
            voxel.second = std::min(voxel.second, extruder_nr);
        });
}

std::optional<uint8_t> VoxelGrid::getOccupation(const LocalCoordinates& local_position) const
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

bool VoxelGrid::hasOccupation(const LocalCoordinates& local_position) const
{
    return occupied_voxels_.contains(local_position);
}

size_t VoxelGrid::occupiedCount() const
{
    return occupied_voxels_.size();
}

std::vector<VoxelGrid::LocalCoordinates> VoxelGrid::getVoxelsAround(const LocalCoordinates& point) const
{
    const Point3U16& position = point.position;
    std::vector<LocalCoordinates> voxels_around;
    static constexpr uint8_t nb_voxels_around = 3 * 3 * 3 - 1;
    voxels_around.reserve(nb_voxels_around);

    for (int8_t delta_x = -1; delta_x < 2; ++delta_x)
    {
        const int64_t pos_x = position.x + delta_x;
        if (pos_x < 0 || pos_x >= slices_count_.x_)
        {
            continue;
        }

        for (int8_t delta_y = -1; delta_y < 2; ++delta_y)
        {
            const int64_t pos_y = position.y + delta_y;
            if (pos_y < 0 || pos_y >= slices_count_.y_)
            {
                continue;
            }

            for (int8_t delta_z = -1; delta_z < 2; ++delta_z)
            {
                const int64_t pos_z = position.z + delta_z;
                if (pos_z < 0 || pos_z >= slices_count_.z_)
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

VoxelGrid::LocalCoordinates VoxelGrid::toLocalCoordinates(const Point3D& position) const
{
    return LocalCoordinates(toLocalX(position.x_), toLocalY(position.y_), toLocalZ(position.z_));
}

uint16_t VoxelGrid::toLocalX(const double x) const
{
    return (x - origin_.x_) / resolution_.x_;
}

double VoxelGrid::toGlobalX(const uint16_t x, const bool at_center) const
{
    return (x * resolution_.x_) + origin_.x_ + (at_center ? resolution_.x_ / 2.0 : 0.0);
}

uint16_t VoxelGrid::toLocalY(const double y) const
{
    return (y - origin_.y_) / resolution_.y_;
}

double VoxelGrid::toGlobalY(const uint16_t y, const bool at_center) const
{
    return (y * resolution_.y_) + origin_.y_ + (at_center ? resolution_.y_ / 2.0 : 0.0);
}

uint16_t VoxelGrid::toLocalZ(const double z) const
{
    return (z - origin_.z_) / resolution_.z_;
}

double VoxelGrid::toGlobalZ(const uint16_t z, const bool at_center) const
{
    return (z * resolution_.z_) + origin_.z_ + (at_center ? resolution_.z_ / 2.0 : 0.0);
}

std::vector<VoxelGrid::LocalCoordinates> VoxelGrid::getTraversedVoxels(const Triangle3D& triangle) const
{
    const Point3U16 p0 = toLocalCoordinates(triangle[0]).position;
    const Point3U16 p1 = toLocalCoordinates(triangle[1]).position;
    const Point3U16 p2 = toLocalCoordinates(triangle[2]).position;

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

void VoxelGrid::saveToObj(const std::string& filename, const double scale) const
{
    OBJ obj(filename, scale);
    const double radius = std::min({ resolution_.x_, resolution_.y_, resolution_.z_ }) / 4.0;
    std::mutex mutex;

    visitOccupiedVoxels(
        [this, &mutex, &obj, &radius](const auto& voxel)
        {
            std::lock_guard lock(mutex);
            obj.writeSphere(toGlobalCoordinates(voxel.first), radius, static_cast<SVG::Color>(voxel.second), 2, 4);
        });
}

} // namespace cura
