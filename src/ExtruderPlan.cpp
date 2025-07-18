// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "ExtruderPlan.h"

#include "TextureDataProvider.h"

namespace cura
{
ExtruderPlan::ExtruderPlan(
    const size_t extruder,
    const LayerIndex layer_nr,
    const bool is_initial_layer,
    const bool is_raft_layer,
    const coord_t layer_thickness,
    const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings,
    const RetractionConfig& retraction_config)
    : extruder_nr_(extruder)
    , layer_nr_(layer_nr)
    , is_initial_layer_(is_initial_layer)
    , is_raft_layer_(is_raft_layer)
    , layer_thickness_(layer_thickness)
    , fan_speed_layer_time_settings_(fan_speed_layer_time_settings)
    , retraction_config_(retraction_config)
{
}

void ExtruderPlan::insertCommand(NozzleTempInsert&& insert)
{
    inserts_.emplace_back(insert);
}

void ExtruderPlan::handleInserts(const size_t path_idx, GCodeExport& gcode, const double cumulative_path_time)
{
    while (! inserts_.empty() && path_idx >= inserts_.front().path_idx && inserts_.front().time_after_path_start < cumulative_path_time)
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        inserts_.front().write(gcode);
        inserts_.pop_front();
    }
}

void ExtruderPlan::handleAllRemainingInserts(GCodeExport& gcode)
{
    while (! inserts_.empty())
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        NozzleTempInsert& insert = inserts_.front();
        insert.write(gcode);
        inserts_.pop_front();
    }
}

void ExtruderPlan::setFanSpeed(double _fan_speed)
{
    fan_speed = _fan_speed;
}
double ExtruderPlan::getFanSpeed()
{
    return fan_speed;
}

void ExtruderPlan::applyBackPressureCompensation(const Ratio back_pressure_compensation)
{
    constexpr double epsilon_speed_factor = 0.001; // Don't put on actual 'limit double minimum', because we don't want printers to stall.
    for (auto& path : paths_)
    {
        const double nominal_width_for_path = static_cast<double>(path.config.getLineWidth());
        if (path.width_factor <= 0.0 || nominal_width_for_path <= 0.0 || path.config.isTravelPath() || path.config.isBridgePath())
        {
            continue;
        }
        const double line_width_for_path = path.width_factor * nominal_width_for_path;
        path.speed_back_pressure_factor = std::max(epsilon_speed_factor, 1.0 + (nominal_width_for_path / line_width_for_path - 1.0) * back_pressure_compensation);
    }
}

void ExtruderPlan::applyIdLabel()
{
    // TODO?: message (format) should be a (string) setting, like 'ID: \H:\M:\S' or something

    constexpr coord_t inset_dist = 40; // TODO?: make this configurable as well?
    for (auto& path : paths_)
    {
        if (path.points.empty() ||
            path.mesh == nullptr ||
            path.mesh->layers[layer_nr_].texture_data_provider_ == nullptr ||
            (! path.mesh->id_field_info) ||
            (path.mesh->id_field_info.value().normal_ != IdFieldInfo::Axis::Z && path.config.type != PrintFeatureType::OuterWall) ||
            (path.mesh->id_field_info.value().normal_ == IdFieldInfo::Axis::Z && path.config.type != PrintFeatureType::Skin))
        {
            continue;
        }

        const auto zero_pt = Point3LL(0, 0, 0);
        const auto offset_pt = path.mesh->id_field_info.value().normal_offset(-inset_dist);

        std::vector<Point3LL> new_points;
        std::vector<bool> message_bits;
        new_points.push_back(path.points[0]);
        message_bits.push_back(false);
        for (const auto& window : path.points | ranges::views::sliding(2))
        {
            const auto& a = window[0];
            const auto& b = window[1];

            std::vector<TextureArea> span_pixels;
            if (path.mesh->layers[layer_nr_].texture_data_provider_->getAreaPreferencesForSpan(a.toPoint2LL(), b.toPoint2LL(), "label", span_pixels))
            {
                const auto pixel_span_3d = (b - a) / static_cast<coord_t>(span_pixels.size());
                auto last_pixel = TextureArea::Normal;
                auto last_pt = a;
                for (auto [idx, pixel] : span_pixels | ranges::views::enumerate)
                {
                    // TODO: Just make it 'random' for now -- later, use:
                    //  - A message/id of some sort.
                    //  - Some sort of simple 'text to pixels' method (lookup table?)
                    //  - IdFieldInfo (already made) to get the plane-normal(s) right (well, at least approximately -- it now does so only coursely, by axis)
                    const bool raw_val = (std::rand() % 2 == 0);
                    const auto raw_pt = a + (idx * pixel_span_3d) + (pixel_span_3d / 2);

                    const bool preferred = (pixel == TextureArea::Preferred);
                    if (preferred || last_pixel != pixel)
                    {
                        if (last_pixel != TextureArea::Preferred)
                        {
                            new_points.push_back(last_pt);
                            message_bits.push_back(false);
                        }

                        const bool val = preferred && raw_val;
                        new_points.push_back(raw_pt + (val ? offset_pt : zero_pt));
                        message_bits.push_back(val);
                    }
                    last_pixel = pixel;
                    last_pt = raw_pt;
                }
            }

            new_points.push_back(b);
            message_bits.push_back(false);
        }

        if (new_points.size() != path.points.size())
        {
            path.points = new_points;
            path.message_bit_per_point = message_bits;
        }
    }
}

std::shared_ptr<const SliceMeshStorage> ExtruderPlan::findFirstPrintedMesh() const
{
    for (const GCodePath& path : paths_)
    {
        if (path.mesh)
        {
            return path.mesh;
        }
    }

    return nullptr;
}

} // namespace cura
