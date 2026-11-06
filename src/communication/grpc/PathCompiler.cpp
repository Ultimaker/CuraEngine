// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/grpc/PathCompiler.h"

#include "ExtruderTrain.h"
#include "settings/types/Velocity.h" //To send to layer view how fast stuff is printing.

namespace cura
{

void PathCompiler::setLayer(const LayerIndex& new_layer_nr)
{
    if (_layer_nr != new_layer_nr)
    {
        flushPathSegments();
        _layer_nr = new_layer_nr;
    }
}

void PathCompiler::setExtruder(const ExtruderTrain& new_extruder)
{
    if (extruder != new_extruder.extruder_nr_)
    {
        flushPathSegments();
        extruder = new_extruder.extruder_nr_;
    }
}

proto::LayerOptimized* PathCompiler::getOptimizedLayerById(LayerIndex::value_type layer_nr)
{
    layer_nr += optimized_layers.current_layer_offset;
    const auto find_result = optimized_layers.slice_data.find(layer_nr);

    if (find_result != optimized_layers.slice_data.end()) // Load layer from the cache.
    {
        return &find_result->second;
    }
    else // Not in the cache yet. Create an empty layer.
    {
        auto [iterator, inserted] = optimized_layers.slice_data.try_emplace(layer_nr);
        iterator->second.set_id(layer_nr);
        optimized_layers.current_layer_count++;
        return &iterator->second;
    }
}

void PathCompiler::handleInitialPoint(const Point3LL& initial_point)
{
    if (points.size() == 0)
    {
        addPoint3D(initial_point);
    }
    else if (initial_point != last_point)
    {
        addLineSegment(PrintFeatureType::NoneType, initial_point, 1, 0, 0.0, {});
    }
}

void PathCompiler::flushPathSegments()
{
    if (line_types.empty())
    {
        return; // Nothing to do.
    }

    // proto::LayerOptimized* proto_layer = _cs_private_data.getOptimizedLayerById(_layer_nr);
    proto::LayerOptimized* proto_layer = getOptimizedLayerById(_layer_nr);

    proto::PathSegment* path_segment = proto_layer->add_path_segment();
    path_segment->set_extruder(extruder);
    path_segment->set_point_type(data_point_type);

    std::string line_type_data;
    line_type_data.append(reinterpret_cast<const char*>(line_types.data()), line_types.size() * sizeof(PrintFeatureType));
    line_types.clear();
    path_segment->set_line_type(line_type_data);

    std::string polygon_data;
    polygon_data.append(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(float));
    points.clear();
    path_segment->set_points(polygon_data);

    std::string line_width_data;
    line_width_data.append(reinterpret_cast<const char*>(line_widths.data()), line_widths.size() * sizeof(float));
    line_widths.clear();
    path_segment->set_line_width(line_width_data);

    std::string line_thickness_data;
    line_thickness_data.append(reinterpret_cast<const char*>(line_thicknesses.data()), line_thicknesses.size() * sizeof(float));
    line_thicknesses.clear();
    path_segment->set_line_thickness(line_thickness_data);

    std::string line_velocity_data;
    line_velocity_data.append(reinterpret_cast<const char*>(line_velocities.data()), line_velocities.size() * sizeof(float));
    line_velocities.clear();
    path_segment->set_line_feedrate(line_velocity_data);

    for (const uint32_t line_attribute : line_attributes)
    {
        path_segment->add_attributes(line_attribute);
    }
    line_attributes.clear();
}

void PathCompiler::setCurrentPosition(const Point3LL& position)
{
    handleInitialPoint(position);
}

void PathCompiler::sendLineTo(
    const PrintFeatureType& print_feature_type,
    const Point3LL& to,
    const coord_t& width,
    const coord_t& thickness,
    const Velocity& feedrate,
    const PrintSegmentAttributes& print_attributes)
{
    assert(! points.empty() && "A point must already be in the buffer for sendLineTo(.) to function properly.");

    if (to != last_point)
    {
        addLineSegment(print_feature_type, to, width, thickness, feedrate, print_attributes);
    }
}

void PathCompiler::addPoint3D(const Point3LL& point)
{
    points.push_back(INT2MM(point.x_));
    points.push_back(INT2MM(point.y_));
    points.push_back(INT2MM(point.z_));
    last_point = point;
}

void PathCompiler::addLineSegment(
    const PrintFeatureType& print_feature_type,
    const Point3LL& point,
    const coord_t& width,
    const coord_t& thickness,
    const Velocity& velocity,
    const PrintSegmentAttributes& print_attributes)
{
    addPoint3D(point);
    line_types.push_back(print_feature_type);
    line_widths.push_back(INT2MM(width));
    line_thicknesses.push_back(INT2MM(thickness));
    line_velocities.push_back(velocity);
    line_attributes.push_back(static_cast<uint32_t>(print_attributes.value()));
}

}; // namespace cura
