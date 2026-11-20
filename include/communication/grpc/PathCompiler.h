// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>
#include <vector>

#include "Cura.pb.h"
#include "PrintFeatureType.h"
#include "PrintSegmentAttribute.h"
#include "SliceDataStruct.h"
#include "types/Coord_t.h"
#include "types/LayerIndex.h"
#include "types/Velocity.h"
#include "types/geometry/Point3LL.h"

namespace curaengine
{
class ExtruderTrain;
}

namespace curaengine::communication::grpc
{

/*
 * \brief A computation class that formats layer view data in a way that the
 * front-end can understand it.
 *
 * This converts data from CuraEngine's internal data structures to Protobuf
 * messages that can be sent to the front-end.
 */
class PathCompiler
{
    using PointType = ::cura::proto::PathSegment::PointType;
    static_assert(sizeof(PrintFeatureType) == 1, "To be compatible with the Cura frontend code PrintFeatureType needs to be of size 1");

    //! Keeps track of the current layer number being processed. If layer number is set to a different value, the current data is flushed to CommandSocket.
    types::LayerIndex _layer_nr{ 0 };
    size_t extruder{ 0 };
    PointType data_point_type{ ::cura::proto::PathSegment::Point3D };

    std::vector<PrintFeatureType> line_types; //!< Line types for the line segments stored, the size of this vector is N.
    std::vector<float> line_widths; //!< Line widths for the line segments stored, the size of this vector is N.
    std::vector<float> line_thicknesses; //!< Line thicknesses for the line segments stored, the size of this vector is N.
    std::vector<float> line_velocities; //!< Line feedrates for the line segments stored, the size of this vector is N.
    std::vector<float> points; //!< The points used to define the line segments, the size of this vector is D*(N+1) as each line segment is defined from one point to the next. D is
                               //!< the dimensionality of the point.
    std::vector<uint32_t> line_attributes; //! Indicates if the line is processed with specific settings, e.g. overhanging or bridging, the size of this vector is N.

    types::geometry::Point3LL last_point;
    SliceDataStruct<::cura::proto::LayerOptimized> optimized_layers;

    PathCompiler(const PathCompiler&) = delete;
    PathCompiler& operator=(const PathCompiler&) = delete;

public:
    /*
     * Create a new path compiler.
     */
    PathCompiler() = default;

    SliceDataStruct<::cura::proto::LayerOptimized>& getOptimizedLayers()
    {
        return optimized_layers;
    }

    /*!
     * \brief Used to select which layer the following layer data is intended
     * for.
     * \param new_layer_nr The new layer to switch to.
     */
    void setLayer(const types::LayerIndex& new_layer_nr);

    /*!
     * \brief Returns the current layer which data is written to.
     */
    int getLayer() const
    {
        return _layer_nr;
    }
    /*!
     * \brief Used to set which extruder will be used for printing the following
     * layer data.
     * \param new_extruder The new extruder to switch to.
     */
    void setExtruder(const ExtruderTrain& new_extruder);

    /*
     * Get the optimised layer data for a specific layer.
     * \param layer_nr The layer number to get the optimised layer data for.
     * \return The optimised layer data for that layer.
     */
    ::cura::proto::LayerOptimized* getOptimizedLayerById(types::LayerIndex::value_type layer_nr);

    /*!
     * \brief Special handling of the first point in an added line sequence.
     *
     * If the new sequence of lines does not start at the current end point
     * of the path this jump is marked as `PrintFeatureType::NoneType`.
     * \param from The initial point of a polygon.
     */
    void handleInitialPoint(const types::geometry::Point3LL& initial_point);

    /*!
     * \brief Transfers the currently buffered line segments to the layer
     * message storage.
     */
    void flushPathSegments();

    /*!
     * \brief Move the current point of this path to \p position.
     */
    void setCurrentPosition(const types::geometry::Point3LL& position);

    /*!
     * \brief Adds a single line segment to the current path.
     *
     * The line segment added is from the current last point to point \p to.
     * \param type The type of print feature the line represents (infill, wall,
     * support, etc).
     * \param to The destination coordinate of the line.
     * \param line_width The width of the line.
     * \param line_thickness The thickness (in the Z direction) of the line.
     * \param velocity The velocity of printing this polygon.
     * \param print_attributes Print attributes to be set for this segment, e.g. overhanging or bridging
     */
    void sendLineTo(
        const PrintFeatureType& print_feature_type,
        const types::geometry::Point3LL& to,
        const types::coord_t& width,
        const types::coord_t& thickness,
        const types::Velocity& feedrate,
        const PrintSegmentAttributes& print_attributes);

private:
    /*!
     * \brief Convert and add a point to the points buffer.
     *
     * Each point is represented as three consecutive floats. All members adding a
     * 3D point to the data should use this function.
     */
    void addPoint3D(const types::geometry::Point3LL& point);

    /*!
     * \brief Implements the functionality of adding a single 2D line segment to
     * the path data.
     *
     * All member functions adding a 2D line segment should use this functions.
     * \param print_feature_type The type of feature that the polygon is part of
     * (infill, wall, etc).
     * \param point The destination point of the line segment.
     * \param width The width of the lines of the polygon.
     * \param thickness The layer thickness of the polygon.
     * \param velocity How fast the polygon is printed.
     * \param print_attributes Print attributes to be set for this segment, e.g. overhanging or bridging
     */
    void addLineSegment(
        const PrintFeatureType& print_feature_type,
        const types::geometry::Point3LL& point,
        const types::coord_t& width,
        const types::coord_t& thickness,
        const types::Velocity& velocity,
        const PrintSegmentAttributes& print_attributes);
};

} // namespace curaengine::communication::grpc
