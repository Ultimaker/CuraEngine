// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GRPCCOMMUNICATION_H
#define GRPCCOMMUNICATION_H

#ifdef BUILD_TESTS
#include <gtest/gtest_prod.h>
#endif
#include <memory> //For unique_ptr and shared_ptr.

#include "SliceDataStruct.h"
#include "communication/Communication.h" //The class we're implementing.

namespace google::protobuf
{

class Message;
template<class Field>
class RepeatedPtrField;

} // namespace google::protobuf

// Forward declarations to speed up compilation.
namespace cura
{

class Settings;
class Scene;
class MeshGroup;
class Mesh;
class Slice;
class Polygon;
class GrpcTransport;
class PathCompiler;

namespace proto
{

class SettingList;
class Extruder;
class MeshGroup;
class Object;
class Layer;
class Slice;

} // namespace proto

/*
 * \brief Communication class that connects via gRPC to Cura's front-end.
 */
class GrpcCommunication : public Communication
{
public:
    /*
     * \brief Construct a new gRPC client communicator.
     */
    GrpcCommunication();

    /*
     * \brief Closes the connection.
     */
    ~GrpcCommunication();

    /*
     * \brief Connect to Cura's streaming gRPC endpoint.
     * \param endpoint The endpoint to connect to, which should be something like "127.0.0.1:49674"
     */
    bool connect(const std::string& endpoint);

    /* \brief Sends a piece of GCode that is ready to be exported */
    void sendGCodePart(const std::string& gcode_part) override;

    /*
     * \brief Test if there are any more slices in the queue.
     */
    bool hasSlice() override;

    /*
     * \brief Send the current position to the front-end.
     *
     * This may indicate the starting position (or any other jump in the path).
     * \param position The current position to start the next line at.
     */
    void sendCurrentPosition(const Point3LL& position) override;

    /*
     * \brief Sends a message to indicate that all the slicing is done.
     *
     * This should indicate that no more data (g-code, prefix/postfix, metadata
     * or otherwise) should be sent any more regarding the last slice job.
     */
    void sendFinishedSlicing() const override;

    /*
     * \brief Send the uuid of the generated slice so that it may be processed by
     * the front-end.
     */
    void sendSliceUUID(const std::string& slice_uuid) const override;

    /*
     * \brief Indicate to the front-end that a layer is complete and send a
     * visualisation of the layer.
     *
     * This will be called after all the polygons and lines of this layer are
     * sent via sendLineTo. This will flush all
     * visualised data for one layer in one go.
     * \param layer_nr The layer that was completed.
     * \param z The z-coordinate of the top side of the layer.
     * \param thickness The thickness of the layer.
     */
    void sendLayerComplete(const LayerIndex::value_type& layer_nr, const coord_t& z, const coord_t& thickness) override;

    /*
     * \brief Send a line to the front-end to display in layer view.
     *
     * The line is not actually flushed until ``sendLayerComplete`` is called.
     * \param type The type of print feature the line represents (infill, wall,
     * support, etc).
     * \param to The destination coordinate of the line.
     * \param line_width The width of the line.
     * \param line_thickness The thickness (in the Z direction) of the line.
     * \param velocity The velocity of printing this polygon.
     * \param print_attributes Print attributes to be set for this segment, e.g. overhanging or bridging
     */
    void sendLineTo(
        const PrintFeatureType& type,
        const Point3LL& to,
        const coord_t& line_width,
        const coord_t& line_thickness,
        const Velocity& velocity,
        const PrintSegmentAttributes& print_attributes) override;

    /*
     * \brief Send the sliced layer data to the front-end after the optimisation
     * is done and the actual order in which to print has been set.
     *
     * This layer data will be shown in the layer view of the front end.
     */
    void sendOptimizedLayerData() override;

    /*
     * \brief Send an estimate of how long the print would take and how much material it would use.
     * \param time_estimates The calculated time estimations, per extruder
     * \param print_information The calculated materials consumptions, per extruder
     */
    void sendPrintInformation(const std::vector<Duration>& time_estimates, const PrintInformation& print_information) const override;

    /*
     * \brief Send progress to Cura.
     */
    void sendProgress(double progress) const override;

    /*
     * \brief Set which extruder is being used for the following calls to ``sendLineTo``.
     * \param extruder The new extruder to send data for.
     */
    void setExtruderForSend(const ExtruderTrain& extruder) override;

    /*
     * \brief Set which layer is being used for the following calls to ``sendLineTo``.
     * \param layer_nr The index of the layer to send data for. This is zero-
     * indexed but may be negative for raft layers.
     */
    void setLayerForSend(const LayerIndex::value_type& layer_nr) override;

    /*
     * \brief Slice the next scene that the front-end wants us to slice.
     */
    void sliceNext() override;

private:
    /*!
     * Make an empty Slice object, given a received slice message
     * @param slice_message The received messages
     * @return A Slice object, properly setup with the slice_message data
     */
    static std::shared_ptr<Slice> makeSlice(const proto::Slice& slice_message);

    /*!
     * Reads the global settings from a Protobuf message.
     * \param global_settings_message The received global settings message
     * \param settings The settings object to store the parsed values to
     */
    static void readGlobalSettingsMessage(const proto::SettingList& global_settings_message, Settings& settings);

    /*!
     * Reads the extruder settings from a Protobuf message
     * @param extruder_messages The received message containing the extruders settings
     * @param scene The scene object to store the parsed values to
     */
    static void readExtruderSettingsMessage(const google::protobuf::RepeatedPtrField<proto::Extruder>& extruder_messages, Scene& scene);

    /*!
     * \brief Reads a Protobuf message describing a mesh group.
     * \param mesh_group_message The received message containing the mesh group basic data
     * \param scene The scene object to store the parsed values to
     */
    static void readMeshGroupMessage(const proto::MeshGroup& mesh_group_message, Scene& scene);

    /*!
     * \brief Reads a Protobuf message describing an object on the build plate.
     * \param object_message The received message containing the object data
     * \param extruders The list of extruders already in the scene
     * \param mesh_group The mesh group to add the new object to
     */
    static void readObjectMessage(const proto::Object& object_message, const std::vector<ExtruderTrain>& extruders, MeshGroup& mesh_group);

    std::unique_ptr<GrpcTransport> grpc_transport_; //!< The transport object sending the actual messages sending/receiving
    std::unique_ptr<PathCompiler> path_compiler; //!< The compiler that generates optimized layer plans to be sent for display
    std::shared_ptr<Slice> current_slice_; //!< The Slice object created during a call to hasSlice()
    int last_sent_progress{ -1 }; //!< Last sent progress promille (1/1000th). Used to not send duplicate messages with the same promille.
    SliceDataStruct<proto::Layer> sliced_layers;
};

} // namespace cura

#endif // GRPCCOMMUNICATION_H
