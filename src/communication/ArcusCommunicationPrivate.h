//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ARCUSCOMMUNICATIONPRIVATE_H
#define ARCUSCOMMUNICATIONPRIVATE_H
#ifdef ARCUS

#include <sstream> //For ostringstream.

#include "ArcusCommunication.h" //We're adding a subclass to this.
#include "SliceDataStruct.h"

namespace cura
{

struct LayerIndex;

class ArcusCommunication::Private
{
    friend class ArcusCommunicationPrivateTest;
public:
    Private();

    /*
     * Get the optimised layer data for a specific layer.
     * \param layer_nr The layer number to get the optimised layer data for.
     * \return The optimised layer data for that layer.
     */
    std::shared_ptr<proto::LayerOptimized> getOptimizedLayerById(LayerIndex layer_nr);

    /*
     * Reads the global settings from a Protobuf message.
     *
     * The global settings are stored in the current scene.
     * \param global_settings_message The global settings message.
     */
    void readGlobalSettingsMessage(const proto::SettingList& global_settings_message);

    void readExtruderSettingsMessage(const google::protobuf::RepeatedPtrField<proto::Extruder>& extruder_messages);

    /*
     * \brief Reads a Protobuf message describing a mesh group.
     *
     * This gets the vertex data from the message as well as the settings.
     */
    void readMeshGroupMessage(const proto::ObjectList& mesh_group_message);

    Arcus::Socket* socket; //!< Socket to send data to.
    size_t object_count; //!< Number of objects that need to be sliced.
    std::string temp_gcode_file; //!< Temporary buffer for the g-code.
    std::ostringstream gcode_output_stream; //!< The stream to write g-code to.

    SliceDataStruct<cura::proto::Layer> sliced_layers;
    SliceDataStruct<cura::proto::LayerOptimized> optimized_layers;

    int last_sent_progress; //!< Last sent progress promille (1/1000th). Used to not send duplicate messages with the same promille.

    /*
     * \brief How often we've sliced so far during this run of CuraEngine.
     *
     * This is currently used to limit the number of slices per run to 1,
     * because CuraEngine produced different output for each slice. The fix was
     * to restart CuraEngine every time you make a slice.
     *
     * Once this bug is resolved, we can allow multiple slices for each run. Our
     * intuition says that there might be some differences if we let stuff
     * depend on the order of iteration in unordered_map or unordered_set,
     * because those data structures will give a different order if more memory
     * has already been reserved for them.
     */
    size_t slice_count; //!< How often we've sliced so far during this run of CuraEngine.

    const size_t millisecUntilNextTry; // How long we wait until we try to connect again.
};

} //namespace cura

#endif //ARCUS
#endif //ARCUSCOMMUNICATIONPRIVATE_H