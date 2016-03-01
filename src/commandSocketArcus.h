#ifndef COMMAND_SOCKET_ARCUS_H
#define COMMAND_SOCKET_ARCUS_H

#ifdef ARCUS

#include "commandSocket.h"

#include "FffProcessor.h"
#include "Progress.h"

#include "Arcus/Socket.h"
#include "Arcus/SocketListener.h"
#include "Arcus/Error.h"


#include "Cura.pb.h"

namespace cura
{

class CommandSocketArcus : public CommandSocket
{
public:
    void sendLayerInfo(int layer_nr, int32_t z, int32_t height) override;
    void sendPolygons(cura::PrintFeatureType type, int layer_nr, cura::Polygons& polygons, int line_width) override;
    void sendProgress(float amount) override;
    void sendProgressStage(Progress::Stage stage) override;
    void sendPrintTime() override;
    void sendPrintMaterialForObject(int index, int extruder_nr, float material_amount) override;
    void beginSendSlicedObject() override;
    void endSendSlicedObject() override;
    void sendFinishedSlicing() override;
    void beginGCode() override;
    void flushGcode() override;
    void sendGCodePrefix(std::string prefix) override;

    /*!
     * Handler for ObjectList message.
     * Loads all objects from the message and starts the slicing process
     */
    void handleObjectList(cura::proto::ObjectList* list);

    /*!
     * Handler for SettingList message.
     * This simply sets all the settings by using key value pair
     */
    void handleSettingList(cura::proto::SettingList* list);

	/*!
     * Connect with the GUI
     * This creates and initialises the arcus socket and then continues listening for messages.
     * \param ip string containing the ip to connect with
     * \param port int of the port to connect with.
     */
    void connect(const std::string& ip, int port);

    CommandSocketArcus();
private:
    cura::proto::Layer* getLayerById(int id);

    Arcus::Socket* socket_;

    // Number of objects that need to be sliced
    int object_count_;

    // Message that holds a list of sliced objects
    std::shared_ptr<cura::proto::SlicedObjectList> sliced_object_list_;

    // Message that holds the currently sliced object (to be added to sliced_object_list)
    cura::proto::SlicedObject* current_sliced_object_;

    // Number of sliced objects for this sliced object list
    int sliced_objects_;

    // Number of layers sent to the front end so far
    // Used for incrementing the current layer in one at a time mode
    int current_layer_count_;
    int current_layer_offset_;

    // Ids of the sliced objects
    std::vector<int64_t> object_ids_;

    std::string temp_gcode_file_;
    std::ostringstream gcode_output_stream_;

    // Print object that olds one or more meshes that need to be sliced.
    std::vector< std::shared_ptr<MeshGroup> > objects_to_slice_;
};

}//namespace cura

#endif

#endif//COMMAND_SOCKET_ARCUS_H
