#ifndef COMMAND_SOCKET_H
#define COMMAND_SOCKET_H

#include "utils/socket.h"
#include "utils/polygon.h"
#include "settings.h"
#include "Progress.h"
#include "PrintFeature.h"

#include <memory>

#include "Cura.pb.h"

namespace cura {


class CommandSocket
{
private:
    static CommandSocket* instance; //!< May be a nullptr in case it hasn't been instantiated.

    CommandSocket(); //!< The single constructor is known only privately, since this class is similar to a singleton class (except the single object doesn't need to be instantiated)

public:
    static CommandSocket* getInstance(); //!< Get the CommandSocket instance, or nullptr if it hasn't been instantiated.

    static void instantiate(); //!< Instantiate the CommandSocket.

    static bool isInstantiated(); //!< Check whether the singleton is instantiated

    /*!
     * Connect with the GUI
     * This creates and initialises the arcus socket and then continues listening for messages. 
     * \param ip string containing the ip to connect with
     * \param port int of the port to connect with.
     */
    void connect(const std::string& ip, int port);
    
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
     * Send info on a layer to be displayed by the forntend: set the z and the thickness of the layer.
     */
    void sendLayerInfo(int layer_nr, int32_t z, int32_t height);
    
    /*! 
     * Send a polygon to the engine. This is used for the layerview in the GUI
     */
    void sendPolygons(cura::PrintFeatureType type, int layer_nr, cura::Polygons& polygons, int line_width);

    /*! 
     * Send a polygon to the engine if the command socket is instantiated. This is used for the layerview in the GUI
     */
    static void sendPolygonsToCommandSocket(cura::PrintFeatureType type, int layer_nr, cura::Polygons& polygons, int line_width);

    /*! 
     * Send progress to GUI
     */
    void sendProgress(float amount);
    
    /*!
     * Send the current stage of the process to the GUI (starting, slicing infill, etc) 
     */
    void sendProgressStage(Progress::Stage stage);
    
    /*!
     * Send time estimate of how long print would take.
     */
    void sendPrintTime();
    
    /*!
     * Notify that the slicing has finished.
     */
    void sendSlicingFinished();
    
    /*!
     * Does nothing at the moment
     */
    void sendPrintMaterialForObject(int index, int extruder_nr, float material_amount);

    /*!
     * Start the slicing of a new meshgroup
     */
    void beginSendSlicedObject();
    
    /*!
     * Conclude the slicing of the current meshgroup, so that we can start the next
     */
    void endSendSlicedObject();

    void beginGCode();
    
    /*!
     * Flush the gcode in gcode_output_stream into a message queued in the socket.
     */
    void flushGcode();
    void sendGCodePrefix(std::string prefix);

private:
    class Private;
    const std::unique_ptr<Private> private_data;
};

}//namespace cura

#endif//COMMAND_SOCKET_H
