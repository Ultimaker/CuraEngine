#ifndef COMMAND_SOCKET_H
#define COMMAND_SOCKET_H

#include "utils/socket.h"
#include "utils/polygon.h"
#include "settings.h"
#include "Progress.h"
#include "PrintFeature.h"

namespace cura
{

class CommandSocket
{
private:
    static std::unique_ptr<CommandSocket> instance_; //!< always instantiated

public:
    static CommandSocket* getInstance();
    static void setInstance(std::unique_ptr<CommandSocket> instance);

protected:
    CommandSocket() = default;

public:
    virtual ~CommandSocket() = default;
    /*!
     * Send info on a layer to be displayed by the forntend: set the z and the thickness of the layer.
     */
    virtual void sendLayerInfo(int layer_nr, int32_t z, int32_t height) {}
    
    /*! 
     * Send a polygon to the engine. This is used for the layerview in the GUI
     */
    virtual void sendPolygons(cura::PrintFeatureType type, int layer_nr, cura::Polygons& polygons, int line_width) {}

    /*! 
     * Send progress to GUI
     */
    virtual void sendProgress(float amount) {}
    
    /*!
     * Send the current stage of the process to the GUI (starting, slicing infill, etc) 
     */
    virtual void sendProgressStage(Progress::Stage stage) {}
    
    /*!
     * Send time estimate of how long print would take.
     */
    virtual void sendPrintTime() {}
    
    /*!
     * Does nothing at the moment
     */
    virtual void sendPrintMaterialForObject(int index, int extruder_nr, float material_amount) {}

    /*!
     * Start the slicing of a new meshgroup
     */
    virtual void beginSendSlicedObject() {}
    
    /*!
     * Conclude the slicing of the current meshgroup, so that we can start the next
     */
    virtual void endSendSlicedObject() {}

    /*!
     * \brief Sends a message to indicate that all the slicing is done.
     *
     * This should indicate that no more data (g-code, prefix/postfix, metadata
     * or otherwise) should be sent any more regarding the latest slice job.
     */
    virtual void sendFinishedSlicing() {}

    virtual void beginGCode() {}
    
    /*!
     * Flush the gcode in gcode_output_stream into a message queued in the socket.
     */
    virtual void flushGcode() {}
    virtual void sendGCodePrefix(std::string prefix) {}
};

}//namespace cura

#endif//COMMAND_SOCKET_H
