#ifndef WIREFRAME2GCODE_H
#define WIREFRAME2GCODE_H


#include <functional> // passing function pointer or lambda as argument to a function

#include "weaveDataStorage.h"
#include "commandSocket.h"
#include "settings.h"

#include "modelFile/modelFile.h" // PrintObject
#include "slicer.h"

#include "utils/polygon.h"
#include "Weaver.h"

#include "debug.h"

namespace cura
{

/*!
 * Export class for exporting wireframe print gcode / weaver gcode / wireprint gcode.
 */
class Wireframe2gcode : public SettingsBase
{
private:
    static const int STRATEGY_COMPENSATE = 0;
    static const int STRATEGY_KNOT = 1;
    static const int STRATEGY_RETRACT = 2;
    
    int initial_layer_thickness;
    int filament_diameter;
    int extrusionWidth;
    double flowConnection;// = getSettingInt("wireframeFlowConnection");
    double flowFlat; // = getSettingInt("wireframeFlowFlat");
    double extrusion_per_mm_connection; // = lineArea / filament_area * double(flowConnection) / 100.0;
    double extrusion_per_mm_flat; // = lineArea / filament_area * double(flowFlat) / 100.0;
    int nozzle_outer_diameter; // = getSettingInt("machineNozzleTipOuterDiameter");    // ___       ___   .
    int nozzle_head_distance; // = getSettingInt("machineNozzleHeadDistance");         //    |     |      .
    double nozzle_expansion_angle; // = getSettingInt("machineNozzleExpansionAngle");  //     \_U_/       .
    int nozzle_clearance; // = getSettingInt("wireframeNozzleClearance");    // at least line width
    int nozzle_top_diameter; // = tan(static_cast<double>(nozzle_expansion_angle)/180.0 * M_PI) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
    double moveSpeed; // = 40;
    double speedBottom; // =  getSettingInt("wireframePrintspeedBottom");
    double speedUp; // = getSettingInt("wireframePrintspeedUp");
    double speedDown; // = getSettingInt("wireframePrintspeedDown");
    double speedFlat; // = getSettingInt("wireframePrintspeedFlat");
    int connectionHeight; // = getSettingInt("wireframeConnectionHeight"); 
    int roof_inset; // = getSettingInt("wireframeRoofInset"); 
    double flat_delay; // = getSettingInt("wireframeFlatDelay")/100.0;
    double bottom_delay; // = getSettingInt("wireframeBottomDelay")/100.0;
    double top_delay; // = getSettingInt("wireframeTopDelay")/100.0;
    int up_dist_half_speed; // = getSettingInt("wireframeUpDistHalfSpeed");
    int top_jump_dist; // = getSettingInt("wireframeTopJump");
    int fall_down; // = getSettingInt("wireframeFallDown");
    int drag_along; // = getSettingInt("wireframeDragAlong");
    int strategy; // = getSettingInt("wireframeStrategy"); //  HIGHER_BEND_NO_STRAIGHTEN; // RETRACT_TO_STRAIGHTEN; // MOVE_TO_STRAIGHTEN; // 
    double go_back_to_last_top; // = false;
    int straight_first_when_going_down; // = getSettingInt("wireframeStraightBeforeDown"); // %
    int roof_fall_down; // = getSettingInt("wireframeRoofFallDown");
    int roof_drag_along; // = getSettingInt("wireframeRoofDragAlong");
    double roof_outer_delay; // = getSettingInt("wireframeRoofOuterDelay")/100.0;
    
    RetractionConfig standard_retraction_config; //!< The standard retraction settings used for moves between parts etc.
    
public:
    GCodeExport& gcode; //!< Where the result is 'stored'
    
    Wireframe2gcode(Weaver& weaver, GCodeExport& gcode, SettingsBase* settings_base);
    
    void writeGCode(CommandSocket* commandSocket);


private:
    WireFrame wireFrame;
    
    void writeFill(std::vector<WeaveRoofPart>& fill_insets, Polygons& outlines
        , std::function<void (Wireframe2gcode& thiss, WeaveRoofPart& inset, WeaveConnectionPart& part, unsigned int segment_idx)> connectionHandler
        , std::function<void (Wireframe2gcode& thiss, WeaveConnectionSegment& p)> flatHandler);
    
    /*!
     * Function for writing the gcode for a diagonally down movement of a connection.
     * 
     * \param layer The layer in which the segment is
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void go_down(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of an upward move of a connection, which does a couple of small moves at the top.
     * 
     * \param layer The layer in which the segment is
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void strategy_knot(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of an upward move of a connection, which does a retract at the top.
     * 
     * \param layer The layer in which the segment is
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void strategy_retract(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of an upward move of a connection, which goes Wireframe2gcode::fall_down further up 
     * and Wireframe2gcode::drag_along back from the direction it will go to next.
     * 
     * \param layer The layer in which the segment is
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void strategy_compensate(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function writing the gcode of a segment in the connection between two layers.
     * 
     * \param layer The layer in which the segment is
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void handle_segment(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of a segment in the connection between two roof insets / floor outsets.
     * 
     * \param inset The inset in which the segment is
     * \param part the part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void handle_roof_segment(WeaveRoofPart& inset, WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Write a move action to gcode, inserting a retraction if neccesary.
     * 
     * \param to The 3D destination of the move
     */
    void writeMoveWithRetract(Point3 to);
    
    /*!
     * Write a move action to gcode, inserting a retraction if neccesary.
     * 
     * \param to The 2D destination of the move
     */
    void writeMoveWithRetract(Point to);

};

}//namespace cura

#endif//WIREFRAME2GCODE_H
