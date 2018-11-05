//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WIREFRAME2GCODE_H
#define WIREFRAME2GCODE_H

#include <functional> // passing function pointer or lambda as argument to a function

#include "RetractionConfig.h"
#include "settings/types/AngleRadians.h" //For the nozzle expansion angle setting.
#include "utils/NoCopy.h"

namespace cura
{

class Weaver;

/*!
 * Export class for exporting wireframe print gcode / weaver gcode / wireprint gcode.
 */
class Wireframe2gcode : public NoCopy
{
private:
    static const int STRATEGY_COMPENSATE = 0;
    static const int STRATEGY_KNOT = 1;
    static const int STRATEGY_RETRACT = 2;
    
    coord_t initial_layer_thickness;
    coord_t filament_diameter;
    coord_t line_width;
    Ratio flowConnection;
    Ratio flowFlat;
    double extrusion_mm3_per_mm_connection;
    double extrusion_mm3_per_mm_flat;
    bool update_extrusion_offset;
    coord_t nozzle_outer_diameter;
    coord_t nozzle_head_distance;
    AngleRadians nozzle_expansion_angle;
    coord_t nozzle_clearance;
    coord_t nozzle_top_diameter;
    Velocity moveSpeed;
    Velocity speedBottom;
    Velocity speedUp;
    Velocity speedDown;
    Velocity speedFlat;
    coord_t connectionHeight;
    coord_t roof_inset;
    Duration flat_delay;
    Duration bottom_delay;
    Duration top_delay;
    coord_t up_dist_half_speed;
    coord_t top_jump_dist;
    coord_t fall_down;
    coord_t drag_along;
    int strategy;
    bool go_back_to_last_top;
    Ratio straight_first_when_going_down;
    coord_t roof_fall_down;
    coord_t roof_drag_along;
    Duration roof_outer_delay;
    RetractionConfig standard_retraction_config; //!< The standard retraction settings used for moves between parts etc.

public:
    GCodeExport& gcode; //!< Where the result is 'stored'
    
    Wireframe2gcode(Weaver& weaver, GCodeExport& gcode);
    
    void writeGCode();


private:
    WireFrame& wireFrame;
    
    /*!
     * Startup gcode: nozzle temp up, retraction settings, bed temp
     */
    void processStartingCode();
    
    /*!
     * Lay down a skirt
     */
    void processSkirt();
    
    /*!
     * End gcode: nozzle temp down
     */
    void finalize();
    
    void writeFill(std::vector<WeaveRoofPart>& infill_insets, Polygons& outlines
        , std::function<void (Wireframe2gcode&, WeaveConnectionPart& part, unsigned int segment_idx)> connectionHandler
        , std::function<void (Wireframe2gcode&, WeaveConnectionSegment& p)> flatHandler);
    
    /*!
     * Function for writing the gcode for a diagonally down movement of a connection.
     * 
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void go_down(WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of an upward move of a connection, which does a couple of small moves at the top.
     * 
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void strategy_knot(WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of an upward move of a connection, which does a retract at the top.
     * 
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void strategy_retract(WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of an upward move of a connection, which goes Wireframe2gcode::fall_down further up 
     * and Wireframe2gcode::drag_along back from the direction it will go to next.
     * 
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void strategy_compensate(WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function writing the gcode of a segment in the connection between two layers.
     * 
     * \param part The part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void handle_segment(WeaveConnectionPart& part, unsigned int segment_idx);
    
    /*!
     * Function for writing the gcode of a segment in the connection between two roof insets / floor outsets.
     * 
     * \param part the part in which the segment is
     * \param segment_idx The index of the segment in the \p part
     */
    void handle_roof_segment(WeaveConnectionPart& part, unsigned int segment_idx);
    
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
