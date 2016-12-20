#ifndef WIREFRAME2GCODE_H
#define WIREFRAME2GCODE_H


#include <functional> // passing function pointer or lambda as argument to a function

#include "utils/NoCopy.h"

#include "weaveDataStorage.h"
#include "commandSocket.h"
#include "settings/settings.h"

#include "MeshGroup.h"
#include "slicer/Slicer.h"

#include "utils/polygon.h"
#include "Weaver.h"

namespace cura
{

/*!
 * Export class for exporting wireframe print gcode / weaver gcode / wireprint gcode.
 */
class Wireframe2gcode : public SettingsMessenger, NoCopy
{
private:
    static const int STRATEGY_COMPENSATE = 0;
    static const int STRATEGY_KNOT = 1;
    static const int STRATEGY_RETRACT = 2;
    
    coord_t initial_layer_thickness;
    coord_t filament_diameter;
    coord_t line_width;
    double flowConnection;
    double flowFlat; 
    double extrusion_mm3_per_mm_connection;
    double extrusion_mm3_per_mm_flat;
    coord_t nozzle_outer_diameter;
    coord_t nozzle_head_distance;
    double nozzle_expansion_angle;
    coord_t nozzle_clearance;
    coord_t nozzle_top_diameter;
    double moveSpeed;
    double speedBottom;
    double speedUp;
    double speedDown;
    double speedFlat;
    coord_t connectionHeight;
    coord_t roof_inset;
    double flat_delay;
    double bottom_delay;
    double top_delay;
    coord_t up_dist_half_speed;
    coord_t top_jump_dist;
    coord_t fall_down;
    coord_t drag_along;
    int strategy;
    double go_back_to_last_top;
    coord_t straight_first_when_going_down;
    coord_t roof_fall_down;
    coord_t roof_drag_along;
    double roof_outer_delay;
    
    RetractionConfig standard_retraction_config; //!< The standard retraction settings used for moves between parts etc.
    
public:
    GCodeExport& gcode; //!< Where the result is 'stored'
    
    Wireframe2gcode(Weaver& weaver, GCodeExport& gcode, SettingsBase* settings_base);
    
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
