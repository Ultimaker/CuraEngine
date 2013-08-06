#ifndef SETTINGS_H
#define SETTINGS_H

#include <utils/floatpoint.h>
#include <vector>

class _ConfigSettingIndex
{
public:
    const char* key;
    int* ptr;
    
    _ConfigSettingIndex(const char* key, int* ptr) : key(key), ptr(ptr) {}
};

#define FIX_HORRIBLE_UNION_ALL_TYPE_A    0x01
#define FIX_HORRIBLE_UNION_ALL_TYPE_B    0x02
#define FIX_HORRIBLE_EXTENSIVE_STITCHING 0x04
#define FIX_HORRIBLE_KEEP_NONE_CLOSED    0x10

/**
 * RepRap flavored GCode is Marlin/Sprinter/Repetier based GCode. 
 *  This is the most commonly used GCode set.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm of filament extrusion.
 *  Retraction is done on E values with G1. Start/end code is added.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
#define GCODE_FLAVOR_REPRAP              0
/**
 * UltiGCode flavored is Marlin based GCode. 
 *  UltiGCode uses less settings on the slicer and puts more settings in the firmware. This makes for more hardware/material independed GCode.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm^3 of filament extrusion. Ignores the filament diameter setting.
 *  Retraction is done with G10 and G11. Retraction settings are ignored. G10 S1 is used for multi-extruder switch retraction.
 *  Start/end code is not added.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
#define GCODE_FLAVOR_ULTIGCODE           1

class ConfigSettings
{
private:
    std::vector<_ConfigSettingIndex> _index;
public:
    int layerThickness;
    int initialLayerThickness;
    int filamentDiameter;
    int filamentFlow;
    int extrusionWidth;
    int insetCount;
    int downSkinCount;
    int upSkinCount;
    int sparseInfillLineDistance;
    int infillOverlap;
    int skirtDistance;
    int skirtLineCount;
    int retractionAmount;
    int retractionAmountExtruderSwitch;
    int retractionSpeed;
    int retractionMinimalDistance;
    int enableCombing;
    int multiVolumeOverlap;
    
    int initialSpeedupLayers;
    int initialLayerSpeed;
    int printSpeed;
    int infillSpeed;
    int moveSpeed;
    int fanOnLayerNr;
    
    //Support material
    int supportAngle;
    int supportEverywhere;
    int supportLineWidth;

    //Cool settings
    int minimalLayerTime;
    int minimalFeedrate;
    int coolHeadLift;
    int fanSpeedMin;
    int fanSpeedMax;
    
    //Raft settings
    int raftMargin;
    int raftLineSpacing;
    int raftBaseThickness;
    int raftBaseLinewidth;
    int raftInterfaceThickness;
    int raftInterfaceLinewidth;
    
    FMatrix3x3 matrix;
    IntPoint objectPosition;
    int objectSink;
    
    int fixHorrible;
    int gcodeFlavor;
    
    IntPoint extruderOffset[16];
    const char* startCode;
    const char* endCode;
    
    ConfigSettings();
    bool setSetting(const char* key, const char* value);
};

#endif//SETTINGS_H
