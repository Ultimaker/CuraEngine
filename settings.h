#ifndef SETTINGS_H
#define SETTINGS_H

#include <vector>

#include "utils/floatpoint.h"

#ifndef VERSION
#define VERSION "DEV"
#endif

#define FIX_HORRIBLE_UNION_ALL_TYPE_A    0x01
#define FIX_HORRIBLE_UNION_ALL_TYPE_B    0x02
#define FIX_HORRIBLE_EXTENSIVE_STITCHING 0x04
#define FIX_HORRIBLE_UNION_ALL_TYPE_C    0x08
#define FIX_HORRIBLE_KEEP_NONE_CLOSED    0x10

/**
 * Type of support material.
 * Grid is a X/Y grid with an outline, which is very strong, provides good support. But in some cases is hard to remove.
 * Lines give a row of lines which break off one at a time, making them easier to remove, but they do not support as good as the grid support.
 */
#define SUPPORT_TYPE_GRID                0
#define SUPPORT_TYPE_LINES               1

#ifndef DEFAULT_CONFIG_PATH
#define DEFAULT_CONFIG_PATH "default.cfg"
#endif

#define CONFIG_MULTILINE_SEPARATOR "\"\"\""

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
/**
 * Makerbot flavored GCode.
 *  Looks a lot like RepRap GCode with a few changes. Requires MakerWare to convert to X3G files.
 *   Heating needs to be done with M104 Sxxx T0
 *   No G21 or G90
 *   Fan ON is M126 T0 (No fan strength control?)
 *   Fan OFF is M127 T0
 *   Homing is done with G162 X Y F2000
 **/
#define GCODE_FLAVOR_MAKERBOT           2

#define MAX_EXTRUDERS 16

class _ConfigSettingIndex
{
public:
    const char* key;
    int* ptr;

    _ConfigSettingIndex(const char* key, int* ptr) : key(key), ptr(ptr) {}
};

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
    int skirtMinLength;

    //Retraction settings
    int retractionAmount;
    int retractionAmountExtruderSwitch;
    int retractionSpeed;
    int retractionMinimalDistance;
    int minimalExtrusionBeforeRetraction;
    int retractionZHop;

    int enableCombing;
    int enableOozeShield;
    int wipeTowerSize;
    int multiVolumeOverlap;

    int initialSpeedupLayers;
    int initialLayerSpeed;
    int printSpeed;
    int infillSpeed;
    int inset0Speed;
    int insetXSpeed;
    int moveSpeed;
    int fanFullOnLayerNr;

    //Support material
    int supportType;
    int supportAngle;
    int supportEverywhere;
    int supportLineDistance;
    int supportXYDistance;
    int supportZDistance;
    int supportExtruder;

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
    int spiralizeMode;
    int gcodeFlavor;

    IntPoint extruderOffset[MAX_EXTRUDERS];
    std::string startCode;
    std::string endCode;

    ConfigSettings();
    bool setSetting(const char* key, const char* value);
    bool readSettings(void);
    bool readSettings(const char* path);
};

#endif//SETTINGS_H
