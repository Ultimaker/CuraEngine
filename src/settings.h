#ifndef SETTINGS_H
#define SETTINGS_H

#include <vector>
#include <map>
#include <unordered_map>

#include "utils/floatpoint.h"

#ifndef VERSION
#define VERSION "DEV"
#endif

#ifndef PATH_DATA_DEFAULT
/* #define PATH_DATA_DEFAULT "/usr/share/cura-engine/" */
#define PATH_DATA_DEFAULT ""
#endif

/*!
 * Different flavors of GCode. Some machines require different types of GCode.
 * The GCode flavor definition handles this as a big setting to make major or minor modifications to the GCode.
 */
enum EGCodeFlavor
{
/**
 * RepRap flavored GCode is Marlin/Sprinter/Repetier based GCode.
 *  This is the most commonly used GCode set.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm of filament extrusion.
 *  Retraction is done on E values with G1. Start/end code is added.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
    GCODE_FLAVOR_REPRAP = 0,
/**
 * UltiGCode flavored is Marlin based GCode.
 *  UltiGCode uses less settings on the slicer and puts more settings in the firmware. This makes for more hardware/material independed GCode.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm^3 of filament extrusion. Ignores the filament diameter setting.
 *  Retraction is done with G10 and G11. Retraction settings are ignored. G10 S1 is used for multi-extruder switch retraction.
 *  Start/end code is not added.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
    GCODE_FLAVOR_ULTIGCODE = 1,
/**
 * Makerbot flavored GCode.
 *  Looks a lot like RepRap GCode with a few changes. Requires MakerWare to convert to X3G files.
 *   Heating needs to be done with M104 Sxxx T0
 *   No G21 or G90
 *   Fan ON is M126 T0 (No fan strength control?)
 *   Fan OFF is M127 T0
 *   Homing is done with G162 X Y F2000
 **/
    GCODE_FLAVOR_MAKERBOT = 2,

/**
 * Bits From Bytes GCode.
 *  BFB machines use RPM instead of E. Which is coupled to the F instead of independed. (M108 S[deciRPM])
 *  Need X,Y,Z,F on every line.
 *  Needs extruder ON/OFF (M101, M103), has auto-retrection (M227 S[2560*mm] P[2560*mm])
 **/
    GCODE_FLAVOR_BFB = 3,

/**
 * MACH3 GCode
 *  MACH3 is CNC control software, which expects A/B/C/D for extruders, instead of E.
 **/
    GCODE_FLAVOR_MACH3 = 4,
/**
 * RepRap volumatric flavored GCode is Marlin based GCode.
 *  Volumatric uses less settings on the slicer and puts more settings in the firmware. This makes for more hardware/material independed GCode.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm^3 of filament extrusion. Ignores the filament diameter setting.
 *  Retraction is done with G10 and G11. Retraction settings are ignored. G10 S1 is used for multi-extruder switch retraction.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
    GCODE_FLAVOR_REPRAP_VOLUMATRIC = 5,
};

/*!
 * In Cura different infill methods are available.
 * This enum defines which fill patterns are available to get a uniform naming troughout the engine.
 * The different methods are used for top/bottom, support and sparse infill.
 */
enum EFillMethod
{
    Fill_Lines,
    Fill_Grid,
    Fill_Triangles,
    Fill_Concentric,
    Fill_ZigZag,
    Fill_None
};

/*!
 * Type of platform adheasion
 */
enum EPlatformAdhesion
{
    Adhesion_None,
    Adhesion_Brim,
    Adhesion_Raft
};

/*!
 * Type of support material to generate
 */
enum ESupportType
{
    Support_None,
    Support_PlatformOnly,
    Support_Everywhere
};

#define MAX_EXTRUDERS 16

//Maximum number of sparse layers that can be combined into a single sparse extrusion.
#define MAX_SPARSE_COMBINE 8

/*!
 * Base class for every object that can hold settings.
 * The SettingBase object can hold multiple key-value pairs that define settings.
 * The settings that are set on a SettingBase are checked against the SettingRegistry to ensure keys are valid.
 * Different conversion functions are available for settings to increase code clarity and in the future make
 * unit conversions possible.
 */
class SettingsBase
{
private:
    std::unordered_map<std::string, std::string> setting_values;
    SettingsBase* parent;
public:
    SettingsBase();
    SettingsBase(SettingsBase* parent);

    bool hasSetting(std::string key);

    void setSetting(std::string key, std::string value);
    
    std::string getSettingString(std::string key);
    int getSettingAsIndex(std::string key);
    int getSettingAsCount(std::string key);
    
    double getSettingInAngleRadians(std::string key);
    int getSettingInMicrons(std::string key);
    bool getSettingBoolean(std::string key);
    double getSettingInDegreeCelsius(std::string key);
    double getSettingInMillimetersPerSecond(std::string key);
    double getSettingInCubicMillimeters(std::string key);
    double getSettingInPercentage(std::string key);
    double getSettingInSeconds(std::string key);
    
    EGCodeFlavor getSettingAsGCodeFlavor(std::string key);
    EFillMethod getSettingAsFillMethod(std::string key);
    EPlatformAdhesion getSettingAsPlatformAdhesion(std::string key);
    ESupportType getSettingAsSupportType(std::string key);
};


#endif//SETTINGS_H
