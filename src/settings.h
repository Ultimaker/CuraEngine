#ifndef SETTINGS_H
#define SETTINGS_H

#include <vector>
#include <map>

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
enum Support_Pattern
{
    SUPPORT_TYPE_GRID = 0,
    SUPPORT_TYPE_LINES = 1,
    SUPPORT_TYPE_ZIGZAG = 2
};

#ifndef DEFAULT_CONFIG_PATH
#define DEFAULT_CONFIG_PATH "default.cfg"
#endif

#define CONFIG_MULTILINE_SEPARATOR "\"\"\""

enum GCode_Flavor
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

#define MAX_EXTRUDERS 16

//Maximum number of sparse layers that can be combined into a single sparse extrusion.
#define MAX_SPARSE_COMBINE 8

/**
 * Type of skin pattern.
 */
enum Skin_Pattern
{
    SKIN_LINES = 0,
    SKIN_CONCENTRIC = 1,
};

/**
 * Type of infill pattern.
 */
enum Infill_Pattern
{
    INFILL_GRID = 0,
    INFILL_LINES = 1,
    INFILL_CONCENTRIC = 2,
    INFILL_ZIGZAG = 3,
};

class SettingRegistry
{
private:
    std::set<std::string> knownSettings;
    void registerSetting(std::string setting);
public:
    SettingRegistry();
    bool settingExists(std::string setting) const;
};

class SettingsBase
{
private:
    static const SettingRegistry settingRegistry;
    std::map<std::string, std::string> settings;
    SettingsBase* parent;
public:
    SettingsBase();
    SettingsBase(SettingsBase* parent);

    void copySettings(SettingsBase& other);
    
    bool hasSetting(std::string key);

    void setSetting(std::string key, std::string value);
    int getSettingInt(std::string key);
    std::string getSetting(std::string key);
    
    void setDefaultSettings();
};


#endif//SETTINGS_H
