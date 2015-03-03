#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include "utils/logoutput.h"

#include "settings.h"

SettingsBase::SettingsBase()
: parent(NULL)
{
}
SettingsBase::SettingsBase(SettingsBase* parent)
: parent(parent)
{
}

void SettingsBase::setSetting(std::string key, std::string value)
{
    if (SettingsBase::settingRegistry.settingExists(key))
        settings[key] = value;
    else
        cura::logError("Ignoring unknown setting %s\n", key.c_str() );
}

int SettingsBase::getSettingInt(std::string key)
{
    std::string value = getSetting(key);
    return atoi(value.c_str());
}

std::string SettingsBase::getSetting(std::string key)
{
    if (settings.find(key) != settings.end())
    {
        return settings[key];
    }
    if (parent)
        return parent->getSetting(key);
    
    if (SettingsBase::settingRegistry.settingExists(key))
        cura::logError("Failed to find setting %s\n", key.c_str());
    else 
        cura::logError("Unknown setting %s\n", key.c_str());
    settings[key] = "";
    return "";
}

void SettingsBase::copySettings(SettingsBase& other)
{
    settings = other.settings;
}

bool SettingsBase::hasSetting(std::string key)
{
    if (settings.find(key) != settings.end())
    {
        return true;
    }
    if (parent)
        return parent->hasSetting(key);
    
    return false;
}

const SettingRegistry SettingsBase::settingRegistry; // define settingRegistry

bool SettingRegistry::settingExists(std::string setting) const
{
    return knownSettings.find(setting) != knownSettings.end();
}

void SettingRegistry::registerSetting(std::string setting)
{
    knownSettings.insert(setting);
}

SettingRegistry::SettingRegistry()
{
    registerSetting("simpleMode");
    registerSetting("spiralizeMode");
    registerSetting("enableOozeShield");
    registerSetting("autoCenter");
    
    registerSetting("initialSpeedupLayers");
    registerSetting("minimalFeedrate");
    registerSetting("preSwitchExtruderCode");
    registerSetting("insetXSpeed");
    registerSetting("retractionZHop");
    registerSetting("extruderOffset[3].X");
    registerSetting("extruderOffset[3].Y");
    registerSetting("gcodeFlavor");
    registerSetting("postSwitchExtruderCode");
    registerSetting("retractionSpeed");
    registerSetting("filamentFlow");
    registerSetting("infillOverlap");
    registerSetting("inset0Speed");
    registerSetting("coolHeadLift");
    registerSetting("extrusionWidth");
    registerSetting("upSkinCount");
    registerSetting("initialLayerSpeed");
    registerSetting("minimalLayerTime");
    registerSetting("infillSpeed");
    registerSetting("fanSpeedMax");
    registerSetting("enableCombing");
    registerSetting("fanSpeedMin");
    
    registerSetting("raftAirGapLayer0");
    registerSetting("raftBaseThickness");
    registerSetting("raftBaseLinewidth");
    registerSetting("raftBaseSpeed");
    registerSetting("raftInterfaceThickness");
    registerSetting("raftInterfaceLinewidth");
    registerSetting("raftInterfaceLineSpacing");
    registerSetting("raftInterfaceSpeed");
    registerSetting("raftLineSpacing");
    registerSetting("raftFanSpeed");
    registerSetting("raftSurfaceLinewidth");
    registerSetting("raftSurfaceLineSpacing");
    registerSetting("raftSurfaceSpeed");
    registerSetting("raftSurfaceLayers");
    registerSetting("raftSurfaceThickness");
    registerSetting("raftMargin");
    registerSetting("raftAirGap");
    
    registerSetting("supportXYDistance");
    registerSetting("supportExtruder");
    registerSetting("supportType");
    registerSetting("supportZDistance");
    registerSetting("supportEverywhere");
    registerSetting("supportAngle");
    registerSetting("supportZDistanceBottom");
    registerSetting("supportZDistanceTop");
    registerSetting("supportJoinDistance");
    registerSetting("supportBridgeBack");
    registerSetting("supportSpeed");
    registerSetting("supportSkipLayers");
    registerSetting("areaSupportPolyGenerator");
    
    registerSetting("filamentDiameter");
    registerSetting("fanFullOnLayerNr");
    registerSetting("extruderOffset[1].X");
    registerSetting("extruderOffset[1].Y");
    registerSetting("layerThickness");
    registerSetting("initialLayerThickness");
    registerSetting("endCode");
    registerSetting("skirtLineCount");
    registerSetting("supportBottomStairDistance");
    registerSetting("minimalExtrusionBeforeRetraction");
    registerSetting("retractionMinimalDistance");
    registerSetting("skirtMinLength");
    registerSetting("objectSink");
    registerSetting("retractionAmount");
    registerSetting("skinSpeed");
    registerSetting("skirtLineCount");
    registerSetting("startCode");
    registerSetting("skirtDistance");
    registerSetting("extruderOffset[2].Y");
    registerSetting("extruderOffset[2].X");
    registerSetting("printSpeed");
    registerSetting("fixHorrible");
    registerSetting("layer0extrusionWidth");
    registerSetting("moveSpeed");
    registerSetting("supportLineDistance");
    registerSetting("retractionAmountExtruderSwitch");
    registerSetting("sparseInfillLineDistance");
    registerSetting("insetCount");
    registerSetting("downSkinCount");
    registerSetting("multiVolumeOverlap");
    registerSetting("position.X");
    registerSetting("position.Y");
    registerSetting("position.Z");
    registerSetting("retractionPrimeAmount");
    registerSetting("retractionPrimeSpeed");
    registerSetting("skirtSpeed");
    registerSetting("extruderNr");
    registerSetting("sparseInfillCombineCount");
    registerSetting("retractionExtruderSwitchPrimeSpeed");
    registerSetting("retractionExtruderSwitchSpeed");
    registerSetting("infillPattern");
    registerSetting("skinPattern");
    registerSetting("wipeTowerSize");
    
    
    for(int n=0; n<MAX_EXTRUDERS; n++)
    {
        std::ostringstream stream;
        stream << "extruderOffset" << n;
        registerSetting(stream.str() + ".X");
        registerSetting(stream.str() + ".Y");
    }
}


void SettingsBase::setDefaultSettings()
{
//     setSetting("simpleMode", "0");
//     setSetting("spiralizeMode", "0");
//     setSetting("enableOozeShield", "0");
//     setSetting("wipeTowerSize", "0");
//     setSetting("extruderNr", "0");
//     
//     
    
//     setSetting("", "");
//     setSetting("", "");
//     setSetting("", "");
/*
    setSetting("layerThickness", "150");
    setSetting("initialLayerThickness", "300");
    setSetting("filamentDiameter", "2890");
    setSetting("filamentFlow", "100");
    setSetting("layer0extrusionWidth", "600");
    setSetting("extrusionWidth", "400");
    setSetting("insetCount", "2");
    setSetting("downSkinCount", "6");
    setSetting("upSkinCount", "6");
    setSetting("skinPattern", "LINES");
    setSetting("skirtDistance", "6000");
    setSetting("skirtLineCount", "1");
    setSetting("skirtMinLength", "0");

    setSetting("initialSpeedupLayers", "4");
    setSetting("initialLayerSpeed", "20");
    setSetting("skirtSpeed", "50");
    setSetting("inset0Speed", "50");
    setSetting("insetXSpeed", "50");
    setSetting("skinSpeed", "50");
    setSetting("supportSpeed", "50");
    setSetting("moveSpeed", "150");
    setSetting("fanFullOnLayerNr", "2");

    setSetting("sparseInfillLineDistance", "500");
    setSetting("sparseInfillCombineCount", "1");
    setSetting("infillOverlap", "15");
    setSetting("infillSpeed", "50");
    setSetting("infillPattern", "INFILL_GRID");

    setSetting("supportType", "LINES");
    setSetting("supportAngle", "60");
    //setSetting("supportAngle", "-1");
    setSetting("supportEverywhere", "0");
    setSetting("supportLineDistance", "2667");
    setSetting("supportXYDistance", "500");
    setSetting("supportZDistance", "100");
    setSetting("supportExtruder", "-1");
    
    setSetting("areaSupportPolyGenerator", "1"); // TK
    setSetting("supportZDistanceBottom", "100");
    setSetting("supportZDistanceTop", "100");
    setSetting("supportJoinDistance", "1000"); // distance between support blocks which will get merged 
    setSetting("supportBridgeBack", "100"); // percentage of bridge between layer and overhang which should be supported
    setSetting("supportBottomStairDistance", "100"); // height of the steps with which the lines support rests on the model
    
    
    
    setSetting("extruderOffset1.X", "18000");
    setSetting("extruderOffset1.Y", "0");
    
    setSetting("retractionAmount", "4500");
    setSetting("retractionPrimeAmount", "0");
    setSetting("retractionSpeed", "25");
    setSetting("retractionPrimeSpeed", "25");
    setSetting("retractionAmountExtruderSwitch", "14500");
    setSetting("retractionExtruderSwitchSpeed", "25");
    setSetting("retractionExtruderSwitchPrimeSpeed", "25");
    setSetting("retractionMinimalDistance", "1500");
    setSetting("minimalExtrusionBeforeRetraction", "100");
    setSetting("retractionZHop", "0");

    setSetting("enableCombing", "1");
    setSetting("enableOozeShield", "0");
    setSetting("wipeTowerSize", "0");
    setSetting("multiVolumeOverlap", "0");
    setSetting("position.X", "115000"); // UM2
    setSetting("position.Y", "112500");
    setSetting("position.Z", "0");
    setSetting("objectSink", "0");
    setSetting("autoCenter", "1");
    setSetting("extruderNr", "0");

    setSetting("raftMargin", "5000");
    setSetting("raftLineSpacing", "1000");
    setSetting("raftBaseThickness", "0");
    setSetting("raftBaseLinewidth", "0");
    setSetting("raftInterfaceThickness", "0");
    setSetting("raftInterfaceLinewidth", "0");
    setSetting("raftInterfaceLineSpacing", "0");
    setSetting("raftAirGap", "0");
    setSetting("raftAirGapLayer0", "0");
    setSetting("raftBaseSpeed", "15");
    setSetting("raftInterfaceSpeed", "15");
    setSetting("raftFanSpeed", "0");
    setSetting("raftSurfaceThickness", "0");
    setSetting("raftSurfaceLinewidth", "0");
    setSetting("raftSurfaceLineSpacing", "0");
    setSetting("raftSurfaceLayers", "0");
    setSetting("raftSurfaceSpeed", "0");

    setSetting("minimalLayerTime", "5");
    setSetting("minimalFeedrate", "10");
    setSetting("coolHeadLift", "0");
    setSetting("fanSpeedMin", "100");
    setSetting("fanSpeedMax", "100");

    setSetting("fixHorrible", "0");
    setSetting("spiralizeMode", "0");
    setSetting("simpleMode", "0");
    setSetting("gcodeFlavor", "GCODE_FLAVOR_REPRAP");

    for(int n=0; n<MAX_EXTRUDERS; n++)
    {
        std::ostringstream stream;
        stream << "extruderOffset" << n;
        setSetting(stream.str() + ".X", "0");
        setSetting(stream.str() + ".Y", "0");
    }

    setSetting("startCode",
        "M109 S210     ;Heatup to 210C\n"
        "G21           ;metric values\n"
        "G90           ;absolute positioning\n"
        "G28           ;Home\n"
        "G1 Z15.0 F300 ;move the platform down 15mm\n"
        "G92 E0        ;zero the extruded length\n"
        "G1 F200 E5    ;extrude 5mm of feed stock\n"
        "G92 E0        ;zero the extruded length again\n");
    setSetting("endCode",
        "M104 S0                     ;extruder heater off\n"
        "M140 S0                     ;heated bed heater off (if you have it)\n"
        "G91                            ;relative positioning\n"
        "G1 E-1 F300                    ;retract the filament a bit before lifting the nozzle, to release some of the pressure\n"
        "G1 Z+0.5 E-5 X-20 Y-20 F9000   ;move Z up a bit and retract filament even more\n"
        "G28 X0 Y0                      ;move X/Y to min endstops, so the head is out of the way\n"
        "M84                         ;steppers off\n"
        "G90                         ;absolute positioning\n");
    setSetting("postSwitchExtruderCode", "");
    setSetting("preSwitchExtruderCode", "");
    */
}
