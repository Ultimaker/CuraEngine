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
    // mode
    registerSetting("neith");
    registerSetting("simpleMode");
    registerSetting("spiralizeMode");
    registerSetting("enableOozeShield");
    registerSetting("autoCenter");
    registerSetting("fixHorrible");
    registerSetting("avoidOverlappingPerimeters");
    
    // machine settings
    registerSetting("bedTemperature"); // in 1/100th degrees
    registerSetting("printTemperature"); 
    registerSetting("filamentDiameter");
    
    registerSetting("gcodeFlavor");
    registerSetting("startCode");
    registerSetting("endCode");
    
    registerSetting("extrusionWidth");
    registerSetting("fanSpeedMax");
    registerSetting("fanSpeedMin");
    registerSetting("fanFullOnLayerNr");
    
    registerSetting("filamentFlow");
    registerSetting("minimalFeedrate");
    registerSetting("minimalLayerTime");
        
    for(int n=0; n<MAX_EXTRUDERS; n++)
    {
        std::ostringstream stream;
        stream << "extruderOffset" << n;
        registerSetting(stream.str() + ".X");
        registerSetting(stream.str() + ".Y");
    }
    
    // speeds
    registerSetting("initialSpeedupLayers");
    registerSetting("initialLayerSpeed");
    registerSetting("inset0Speed");
    registerSetting("insetXSpeed");
    registerSetting("infillSpeed");
    registerSetting("moveSpeed");
    registerSetting("skinSpeed");
    registerSetting("skirtSpeed");
    
    // uncategorized
    registerSetting("infillOverlap");
    registerSetting("coolHeadLift");
    registerSetting("upSkinCount");
    registerSetting("enableCombing");
    
    
    registerSetting("layerThickness");
    registerSetting("initialLayerThickness");
    registerSetting("layer0extrusionWidth");
    
    registerSetting("XYcompensation");
    

    // infill
    registerSetting("sparseInfillLineDistance");
    registerSetting("sparseInfillCombineCount");
    registerSetting("infillPattern");
    
    registerSetting("insetCount");
    registerSetting("downSkinCount");
    registerSetting("position.X");
    registerSetting("position.Y");
    registerSetting("position.Z");
    registerSetting("extruderNr");
    registerSetting("skinPattern");
    registerSetting("wipeTowerSize");
    registerSetting("wipeTowerDistance");

    // retraction
    registerSetting("retractionSpeed");
    registerSetting("minimalExtrusionBeforeRetraction");
    registerSetting("retractionMinimalDistance");
    registerSetting("retractionAmount");
    registerSetting("retractionPrimeAmount");
    registerSetting("retractionPrimeSpeed");
    registerSetting("retractionZHop");
    
    // dual extrusion
    registerSetting("multiVolumeOverlap");
    for(int n=0; n<MAX_EXTRUDERS; n++)
    {
        std::ostringstream stream;
        stream << n;
        registerSetting("preSwitchExtruderCode" + stream.str());
        registerSetting("postSwitchExtruderCode" + stream.str());
    }
    registerSetting("retractionExtruderSwitchPrimeSpeed");
    registerSetting("retractionExtruderSwitchSpeed");
    registerSetting("retractionAmountExtruderSwitch");
    
    // skirt / brim
    registerSetting("skirtLineCount");
    registerSetting("skirtMinLength");
    registerSetting("skirtLineCount");
    registerSetting("skirtDistance");
    
    // raft
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
    
    // support
    registerSetting("supportXYDistance");
    registerSetting("supportExtruder");
    registerSetting("supportType");
    registerSetting("supportZDistance");
    registerSetting("supportOnBuildplateOnly");
    registerSetting("supportAngle");
    registerSetting("supportZDistanceBottom");
    registerSetting("supportZDistanceTop");
    registerSetting("supportSpeed");
    registerSetting("supportLineDistance");
    
    registerSetting("supportBottomStairDistance");
    registerSetting("supportJoinDistance");
    registerSetting("supportAreaSmoothing");
    registerSetting("supportConnectZigZags");
    registerSetting("supportMinimalAreaSqrt");
    registerSetting("supportTowerDiameter");
    registerSetting("supportTowerRoofAngle");
    
    
    // machine settings for wireframe 
    registerSetting("machineNozzleTipOuterDiameter");
    registerSetting("machineNozzleHeadDistance");
    registerSetting("machineNozzleExpansionAngle");
    
    // wireframe 
    registerSetting("wireframeFlowConnection");
    registerSetting("wireframeFlowFlat");
    registerSetting("wireframePrintspeedBottom");
    registerSetting("wireframePrintspeedUp");
    registerSetting("wireframePrintspeedDown");
    registerSetting("wireframePrintspeedFlat");
    registerSetting("wireframeNozzleClearance");
    registerSetting("wireframeConnectionHeight");
    registerSetting("wireframeRoofInset");
    registerSetting("wireframeFlatDelay");
    registerSetting("wireframeBottomDelay");
    registerSetting("wireframeTopDelay");
    registerSetting("wireframeUpDistHalfSpeed");
    registerSetting("wireframeTopJump");
    registerSetting("wireframeFallDown");
    registerSetting("wireframeDragAlong");
    registerSetting("wireframeStrategy");
    registerSetting("wireframeStraightBeforeDown");
    registerSetting("wireframeRoofFallDown");
    registerSetting("wireframeRoofDragAlong");
    registerSetting("wireframeRoofOuterDelay");
}
