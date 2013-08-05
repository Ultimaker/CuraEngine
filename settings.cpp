#include <stdio.h>

#include "settings.h"

#define STRINGIFY(_s) #_s
#define SETTING(name) _index.push_back(_ConfigSettingIndex(STRINGIFY(name), &name))
#define SETTING2(name, altName) _index.push_back(_ConfigSettingIndex(STRINGIFY(name), &name)); _index.push_back(_ConfigSettingIndex(STRINGIFY(altName), &name))

ConfigSettings::ConfigSettings()
{
    SETTING(layerThickness);
    SETTING(initialLayerThickness);
    SETTING(filamentDiameter);
    SETTING(filamentFlow);
    SETTING(extrusionWidth);
    SETTING(insetCount);
    SETTING(downSkinCount);
    SETTING(upSkinCount);
    SETTING(sparseInfillLineDistance);
    SETTING(infillOverlap);
    SETTING(skirtDistance);
    SETTING(skirtLineCount);

    SETTING(initialSpeedupLayers);
    SETTING(initialLayerSpeed);
    SETTING(printSpeed);
    SETTING(infillSpeed);
    SETTING(moveSpeed);
    SETTING(fanOnLayerNr);
    
    SETTING(supportAngle);
    SETTING(supportEverywhere);
    SETTING(supportLineWidth);
    
    SETTING(retractionAmount);
    SETTING(retractionSpeed);
    SETTING(retractionAmountExtruderSwitch);
    SETTING(retractionMinimalDistance);
    SETTING(multiVolumeOverlap);
    SETTING2(objectPosition.X, posx);
    SETTING2(objectPosition.Y, posy);
    SETTING(objectSink);

    SETTING(raftMargin);
    SETTING(raftLineSpacing);
    SETTING(raftBaseThickness);
    SETTING(raftBaseLinewidth);
    SETTING(raftInterfaceThickness);
    SETTING(raftInterfaceLinewidth);
    
    SETTING(minimalLayerTime);
    SETTING(minimalFeedrate);
    SETTING(coolHeadLift);
    SETTING(fanSpeedMin);
    SETTING(fanSpeedMax);
    
    SETTING(fixHorrible);
    SETTING(gcodeFlavor);
    
    SETTING(extruderOffset[1].X);
    SETTING(extruderOffset[1].Y);
    SETTING(extruderOffset[2].X);
    SETTING(extruderOffset[2].Y);
    SETTING(extruderOffset[3].X);
    SETTING(extruderOffset[3].Y);
}

#undef STRINGIFY
#undef SETTING

bool ConfigSettings::setSetting(const char* key, const char* value)
{
    for(unsigned int n=0; n < _index.size(); n++)
    {
        if (strcasecmp(key, _index[n].key) == 0)
        {
            *_index[n].ptr = atoi(value);
            return true;
        }
    }
    if (strcasecmp(key, "startCode") == 0)
    {
        this->startCode = value;
        return true;
    }
    if (strcasecmp(key, "endCode") == 0)
    {
        this->endCode = value;
        return true;
    }
    return false;
}
