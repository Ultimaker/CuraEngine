#include <stdio.h>

#include "settings.h"

#define STRINGIFY(_s) #_s
#define SETTING(name) { STRINGIFY(name) , offsetof(ConfigSettings, name), sizeof(ConfigSettings::name) }
#define SETTING2(name, altName) { STRINGIFY(name) , offsetof(ConfigSettings, name), sizeof(ConfigSettings::name) },  { STRINGIFY(altName) , offsetof(ConfigSettings, name), sizeof(ConfigSettings::name) }
static struct { const char* name; size_t offset; size_t itemSize;} settingsIndex[] = {
    SETTING(layerThickness),
    SETTING(initialLayerThickness),
    SETTING(filamentDiameter),
    SETTING(filamentFlow),
    SETTING(extrusionWidth),
    SETTING(insetCount),
    SETTING(downSkinCount),
    SETTING(upSkinCount),
    SETTING(sparseInfillLineDistance),
    SETTING(infillOverlap),
    SETTING(skirtDistance),
    SETTING(skirtLineCount),

    SETTING(initialSpeedupLayers),
    SETTING(initialLayerSpeed),
    SETTING(printSpeed),
    SETTING(infillSpeed),
    SETTING(moveSpeed),
    SETTING(fanOnLayerNr),
    
    SETTING(supportAngle),
    SETTING(supportEverywhere),
    SETTING(supportLineWidth),
    
    SETTING(retractionAmount),
    SETTING(retractionSpeed),
    SETTING(retractionAmountExtruderSwitch),
    SETTING(multiVolumeOverlap),
    SETTING2(objectPosition.X, posx),
    SETTING2(objectPosition.Y, posy),
    SETTING(objectSink),

    SETTING(raftMargin),
    SETTING(raftLineSpacing),
    SETTING(raftBaseThickness),
    SETTING(raftBaseLinewidth),
    SETTING(raftInterfaceThickness),
    SETTING(raftInterfaceLinewidth),
    
    SETTING(minimalLayerTime),
    SETTING(minimalFeedrate),
    SETTING(coolHeadLift),
    SETTING(fanSpeedMin),
    SETTING(fanSpeedMax),
    
    SETTING(fixHorrible),
    
    SETTING(extruderOffset[1].X),
    SETTING(extruderOffset[1].Y),
    SETTING(extruderOffset[2].X),
    SETTING(extruderOffset[2].Y),
    SETTING(extruderOffset[3].X),
    SETTING(extruderOffset[3].Y),
{NULL, 0}};

#undef STRINGIFY
#undef SETTING

bool ConfigSettings::setSetting(const char* key, const char* value)
{
    for(unsigned int n=0; settingsIndex[n].name; n++)
    {
        if (strcasecmp(key, settingsIndex[n].name) == 0)
        {
            switch(settingsIndex[n].itemSize)
            {
            case sizeof(int):
                {
                    int* ptr = (int*)(((int8_t*)&this->layerThickness) + settingsIndex[n].offset);
                    *ptr = atoi(value);
                }
                break;
            case sizeof(int64_t):
                {
                    int64_t* ptr = (int64_t*)(((int8_t*)&this->layerThickness) + settingsIndex[n].offset);
                    *ptr = atoi(value);
                }
                break;
            default:
                fprintf(stderr, "ERR: Unknown itemSize in settingsIndex: %i\n", settingsIndex[n].itemSize);
            }
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
