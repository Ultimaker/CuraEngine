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
    
    IntPoint extruderOffset[16];
    const char* startCode;
    const char* endCode;
    
    ConfigSettings();
    bool setSetting(const char* key, const char* value);
};

#endif//SETTINGS_H
