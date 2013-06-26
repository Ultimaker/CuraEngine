#ifndef SETTINGS_H
#define SETTINGS_H

#include <utils/floatpoint.h>
#include <stddef.h>

class ConfigSettings
{
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
    Point objectPosition;
    int objectSink;
    
    int fixHorrible;
    
    Point extruderOffset[16];
    const char* startCode;
    const char* endCode;
    
    bool setSetting(const char* key, const char* value);
};

#endif//SETTINGS_H
