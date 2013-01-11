#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include "clipper/clipper.hpp"

#include "utils/gettime.h"

#include "modelFile/modelFile.h"
#include "optimizedModel.h"
#include "slicer.h"
#include "layerPart.h"
#include "inset.h"
#include "gcodeExport.h"

class Config
{
public:
    int layerThickness;
    int initialLayerThickness;
    int filamentDiameter;
    int extrusionWidth;
    int insetCount;
};

void processFile(const char* filename)
{
    Config config;
    config.filamentDiameter = 2890;
    config.initialLayerThickness = 300;
    config.layerThickness = 100;
    config.extrusionWidth = 400;
    config.insetCount = 20;
    FMatrix3x3 matrix;
    
    double t = getTime();
    SimpleModel* m = loadModel(filename, matrix);
    fprintf(stderr, "Load from disk time: %f\n", getTime() - t); t = getTime();
    if (!m)
    {
        fprintf(stderr, "Failed to load model: %s\n", filename);
        return;
    }
    OptimizedModel* om = new OptimizedModel(m, Point3(102500, 102500, 0));
    fprintf(stderr, "#Face counts: %i %i %0.1f%%\n", m->faces.size(), om->faces.size(), float(om->faces.size()) / float(m->faces.size()) * 100);
    fprintf(stderr, "#Vertex counts: %i %i %0.1f%%\n", m->faces.size() * 3, om->points.size(), float(om->points.size()) / float(m->faces.size() * 3) * 100);
    delete m;
    fprintf(stderr, "Optimize model: %f\n", getTime() - t); t = getTime();
    om->saveDebugSTL("output.stl");
    
    Slicer* slicer = new Slicer(om, config.initialLayerThickness / 2, config.layerThickness);
    delete om;
    fprintf(stderr, "Slice model: %f\n", getTime() - t); t = getTime();
    //slicer.dumpSegments("output.html");
    
    LayerParts layerParts(slicer);
    delete slicer;
    fprintf(stderr, "Create layerparts: %f\n", getTime() - t); t = getTime();
    //layerParts.dumpLayerparts("output.html");
    
    GCodeExport gcode("output.gcode");
    gcode.addComment("\nG28\nG92 E0");
    gcode.setExtrusion(config.initialLayerThickness, config.extrusionWidth, config.filamentDiameter);
    for(unsigned int layerNr=0; layerNr<layerParts.layers.size(); layerNr++)
    {
        InsetLayer inset(&layerParts.layers[layerNr], config.extrusionWidth, config.insetCount);
        
        gcode.addComment("LAYER:%d", layerNr);
        for(unsigned int partNr=0; partNr<inset.parts.size(); partNr++)
        {
            //for(unsigned int insetNr=0; insetNr<inset.parts[partNr].inset.size(); insetNr++)
            for(int insetNr=inset.parts[partNr].inset.size()-1; insetNr>-1; insetNr--)
            {
                if (insetNr == 0)
                    gcode.addComment("TYPE:WALL-OUTER");
                else
                    gcode.addComment("TYPE:WALL-INNER");
                for(unsigned int polygonNr=0; polygonNr<inset.parts[partNr].inset[insetNr].size(); polygonNr++)
                {
                    gcode.addPolygon(inset.parts[partNr].inset[insetNr][polygonNr], config.initialLayerThickness + layerNr * config.layerThickness);
                }
            }
        }
        gcode.setExtrusion(config.layerThickness, config.extrusionWidth, config.filamentDiameter);
    }
}

int main(int argc, char** argv)
{
    for(int i=1;i<argc;i++)
    {
        processFile(argv[i]);
    }
    processFile("C:/Software/Cura/Cura/resources/example/UltimakerRobot_support.stl");
    //processFile("C:/Models/cellularThing_optimizedForMakerbot.stl");
    //processFile("C:/Models/Knot-Thing.stl");
    //processFile("C:/Models/DoubleHelix.stl");
    //processFile("C:/Models/box_20x20x10.stl");
    //processFile("C:/Models/FanHolder2.stl");
    //processFile("C:/Models/EnclosedGears.stl");
    //processFile("C:/Models/GameTiles.stl");
}
