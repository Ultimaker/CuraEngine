#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <sys/resource.h>
#endif

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"

#include "modelFile/modelFile.h"
#include "optimizedModel.h"
#include "polygonOptimizer.h"
#include "slicer.h"
#include "layerPart.h"
#include "inset.h"
#include "skin.h"
#include "infill.h"
#include "bridge.h"
#include "support.h"
#include "pathOptimizer.h"
#include "skirt.h"
#include "comb.h"
#include "gcodeExport.h"

#define VERSION "0.2"
class Config
{
public:
    int layerThickness;
    int initialLayerThickness;
    int filamentDiameter;
    int extrusionWidth;
    int insetCount;
    int downSkinCount;
    int upSkinCount;
    int sparseInfillLineDistance;
    int skirtDistance;
    int skirtLineCount;
    int retractionAmount;
    int retractionSpeed;
    
    int initialSpeedupLayers;
    int initialLayerSpeed;
    int printSpeed;
    int moveSpeed;
    int fanOnLayerNr;
    int supportAngle;
    int supportEverywhere;
    
    FMatrix3x3 matrix;
    Point objectPosition;
    int objectSink;
    
    int fixHorrible;
};

int verbose_level;
int maxObjectHeight;

void processFile(const char* input_filename, Config& config, GCodeExport& gcode, bool firstFile)
{
    double t = getTime();
    log("Loading %s from disk...\n", input_filename);
    SimpleModel* m = loadModel(input_filename, config.matrix);
    if (!m)
    {
        log("Failed to load model: %s\n", input_filename);
        return;
    }
    log("Loaded from disk in %5.3fs\n", timeElapsed(t));
    log("Analyzing and optimizing model...\n");
    OptimizedModel* om = new OptimizedModel(m, Point3(config.objectPosition.X, config.objectPosition.Y, -config.objectSink));
    log("  Face counts: %i -> %i %0.1f%%\n", (int)m->faces.size(), (int)om->faces.size(), float(om->faces.size()) / float(m->faces.size()) * 100);
    log("  Vertex counts: %i -> %i %0.1f%%\n", (int)m->faces.size() * 3, (int)om->points.size(), float(om->points.size()) / float(m->faces.size() * 3) * 100);
    delete m;
    log("Optimize model %5.3fs \n", timeElapsed(t));
    om->saveDebugSTL("output.stl");
    
    log("Slicing model...\n");
    Slicer* slicer = new Slicer(om, config.initialLayerThickness / 2, config.layerThickness, config.fixHorrible);
    log("Sliced model in %5.3fs\n", timeElapsed(t));
    //slicer->dumpSegments("output.html");

    SliceDataStorage storage;
    if (config.supportAngle > -1)
    {
        fprintf(stdout,"Generating support map...\n");
        generateSupportGrid(storage.support, om, config.initialLayerThickness / 2, config.layerThickness);
    }
    storage.modelSize = om->modelSize;
    storage.modelMin = om->vMin;
    storage.modelMax = om->vMax;
    delete om;
    
    fprintf(stdout,"Generating layer parts...\n");
    createLayerParts(storage, slicer, config.fixHorrible);
    delete slicer;
    fprintf(stdout, "Generated layer parts in %5.3fs\n", timeElapsed(t));
    //dumpLayerparts(storage, "output.html");
    
    const unsigned int totalLayers = storage.layers.size();
    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        generateInsets(&storage.layers[layerNr], config.extrusionWidth, config.insetCount);
        
        logProgress("inset",layerNr+1,totalLayers);
    }
    log("Generated inset in %5.3fs\n", timeElapsed(t));
    fflush(stdout);

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        generateSkins(layerNr, storage, config.extrusionWidth, config.downSkinCount, config.upSkinCount);
        generateSparse(layerNr, storage, config.extrusionWidth, config.downSkinCount, config.upSkinCount);
        logProgress("skin",layerNr+1,totalLayers);
    }
    log("Generated up/down skin in %5.3fs\n", timeElapsed(t));
    generateSkirt(storage, config.skirtDistance, config.extrusionWidth, config.skirtLineCount);
    
    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        for(unsigned int partNr=0; partNr<storage.layers[layerNr].parts.size(); partNr++)
        {
            if (layerNr > 0)
                storage.layers[layerNr].parts[partNr].bridgeAngle = bridgeAngle(&storage.layers[layerNr].parts[partNr], &storage.layers[layerNr-1]);
            else
                storage.layers[layerNr].parts[partNr].bridgeAngle = -1;
        }
    }

    gcode.setRetractionSettings(config.retractionAmount, config.retractionSpeed);
    if (firstFile)
    {
        gcode.addStartCode();
    }else{
        gcode.resetExtrusionValue();
        gcode.setSpeeds(config.moveSpeed, config.printSpeed);
        gcode.addRetraction();
        gcode.setZ(maxObjectHeight + 5);
        gcode.addMove(config.objectPosition, 0.0);
    }
    gcode.addComment("total_layers=%d",totalLayers);

    gcode.setExtrusion(config.initialLayerThickness, config.extrusionWidth, config.filamentDiameter);
    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        logProgress("export", layerNr+1, totalLayers);
        
        gcode.addComment("LAYER:%d", layerNr);
        gcode.resetExtrusionValue();
        if (int(layerNr) == config.fanOnLayerNr)
            gcode.addFanCommand(255);
        int32_t z = config.initialLayerThickness + layerNr * config.layerThickness;
        gcode.setZ(z);
        if (layerNr == 0)
        {
            gcode.setSpeeds(config.moveSpeed, config.initialLayerSpeed);
            gcode.addComment("TYPE:SKIRT");
            gcode.addPolygonsByOptimizer(storage.skirt);
        }
        if (int(layerNr) < config.initialSpeedupLayers)
        {
            int n = config.initialSpeedupLayers;
            gcode.setSpeeds((config.initialLayerSpeed * (n - layerNr) + config.moveSpeed * (layerNr)) / n, (config.initialLayerSpeed * (n - layerNr) + config.printSpeed * (layerNr)) / n);
        }else{
            gcode.setSpeeds(config.moveSpeed, config.printSpeed);
        }
        
        SliceLayer* layer = &storage.layers[layerNr];
        
        PathOptimizer partOrderOptimizer(gcode.getPositionXY());
        for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
        {
            partOrderOptimizer.addPolygon(layer->parts[partNr].insets[0][0]);
        }
        partOrderOptimizer.optimize();
        
        for(unsigned int partCounter=0; partCounter<partOrderOptimizer.polyOrder.size(); partCounter++)
        {
            SliceLayerPart* part = &layer->parts[partOrderOptimizer.polyOrder[partCounter]];
            
            gcode.setCombBoundary(&part->insets[0]);
            for(int insetNr=part->insets.size()-1; insetNr>-1; insetNr--)
            {
                if (insetNr == 0)
                    gcode.addComment("TYPE:WALL-OUTER");
                else
                    gcode.addComment("TYPE:WALL-INNER");
                gcode.addPolygonsByOptimizer(part->insets[insetNr]);
            }
            
            gcode.addComment("TYPE:FILL");
            Polygons fillPolygons;
            int fillAngle = 45;
            if (layerNr & 1) fillAngle += 90;
            //int sparseSteps[1] = {config.extrusionWidth};
            //generateConcentricInfill(part->skinOutline, fillPolygons, sparseSteps, 1);
            generateLineInfill(part->skinOutline, fillPolygons, config.extrusionWidth, config.extrusionWidth, 15, (part->bridgeAngle > -1) ? part->bridgeAngle : fillAngle);
            //int sparseSteps[2] = {config.extrusionWidth*5, config.extrusionWidth * 0.8};
            //generateConcentricInfill(part->sparseOutline, fillPolygons, sparseSteps, 2);
            generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance, 15, fillAngle);

            gcode.addPolygonsByOptimizer(fillPolygons);
            
            if (partCounter < layer->parts.size() - 1)
                gcode.addRetraction();
        }
        gcode.setCombBoundary(NULL);
        
        if (config.supportAngle > -1)
        {
            SupportPolyGenerator supportGenerator(storage.support, z, 60, config.supportEverywhere > 0);
            if (supportGenerator.polygons.size() > 0)
            {
                gcode.addComment("TYPE:SUPPORT");
                gcode.addPolygonsByOptimizer(supportGenerator.polygons);
            }
        }
        
        gcode.setExtrusion(config.layerThickness, config.extrusionWidth, config.filamentDiameter);
    }

    /* support debug
    for(int32_t y=0; y<storage.support.gridHeight; y++)
    {
        for(int32_t x=0; x<storage.support.gridWidth; x++)
        {
            unsigned int n = x+y*storage.support.gridWidth;
            if (storage.support.grid[n].size() < 1) continue;
            int32_t z = storage.support.grid[n][0].z;
            gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, 0), 0);
            gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, z), z);
            gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, 0), 0);
        }
    }
    //*/
    
    log("Wrote layers in %5.2fs.\n", timeElapsed(t));
    gcode.tellFileSize();
    gcode.addFanCommand(0);

    log("Total time elapsed %5.2fs.\n", timeElapsed(t,true));
    
    maxObjectHeight = std::max(maxObjectHeight, storage.modelSize.z);
}

void setConfig(Config& config, char* str)
{
    char* valuePtr = strchr(str, '=');
    if (!valuePtr) return;
    *valuePtr++ = '\0';
#define STRINGIFY(_s) #_s
#define SETTING(longName, shortName) if (strcasecmp(str, STRINGIFY(longName)) == 0 || strcasecmp(str, STRINGIFY(shortName)) == 0) { config.longName = atoi(valuePtr); }
    SETTING(layerThickness, lt);
    SETTING(initialLayerThickness, ilt);
    SETTING(filamentDiameter, fd);
    SETTING(extrusionWidth, ew);
    SETTING(insetCount, ic);
    SETTING(downSkinCount, dsc);
    SETTING(upSkinCount, usc);
    SETTING(sparseInfillLineDistance, sild);
    SETTING(skirtDistance, sd);
    SETTING(skirtLineCount, slc);

    SETTING(initialSpeedupLayers, isl);
    SETTING(initialLayerSpeed, ils);
    SETTING(printSpeed, ps);
    SETTING(moveSpeed, ms);
    SETTING(fanOnLayerNr, fl);
    SETTING(supportAngle, supa);
    SETTING(supportEverywhere, supe);
    SETTING(retractionAmount, reta);
    SETTING(retractionSpeed, rets);
    SETTING(objectPosition.X, posx);
    SETTING(objectPosition.Y, posy);
    SETTING(objectSink, objsink);
#undef SETTING
}

void print_usage()
{
    printf("TODO\n");
}

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac.
    setpriority(PRIO_PROCESS, 0, 10);
#endif
    GCodeExport gcode;
    Config config;
    int fileNr = 0;

    config.filamentDiameter = 2890;
    config.initialLayerThickness = 300;
    config.layerThickness = 100;
    config.extrusionWidth = 400;
    config.insetCount = 2;
    config.downSkinCount = 6;
    config.upSkinCount = 6;
    config.initialSpeedupLayers = 4;
    config.initialLayerSpeed = 20;
    config.printSpeed = 50;
    config.moveSpeed = 200;
    config.fanOnLayerNr = 2;
    config.skirtDistance = 6000;
    config.skirtLineCount = 1;
    config.sparseInfillLineDistance = 100 * config.extrusionWidth / 20;
    config.objectPosition = Point(102500, 102500);
    config.objectSink = 0;
    config.supportAngle = -1;
    config.supportEverywhere = 0;
    config.retractionAmount = 4.5;
    config.retractionSpeed = 45;
    config.fixHorrible = 0;

    fprintf(stdout,"Cura_SteamEngine version %s\n", VERSION);

    for(int argn = 1; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            for(str++; *str; str++)
            {
                switch(*str)
                {
                case 'h':
                    print_usage();
                    exit(1);
                case 'v':
                    verbose_level++;
                    break;
                case 'b':
                    argn++;
                    binaryMeshBlob = fopen(argv[argn], "rb");
                    break;
                case 'o':
                    argn++;
                    gcode.setFilename(argv[argn]);
                    if (!gcode.isValid())
                    {
                        logError("Failed to open %s for output.\n", argv[argn]);
                        exit(1);
                    }
                    gcode.addComment("Generated with Cura_SteamEngine %s", VERSION);
                    break;
                case 's':
                    argn++;
                    setConfig(config, argv[argn]);
                    break;
                case 'm':
                    argn++;
                    sscanf(argv[argn], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &config.matrix.m[0][0], &config.matrix.m[0][1], &config.matrix.m[0][2],
                        &config.matrix.m[1][0], &config.matrix.m[1][1], &config.matrix.m[1][2],
                        &config.matrix.m[2][0], &config.matrix.m[2][1], &config.matrix.m[2][2]);
                    break;
                default:
                    logError("Unknown option: %c\n", *str);
                    break;
                }
            }
        }else{
            if (!gcode.isValid())
            {
                logError("No output file specified\n");
                return 1;
            }
            processFile(argv[argn], config, gcode, fileNr == 0);
            fileNr ++;
        }
    }
    if (gcode.isValid())
    {
        gcode.addEndCode();
    }
}
