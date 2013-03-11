#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <getopt.h>

#include "utils/gettime.h"
#include "sliceDataStorage.h"

#include "modelFile/modelFile.h"
#include "optimizedModel.h"
#include "slicer.h"
#include "layerPart.h"
#include "inset.h"
#include "skin.h"
#include "infill.h"
#include "pathOptimizer.h"
#include "skirt.h"
#include "comb.h"
#include "gcodeExport.h"

void optimizePolygon(ClipperLib::Polygon& poly)
{
    Point p0 = poly[poly.size()-1];
    for(unsigned int i=0;i<poly.size();i++)
    {
        Point p1 = poly[i];
        if (shorterThen(p0 - p1, 100))
        {
            poly.erase(poly.begin() + i);
            i --;
        }else{
            p0 = p1;
        }
    }
}

void optimizePolygons(Polygons& polys)
{
    for(unsigned int n=0;n<polys.size();n++)
    {
        optimizePolygon(polys[n]);
    }
}

#define VERSION "0.1"
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
    
    int initialSpeedupLayers;
    int initialLayerSpeed;
    int printSpeed;
    int moveSpeed;
    int fanOnLayerNr;
    FMatrix3x3 matrix;
    Point objectPosition;
};

static int verbose_flag;

void processFile(const char* input_filename, Config& config, GCodeExport& gcode)
{
    double t = getTime();
    fprintf(stdout,"Loading %s from disk...",input_filename);
    SimpleModel* m = loadModel(input_filename, config.matrix);
    fprintf(stdout, "\nLoaded from disk in %5.3fs\n", timeElapsed(t));
    if (!m)
    {
        fprintf(stdout, "Failed to load model: %s\n", input_filename);
        return;
    }
    fprintf(stdout,"Analyzing and optimizing model...\n");
    OptimizedModel* om = new OptimizedModel(m, Point3(config.objectPosition.X, config.objectPosition.Y, 0));
    fprintf(stdout, "  #Face counts: %i %i %0.1f%%\n", (int)m->faces.size(), (int)om->faces.size(), float(om->faces.size()) / float(m->faces.size()) * 100);
    fprintf(stdout, "  #Vertex counts: %i %i %0.1f%%\n", (int)m->faces.size() * 3, (int)om->points.size(), float(om->points.size()) / float(m->faces.size() * 3) * 100);
    delete m;
    fprintf(stdout, "Optimize model %5.3fs \n", timeElapsed(t));
    //om->saveDebugSTL("output.stl");
    
    fprintf(stdout,"Slicing model...\n");
    Slicer* slicer = new Slicer(om, config.initialLayerThickness / 2, config.layerThickness);
    delete om;
    fprintf(stdout, "Sliced model in %5.3fs\n", timeElapsed(t));
    //slicer->dumpSegments("output.html");
    
    fprintf(stdout,"Generating layer parts...\n");
    SliceDataStorage storage;
    storage.modelSize = slicer->modelSize;
    createLayerParts(storage, slicer);
    delete slicer;
    fprintf(stdout, "Generated layer parts in %5.3fs\n", timeElapsed(t));
    //dumpLayerparts(storage, "output.html");
    
    const unsigned int totalLayers = storage.layers.size();
    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        generateInsets(&storage.layers[layerNr], config.extrusionWidth, config.insetCount);
        if (verbose_flag && (getTime()-t)>2.0) fprintf(stdout, "\rGenerating insets %d of %d...",layerNr+1,totalLayers);
    }
    fprintf(stdout, "Generated inset in %5.3fs\n", timeElapsed(t));

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        generateSkins(layerNr, storage, config.extrusionWidth, config.downSkinCount, config.upSkinCount);
        generateSparse(layerNr, storage, config.extrusionWidth, config.downSkinCount, config.upSkinCount);
        if (verbose_flag && (getTime()-t)>2.0) fprintf(stdout, "\rGenerating skin %d of %d...",layerNr+1,totalLayers);
    }
    fprintf(stdout, "Generated up/down skin in %5.3fs\n", timeElapsed(t));
    generateSkirt(storage, config.skirtDistance, config.extrusionWidth, config.skirtLineCount);

    gcode.addComment("GCode for file %s", input_filename);
    gcode.addComment("total_layers=%d",totalLayers);
    gcode.addStartCode();

    gcode.setExtrusion(config.initialLayerThickness, config.extrusionWidth, config.filamentDiameter);
    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        if (verbose_flag && (getTime()-t)>2.0) 
            fprintf(stdout, "\rWriting layer %d of %d... (%d percent)", layerNr+1, totalLayers, 100*(layerNr+1)/totalLayers);
        
        gcode.addComment("LAYER:%d", layerNr);
        if (int(layerNr) == config.fanOnLayerNr)
            gcode.addFanCommand(255);
        gcode.setZ(config.initialLayerThickness + layerNr * config.layerThickness);
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
            optimizePolygons(part->insets[0]);
            
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
            //int sparseSteps[1] = {config.extrusionWidth};
            //generateConcentricInfill(part->skinOutline, fillPolygons, sparseSteps, 1);
            generateLineInfill(part->skinOutline, fillPolygons, config.extrusionWidth, config.extrusionWidth, 15, 45 + layerNr * 90);
            //int sparseSteps[2] = {config.extrusionWidth*5, config.extrusionWidth * 0.8};
            //generateConcentricInfill(part->sparseOutline, fillPolygons, sparseSteps, 2);
            generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance, 15, 45 + layerNr * 90);

            gcode.addPolygonsByOptimizer(fillPolygons);
            
            if (partCounter < layer->parts.size() - 1)
                gcode.addRetraction();
        }
        gcode.setExtrusion(config.layerThickness, config.extrusionWidth, config.filamentDiameter);
    }
    
    fprintf(stdout, "\nWrote layers in %5.2fs.\n", timeElapsed(t));
    gcode.tellFileSize();
    gcode.addFanCommand(0);

    fprintf(stdout, "Total time elapsed %5.2fs. ", timeElapsed(t,true));
}

int main (int argc, char **argv)
{
    const char* output_file = "output.gcode";
    int help_flag = false;
    Config config;

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

    fprintf(stdout,"Cura_SteamEngine version %s\n", VERSION);

    struct option long_options[] =
    {
        /* These options set a flag. */
        {"verbose", no_argument,       &verbose_flag, 1},
        {"help",    no_argument,       &help_flag, 1},
        {"output",  required_argument, NULL, 'o'},
        {NULL, 0, NULL, 0}
    };
    /* getopt_long stores the option index here. */
    int option_index = 0;
    int c;
    while ((c = getopt_long (argc, argv, "o:m:s:hv", long_options, &option_index)) > -1)
    {
        switch (c)
        {
        case 0:
            /* If this option set a flag, do nothing else now. */
            if (long_options[option_index].flag != 0)
                break;
            fprintf (stderr,"option %s", long_options[option_index].name);
            if (optarg)
                fprintf(stderr," with arg %s", optarg);
            fprintf (stderr,"\n");
            break;

        case 'o':
            output_file = optarg;
            break;
        case 'm':
            sscanf(optarg, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                &config.matrix.m[0][0], &config.matrix.m[0][1], &config.matrix.m[0][2],
                &config.matrix.m[1][0], &config.matrix.m[1][1], &config.matrix.m[1][2],
                &config.matrix.m[2][0], &config.matrix.m[2][1], &config.matrix.m[2][2]);
            break;
        case 's':
            {
                char* valuePtr = strchr(optarg, '=');
                if (!valuePtr) break;
                *valuePtr++ = '\0';
#define STRINGIFY(_s) #_s
#define SETTING(longName, shortName) if (strcasecmp(optarg, STRINGIFY(longName)) == 0 || strcasecmp(optarg, STRINGIFY(shortName)) == 0) { config.longName = atoi(valuePtr); }
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
#undef SETTING
            }
            break;
        case '?':
            exit(1);
            /* getopt_long already printed an error message. */
            break;
        default:
            fprintf(stderr, "Unknown option: %d `%s'\n", c, optarg);
            exit(0);
        }
    }
    if(help_flag) {
        fprintf(stderr,"Options: \n");
        return 1;
    }
    if (output_file == NULL)
    {
        fprintf(stderr, "No output file specified\n");
        return 1;
    }

    /* Print any remaining command line arguments (not options). */
    if (optind < argc)
    {
        GCodeExport gcode(output_file);
        gcode.addComment("Generated with Cura_SteamEngine %s", VERSION);
        while (optind < argc)
            processFile(argv[optind++], config, gcode);
        gcode.addEndCode();
    }
}
