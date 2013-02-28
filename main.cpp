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
#include "gcodeExport.h"

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
};

static int verbose_flag;
static int layerparts_flag;

void processFile(const char* input_filename,const char* output_filename)
{
    Config config;
    config.filamentDiameter = 2890;
    config.initialLayerThickness = 300;
    config.layerThickness = 100;
    config.extrusionWidth = 400;
    config.insetCount = 2;
    config.downSkinCount = 6;
    config.upSkinCount = 6;
    FMatrix3x3 matrix;
    
    double t = getTime();
    fprintf(stderr,"Loading %s from disk...",input_filename);
    SimpleModel* m = loadModel(input_filename, matrix);
    fprintf(stderr, "\nLoaded from disk in %5.3fs\n", timeElapsed(t));
    if (!m)
    {
        fprintf(stderr, "Failed to load model: %s\n", input_filename);
        return;
    }
    fprintf(stderr,"Analyzing and optimizing model...\n");
    OptimizedModel* om = new OptimizedModel(m, Point3(102500, 102500, 0));
    fprintf(stderr, "  #Face counts: %i %i %0.1f%%\n", (int)m->faces.size(), (int)om->faces.size(), float(om->faces.size()) / float(m->faces.size()) * 100);
    fprintf(stderr, "  #Vertex counts: %i %i %0.1f%%\n", (int)m->faces.size() * 3, (int)om->points.size(), float(om->points.size()) / float(m->faces.size() * 3) * 100);
    delete m;
    fprintf(stderr, "Optimize model %5.3fs \n", timeElapsed(t));
    //om->saveDebugSTL("output.stl");
    
    fprintf(stderr,"Slicing model...\n");
    Slicer* slicer = new Slicer(om, config.initialLayerThickness / 2, config.layerThickness);
    delete om;
    fprintf(stderr, "Sliced model in %5.3fs\n", timeElapsed(t));
    //slicer->dumpSegments("output.html");
    
    fprintf(stderr,"Generating layer parts...\n");
    SliceDataStorage storage;
    storage.modelSize = slicer->modelSize;
    createLayerParts(storage, slicer);
    delete slicer;
    fprintf(stderr, "Generated layer parts in %5.3fs\n", timeElapsed(t));
    if(layerparts_flag) dumpLayerparts(storage, "output.html");
    
    GCodeExport gcode(output_filename);
    const unsigned int totalLayers = storage.layers.size();
    gcode.addComment("Generated with Cura_SteamEngine %s",VERSION);
    gcode.addComment("total_layers=%d",totalLayers);
    gcode.addStartCode();

    gcode.setExtrusion(config.initialLayerThickness, config.extrusionWidth, config.filamentDiameter);

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        generateInsets(&storage.layers[layerNr], config.extrusionWidth, config.insetCount);
        if (verbose_flag && (getTime()-t)>2.0) fprintf(stderr, "\rGenerating insets %d of %d...",layerNr+1,totalLayers);
    }
    fprintf(stderr, "Generated inset in %5.3fs\n", timeElapsed(t));

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        generateSkins(layerNr, storage, config.downSkinCount, config.upSkinCount);
        generateSparse(layerNr, storage, config.downSkinCount, config.upSkinCount);
        if (verbose_flag && (getTime()-t)>2.0) fprintf(stderr, "\rGenerating skin %d of %d...",layerNr+1,totalLayers);
    }
    fprintf(stderr, "Generated up/down skin in %5.3fs\n", timeElapsed(t));

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        SliceLayer* layer = &storage.layers[layerNr];
        if (verbose_flag && (getTime()-t)>2.0) 
            fprintf(stderr, "\rWriting layer %d of %d... (%d percent)", layerNr+1, totalLayers, 100*(layerNr+1)/totalLayers);
        
        PathOptimizer partOrderOptimizer(gcode.getPositionXY());
        for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
        {
            partOrderOptimizer.addPolygon(layer->parts[partNr].insets[0][0]);
        }
        partOrderOptimizer.optimize();
        
        gcode.addComment("LAYER:%d", layerNr);
        gcode.setZ(config.initialLayerThickness + layerNr * config.layerThickness);
        for(unsigned int partCounter=0; partCounter<partOrderOptimizer.polyOrder.size(); partCounter++)
        {
            SliceLayerPart* part = &layer->parts[partOrderOptimizer.polyOrder[partCounter]];
            
            for(int insetNr=part->insets.size()-1; insetNr>-1; insetNr--)
            {
                if (insetNr == 0)
                    gcode.addComment("TYPE:WALL-OUTER");
                else
                    gcode.addComment("TYPE:WALL-INNER");
                PathOptimizer insetOrderOptimizer(gcode.getPositionXY());
                insetOrderOptimizer.addPolygons(part->insets[insetNr]);
                insetOrderOptimizer.optimize();
                for(unsigned int polygonNr=0; polygonNr<insetOrderOptimizer.polyOrder.size(); polygonNr++)
                {
                    int nr = insetOrderOptimizer.polyOrder[polygonNr];
                    gcode.addPolygon(part->insets[insetNr][nr], insetOrderOptimizer.polyStart[nr]);
                }
            }
            
            gcode.addComment("TYPE:FILL");
            Polygons fillPolygons;
            //generateConcentricInfill(part->skinOutline, fillPolygons, config.extrusionWidth);
            generateLineInfill(part->skinOutline, fillPolygons, config.extrusionWidth, config.extrusionWidth, 45 + layerNr * 90);
            int sparseSteps[2] = {config.extrusionWidth*5, config.extrusionWidth * 0.8};
            generateConcentricInfill(part->sparseOutline, fillPolygons, sparseSteps, 2);
            generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.extrusionWidth * 10, 45 + layerNr * 90);
            
            PathOptimizer fillOrderOptimizer(gcode.getPositionXY());
            fillOrderOptimizer.addPolygons(fillPolygons);
            fillOrderOptimizer.optimize();
            for(unsigned int polygonNr=0; polygonNr<fillOrderOptimizer.polyOrder.size(); polygonNr++)
            {
                int nr = fillOrderOptimizer.polyOrder[polygonNr];
                gcode.addPolygon(fillPolygons[nr], fillOrderOptimizer.polyStart[nr]);
            }
            
            if (partCounter < layer->parts.size() - 1)
                gcode.addRetraction();
        }
        gcode.setExtrusion(config.layerThickness, config.extrusionWidth, config.filamentDiameter);
    }
    gcode.addEndCode();
    
    fprintf(stderr, "\nWrote layers in %5.2fs.\n", timeElapsed(t));
    gcode.tellFileSize();

    fprintf(stderr, "Total time elapsed %5.2fs. ", timeElapsed(t,true));
}

int main (int argc, char **argv)
{
    int c;
    const char* input_file = "";
    const char* output_file = "output.gcode";
    static int help_flag = false;

    fprintf(stderr,"Cura_SteamEngine version %s\n",VERSION);

    while (1)
    {
        static struct option long_options[] =
        {
            /* These options set a flag. */
            {"verbose", no_argument,       &verbose_flag, 1},
            {"help", no_argument,       &help_flag, 1},
            /* These options don't set a flag.
            We distinguish them by their indices. */
            {"output",  required_argument, 0, 'o'},
            {"input_filename",  required_argument, 0, 'i'},
            {"layerparts",  no_argument, &layerparts_flag, 'l'},
            {0, 0, 0, 0}
        };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long (argc, argv, "i:o:hvl", long_options, &option_index);
 
        /* Detect the end of the options. */
        if (c == -1)
            break;

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

        case 'c':
            fprintf(stderr,"option -c with value `%s'\n", optarg);
            break;

        case 'i':
            input_file = optarg;
            break;

        case 'o':
            fprintf(stderr,"option -o with value `%s'\n", optarg);
            output_file = optarg;
            break;

        case '?':
            exit(1);
            /* getopt_long already printed an error message. */
        break;

        default:
            fprintf(stderr, "%c `%s'\n", c, optarg);
            exit(0);
        }
    }
     if(help_flag) {
        fprintf(stderr,"Options: \n");
     }

     if(strcmp(input_file, "")) {
         processFile(input_file,output_file);
     }

    /* Print any remaining command line arguments (not options). */
    if (optind < argc)
    {
        printf ("non-option ARGV-elements: ");
        while (optind < argc)
            printf ("%s ", argv[optind++]);
        putchar ('\n');
    }
}
