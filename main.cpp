#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include <getopt.h>

#include "clipper/clipper.hpp"

#include "utils/gettime.h"

#include "modelFile/modelFile.h"
#include "optimizedModel.h"
#include "slicer.h"
#include "layerPart.h"
#include "inset.h"
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
    config.insetCount = 20;
    FMatrix3x3 matrix;
    
    double t = getTime();
    std::fprintf(stderr,"Loading %s from disk...",input_filename);
    SimpleModel* m = loadModel(input_filename, matrix);
    fprintf(stderr, "\nLoaded from disk in %5.3fs\n", getTime() - t); t = getTime();
    if (!m)
    {
        fprintf(stderr, "Failed to load model: %s\n", input_filename);
        return;
    }
    std::fprintf(stderr,"Analyzing model...");
    OptimizedModel* om = new OptimizedModel(m, Point3(102500, 102500, 0));
    fprintf(stderr, "#Face counts: %i %i %0.1f%%\n", (int)m->faces.size(), (int)om->faces.size(), float(om->faces.size()) / float(m->faces.size()) * 100);
    fprintf(stderr, "#Vertex counts: %i %i %0.1f%%\n", (int)m->faces.size() * 3, (int)om->points.size(), float(om->points.size()) / float(m->faces.size() * 3) * 100);
    delete m;
    fprintf(stderr, "Optimize model: %f\n", getTime() - t); t = getTime();
    om->saveDebugSTL("output.stl");
    
    std::fprintf(stderr,"Slicing model...\n");
    Slicer* slicer = new Slicer(om, config.initialLayerThickness / 2, config.layerThickness);
    delete om;
    fprintf(stderr, "Sliced model in %5.3fs\n", getTime() - t); t = getTime();
    //slicer.dumpSegments("output.html");
    
    std::fprintf(stderr,"Generating layer parts...\n");
    LayerParts layerParts(slicer);
    delete slicer;
    fprintf(stderr, "Generated layer parts in %5.3fs\n", getTime() - t); t = getTime();
    if(layerparts_flag) layerParts.dumpLayerparts("output.html");
    
    GCodeExport gcode(output_filename);
    const unsigned int totalLayers = layerParts.layers.size();
    //gcode.addComment(printf("GCode created with Cura Engine %s",VERSION));
    gcode.addComment("Generated with Cura_SteamEngine %s",VERSION);
    gcode.addComment("total_layers=%d",totalLayers);
    gcode.addComment("\nG28\nG92 X-102.5 Y-102.5");

    gcode.setExtrusion(config.initialLayerThickness, config.extrusionWidth, config.filamentDiameter);

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        InsetLayer inset(&layerParts.layers[layerNr], config.extrusionWidth, config.insetCount);
        if (verbose_flag) fprintf(stderr, "\rProcessing layer %d of %d...",layerNr,totalLayers);
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
 
       c = getopt_long (argc, argv, "i:o:hvl",
                        long_options, &option_index);
 
       /* Detect the end of the options. */
       if (c == -1)
         break;
 
       switch (c)
         {
         case 0:
           /* If this option set a flag, do nothing else now. */
           if (long_options[option_index].flag != 0)
             break;
           std::fprintf (stderr,"option %s", long_options[option_index].name);
           if (optarg)
             std::fprintf (stderr," with arg %s", optarg);
           std::fprintf (stderr,"\n");
           break;
 
         case 'c':
           std::fprintf (stderr,"option -c with value `%s'\n", optarg);
           break;
 
         case 'i':
           input_file = optarg;
           break;

         case 'o':
           std::fprintf (stderr,"option -o with value `%s'\n", optarg);
           output_file = optarg;
           break;
 
         case '?':
            exit(1);
           /* getopt_long already printed an error message. */
           break;
 
         default:
           abort ();
         }
     }
     if(help_flag) {
        std::fprintf (stderr,"Options: \n");
     }

     if(strcmp(input_file,"")) {
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
