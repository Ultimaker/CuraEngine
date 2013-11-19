/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <execinfo.h>
#include <sys/resource.h>
#endif
#include <stddef.h>

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"

#include "modelFile/modelFile.h"
#include "settings.h"
#include "optimizedModel.h"
#include "multiVolumes.h"
#include "polygonOptimizer.h"
#include "slicer.h"
#include "layerPart.h"
#include "inset.h"
#include "skin.h"
#include "infill.h"
#include "bridge.h"
#include "support.h"
#include "pathOrderOptimizer.h"
#include "skirt.h"
#include "raft.h"
#include "comb.h"
#include "gcodeExport.h"
#include "fffProcessor.h"

void print_usage()
{
    printf("usage: CuraEngine [-h] [-v] [-m 3x3matrix] [-s <settingkey>=<value>] -o <output.gcode> <model.stl>\n");
}

void signal_FPE(int n)
{
    (void)n;
    printf("Arithmetic exception.\n");
    exit(1);
}

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac.
    setpriority(PRIO_PROCESS, 0, 10);
#endif
    signal(SIGFPE, signal_FPE);

    ConfigSettings config;
    fffProcessor processor(config);

    config.filamentDiameter = 2890;
    config.filamentFlow = 100;
    config.initialLayerThickness = 300;
    config.layerThickness = 100;
    config.extrusionWidth = 400;
    config.insetCount = 2;
    config.downSkinCount = 6;
    config.upSkinCount = 6;
    config.initialSpeedupLayers = 4;
    config.initialLayerSpeed = 20;
    config.printSpeed = 50;
    config.infillSpeed = 50;
    config.moveSpeed = 200;
    config.fanFullOnLayerNr = 2;
    config.skirtDistance = 6000;
    config.skirtLineCount = 1;
    config.skirtMinLength = 0;
    config.sparseInfillLineDistance = 100 * config.extrusionWidth / 20;
    config.infillOverlap = 15;
    config.objectPosition.X = 102500;
    config.objectPosition.Y = 102500;
    config.objectSink = 0;
    config.supportAngle = -1;
    config.supportEverywhere = 0;
    config.supportLineDistance = config.sparseInfillLineDistance;
    config.supportExtruder = -1;
    config.supportXYDistance = 700;
    config.supportZDistance = 150;
    config.retractionAmount = 4500;
    config.retractionSpeed = 45;
    config.retractionAmountExtruderSwitch = 14500;
    config.retractionMinimalDistance = 1500;
    config.minimalExtrusionBeforeRetraction = 100;
    config.enableOozeShield = 0;
    config.enableCombing = 1;
    config.enableWipeTower = 0;
    config.multiVolumeOverlap = 0;

    config.minimalLayerTime = 5;
    config.minimalFeedrate = 10;
    config.coolHeadLift = 1;
    config.fanSpeedMin = 100;
    config.fanSpeedMax = 100;

    config.raftMargin = 5000;
    config.raftLineSpacing = 1000;
    config.raftBaseThickness = 0;
    config.raftBaseLinewidth = 0;
    config.raftInterfaceThickness = 0;
    config.raftInterfaceLinewidth = 0;

    config.spiralizeMode = 0;
    config.fixHorrible = 0;
    config.gcodeFlavor = GCODE_FLAVOR_REPRAP;
    memset(config.extruderOffset, 0, sizeof(config.extruderOffset));
    
    config.startCode =
        "M109 S210     ;Heatup to 210C\n"
        "G21           ;metric values\n"
        "G90           ;absolute positioning\n"
        "G28           ;Home\n"
        "G1 Z15.0 F300 ;move the platform down 15mm\n"
        "G92 E0        ;zero the extruded length\n"
        "G1 F200 E5    ;extrude 5mm of feed stock\n"
        "G92 E0        ;zero the extruded length again\n";
    config.endCode = 
        "M104 S0                     ;extruder heater off\n"
        "M140 S0                     ;heated bed heater off (if you have it)\n"
        "G91                            ;relative positioning\n"
        "G1 E-1 F300                    ;retract the filament a bit before lifting the nozzle, to release some of the pressure\n"
        "G1 Z+0.5 E-5 X-20 Y-20 F9000   ;move Z up a bit and retract filament even more\n"
        "G28 X0 Y0                      ;move X/Y to min endstops, so the head is out of the way\n"
        "M84                         ;steppers off\n"
        "G90                         ;absolute positioning\n";

    fprintf(stderr,"Cura_SteamEngine version %s\n", VERSION);

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
                    if (!processor.setTargetFile(argv[argn]))
                    {
                        logError("Failed to open %s for output.\n", argv[argn]);
                        exit(1);
                    }
                    break;
                case 's':
                    {
                        argn++;
                        char* valuePtr = strchr(argv[argn], '=');
                        if (valuePtr)
                        {
                            *valuePtr++ = '\0';
                            
                            if (!config.setSetting(argv[argn], valuePtr))
                                printf("Setting found: %s %s\n", argv[argn], valuePtr);
                        }
                    }
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
            try {
                processor.processFile(argv[argn]);
            }catch(...){
                printf("Unknown exception\n");
                exit(1);
            }
        }
    }
    
    processor.finalize();
}
