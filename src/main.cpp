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
#include <vector>

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "utils/string.h"
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


#define ELPP_CUSTOM_COUT std::cerr
#include "easylogging++.h"
_INITIALIZE_EASYLOGGINGPP
#define _ELPP_STACKTRACE_ON_CRASH


void customCrashLogging(int sig) {
    LOG(ERROR) << "Application crashed!";
    el::Helpers::logCrashReason(sig, true);
    el::Loggers::flushAll();
    el::Helpers::crashAbort(sig); // FOLLOWING LINE IS ABSOLUTELY NEEDED AT THE END IN ORDER TO ABORT APPLICATION
    el::Loggers::flushAll();
}


void print_usage()
{
    cura::logError("usage: CuraEngine [-h] [-v] [-m 3x3matrix] [-c <config file>] [-s <settingkey>=<value>] -o <output.gcode> <model.stl>\n");
}

//Signal handler for a "floating point exception", which can also be integer division by zero errors.
void signal_FPE(int n)
{
    (void)n;
    cura::logError("Arithmetic exception.\n");
    exit(1);
}

using namespace cura;

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac. On windows this is done on process creation from the GUI.
    setpriority(PRIO_PROCESS, 0, 10);
#endif

    el::Helpers::setCrashHandler(customCrashLogging);
    el::Configurations conf("LoggerConfig.conf");
    el::Loggers::setDefaultConfigurations(conf,true); //Ensure all current (and future) loggers use the same settings
    LOG(INFO) << "-----Application started-----" << std::endl;
    el::Loggers::addFlag(el::LoggingFlag::AutoSpacing);


    //Register the exception handling for arithmic exceptions, this prevents the "something went wrong" dialog on windows to pop up on a division by zero.
    signal(SIGFPE, signal_FPE);

    fffProcessor processor;
    std::vector<std::string> files;

    logCopyright("Cura_SteamEngine version %s\n", VERSION);
    logCopyright("Copyright (C) 2014 David Braam\n");
    logCopyright("\n");
    logCopyright("This program is free software: you can redistribute it and/or modify\n");
    logCopyright("it under the terms of the GNU Affero General Public License as published by\n");
    logCopyright("the Free Software Foundation, either version 3 of the License, or\n");
    logCopyright("(at your option) any later version.\n");
    logCopyright("\n");
    logCopyright("This program is distributed in the hope that it will be useful,\n");
    logCopyright("but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
    logCopyright("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
    logCopyright("GNU Affero General Public License for more details.\n");
    logCopyright("\n");
    logCopyright("You should have received a copy of the GNU Affero General Public License\n");
    logCopyright("along with this program.  If not, see <http://www.gnu.org/licenses/>.\n");
//*
    processor.setSetting("layerThickness", "100");
    processor.setSetting("initialLayerThickness", "300");
    processor.setSetting("filamentDiameter", "2890");
    processor.setSetting("filamentFlow", "100");
    processor.setSetting("layer0extrusionWidth", "600");
    processor.setSetting("extrusionWidth", "400");
    processor.setSetting("insetCount", "2");
    processor.setSetting("downSkinCount", "6");
    processor.setSetting("upSkinCount", "6");
    processor.setSetting("skinPattern", "SKIN_LINES");
    processor.setSetting("skirtDistance", "6000");
    processor.setSetting("skirtLineCount", "1");
    processor.setSetting("skirtMinLength", "0");

    processor.setSetting("initialSpeedupLayers", "4");
    processor.setSetting("initialLayerSpeed", "20");
    processor.setSetting("skirtSpeed", "50");
    processor.setSetting("inset0Speed", "50");
    processor.setSetting("insetXSpeed", "50");
    processor.setSetting("supportSpeed", "50");
    processor.setSetting("moveSpeed", "150");
    processor.setSetting("fanFullOnLayerNr", "2");

    processor.setSetting("sparseInfillLineDistance", "100 * extrusionWidth / 20");
    processor.setSetting("sparseInfillCombineCount", "1");
    processor.setSetting("infillOverlap", "15");
    processor.setSetting("infillSpeed", "50");
    processor.setSetting("infillPattern", "INFILL_GRID");

    processor.setSetting("supportType", "SUPPORT_TYPE_GRID");
    processor.setSetting("supportAngle", "-1");
    processor.setSetting("supportEverywhere", "0");
    processor.setSetting("supportLineDistance", "sparseInfillLineDistance");
    processor.setSetting("supportXYDistance", "700");
    processor.setSetting("supportZDistance", "150");
    processor.setSetting("supportExtruder", "-1");

    processor.setSetting("retractionAmount", "4500");
    processor.setSetting("retractionPrimeAmount", "0");
    processor.setSetting("retractionSpeed", "25");
    processor.setSetting("retractionPrimeSpeed", "25");
    processor.setSetting("retractionAmountExtruderSwitch", "14500");
    processor.setSetting("retractionExtruderSwitchSpeed", "25");
    processor.setSetting("retractionExtruderSwitchPrimeSpeed", "25");
    processor.setSetting("retractionMinimalDistance", "1500");
    processor.setSetting("minimalExtrusionBeforeRetraction", "100");
    processor.setSetting("retractionZHop", "0");

    processor.setSetting("enableCombing", "1");
    processor.setSetting("enableOozeShield", "0");
    processor.setSetting("wipeTowerSize", "0");
    processor.setSetting("multiVolumeOverlap", "0");
    processor.setSetting("position.X", "102500");
    processor.setSetting("position.Y", "102500");
    processor.setSetting("position.Z", "0");
    processor.setSetting("objectSink", "0");
    processor.setSetting("autoCenter", "1");
    processor.setSetting("extruderNr", "0");

    processor.setSetting("raftMargin", "5000");
    processor.setSetting("raftLineSpacing", "1000");
    processor.setSetting("raftBaseThickness", "0");
    processor.setSetting("raftBaseLinewidth", "0");
    processor.setSetting("raftInterfaceThickness", "0");
    processor.setSetting("raftInterfaceLinewidth", "0");
    processor.setSetting("raftInterfaceLineSpacing", "0");
    processor.setSetting("raftAirGap", "0");
    processor.setSetting("raftAirGapLayer0", "0");
    processor.setSetting("raftBaseSpeed", "15");
    processor.setSetting("raftInterfaceSpeed", "15");
    processor.setSetting("raftFanSpeed", "0");
    processor.setSetting("raftSurfaceThickness", "0");
    processor.setSetting("raftSurfaceLinewidth", "0");
    processor.setSetting("raftSurfaceLineSpacing", "0");
    processor.setSetting("raftSurfaceLayers", "0");
    processor.setSetting("raftSurfaceSpeed", "0");

    processor.setSetting("minimalLayerTime", "5");
    processor.setSetting("minimalFeedrate", "10");
    processor.setSetting("coolHeadLift", "0");
    processor.setSetting("fanSpeedMin", "100");
    processor.setSetting("fanSpeedMax", "100");

    processor.setSetting("fixHorrible", "0");
    processor.setSetting("spiralizeMode", "0");
    processor.setSetting("simpleMode", "0");
    processor.setSetting("gcodeFlavor", "GCODE_FLAVOR_REPRAP");

    for(int n=0; n<MAX_EXTRUDERS; n++)
    {
        std::ostringstream stream;
        stream << "extruderOffset" << n;
        processor.setSetting(stream.str() + ".X", "0");
        processor.setSetting(stream.str() + ".Y", "0");
    }

    processor.setSetting("startCode",
        "M109 S210     ;Heatup to 210C\n"
        "G21           ;metric values\n"
        "G90           ;absolute positioning\n"
        "G28           ;Home\n"
        "G1 Z15.0 F300 ;move the platform down 15mm\n"
        "G92 E0        ;zero the extruded length\n"
        "G1 F200 E5    ;extrude 5mm of feed stock\n"
        "G92 E0        ;zero the extruded length again\n");
    processor.setSetting("endCode",
        "M104 S0                     ;extruder heater off\n"
        "M140 S0                     ;heated bed heater off (if you have it)\n"
        "G91                            ;relative positioning\n"
        "G1 E-1 F300                    ;retract the filament a bit before lifting the nozzle, to release some of the pressure\n"
        "G1 Z+0.5 E-5 X-20 Y-20 F9000   ;move Z up a bit and retract filament even more\n"
        "G28 X0 Y0                      ;move X/Y to min endstops, so the head is out of the way\n"
        "M84                         ;steppers off\n"
        "G90                         ;absolute positioning\n");
    processor.setSetting("postSwitchExtruderCode", "");
    processor.setSetting("preSwitchExtruderCode", "");
//*/
    CommandSocket* commandSocket = NULL;

    for(int argn = 1; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            if (str[1] == '-')
            {
                //Long options
                if (stringcasecompare(str, "--socket") == 0)
                {
                    argn++;
                    commandSocket = new CommandSocket(atoi(argv[argn]));
                    processor.setCommandSocket(commandSocket);
                }else if (stringcasecompare(str, "--command-socket") == 0)
                {
                    commandSocket->handleIncommingData(&processor);
                }else if (stringcasecompare(str, "--") == 0)
                {
                    try {
                        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
                        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
                        if (files.size() > 0)
                            processor.processFiles(files);
                        files.clear();
                    }catch(...){
                        cura::logError("Unknown exception\n");
                        exit(1);
                    }
                    break;
                }else{
                    cura::logError("Unknown option: %s\n", str);
                }
            }else{
                for(str++; *str; str++)
                {
                    switch(*str)
                    {
                    case 'h':
                        print_usage();
                        exit(1);
                    case 'v':
                        cura::increaseVerboseLevel();
                        break;
                    case 'p':
                        cura::enableProgressLogging();
                        break;
                    case 'o':
                        argn++;
                        if (!processor.setTargetFile(argv[argn]))
                        {
                            cura::logError("Failed to open %s for output.\n", argv[argn]);
                            exit(1);
                        }
                        break;
                    case 's':
                        {
                            //Parse the given setting and store it.
                            argn++;
                            char* valuePtr = strchr(argv[argn], '=');
                            if (valuePtr)
                            {
                                *valuePtr++ = '\0';

                                processor.setSetting(argv[argn], valuePtr);
                            }
                        }
                        break;
                    default:
                        cura::logError("Unknown option: %c\n", *str);
                        break;
                    }
                }
            }
        }else{
            files.push_back(argv[argn]);
        }
    }
    try {
        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
        if (files.size() > 0)
            processor.processFiles(files);
    }catch(...){
        cura::logError("Unknown exception\n");
        exit(1);
    }
    //Finalize the processor, this adds the end.gcode. And reports statistics.
    processor.finalize();


    el::Loggers::flushAll();
    return 0;
}
