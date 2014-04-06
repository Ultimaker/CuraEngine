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
    logError("usage: CuraEngine [-h] [-v] [-m 3x3matrix] [-c <config file>] [-s <settingkey>=<value>] -o <output.gcode> <model.stl>\n");
}

//Signal handler for a "floating point exception", which can also be integer division by zero errors.
void signal_FPE(int n)
{
    (void)n;
    logError("Arithmetic exception.\n");
    exit(1);
}

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac. On windows this is done on process creation from the GUI.
    setpriority(PRIO_PROCESS, 0, 10);
#endif

    //Register the exception handling for arithmic exceptions, this prevents the "something went wrong" dialog on windows to pop up on a division by zero.
    signal(SIGFPE, signal_FPE);

    ConfigSettings config;
    fffProcessor processor(config);

    logError("Cura_SteamEngine version %s\n", VERSION);
    logError("Copyright (C) 2014 David Braam\n");
    logError("\n");
    logError("This program is free software: you can redistribute it and/or modify\n");
    logError("it under the terms of the GNU Affero General Public License as published by\n");
    logError("the Free Software Foundation, either version 3 of the License, or\n");
    logError("(at your option) any later version.\n");
    logError("\n");
    logError("This program is distributed in the hope that it will be useful,\n");
    logError("but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
    logError("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
    logError("GNU Affero General Public License for more details.\n");
    logError("\n");
    logError("You should have received a copy of the GNU Affero General Public License\n");
    logError("along with this program.  If not, see <http://www.gnu.org/licenses/>.\n");

    if(!config.readSettings()) {
        logError("Default config '%s' not used\n", DEFAULT_CONFIG_PATH);
    }
    for(int argn = 1; argn < argc; argn++)
        log("Arg: %s\n", argv[argn]);
    
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
                    increaseVerboseLevel();
                    break;
                case 'p':
                    enableProgressLogging();
                    break;
                case 'g':
                    argn++;
                    //Connect the GUI socket to the given port number.
                    processor.guiConnect(atoi(argv[argn]));
                    break;
                case 'b':
                    argn++;
                    //The binaryMeshBlob is depricated and will be removed in the future.
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
                case 'c':
                    {
                        // Read a config file from the given path
                        argn++;
                        if(!config.readSettings(argv[argn])) {
                            logError("Failed to read config '%s'\n", argv[argn]);
                        }
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

                            if (!config.setSetting(argv[argn], valuePtr))
                                logError("Setting not found: %s %s\n", argv[argn], valuePtr);
                        }
                    }
                    break;
                case 'm':
                    //Read the given rotation/scale matrix
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
                //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
                // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
                processor.processFile(argv[argn]);
            }catch(...){
                logError("Unknown exception\n");
                exit(1);
            }
        }
    }

    //Finalize the processor, this adds the end.gcode. And reports statistics.
    processor.finalize();
}
