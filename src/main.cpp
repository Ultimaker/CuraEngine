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


    CommandSocket* commandSocket = NULL;
    std::string ip;
    int port = 49674;

    for(int argn = 1; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            if (str[1] == '-')
            {
                if (stringcasecompare(str, "--connect") == 0)
                {
                    commandSocket = new CommandSocket(&processor);

                    std::string ip_port(argv[argn + 1]);
                    if (ip_port.find(':') != std::string::npos)
                    {
                        ip = ip_port.substr(0, ip_port.find(':'));
                        port = std::stoi(ip_port.substr(ip_port.find(':') + 1).data());
                    }

                    argn += 1;
                }
                else if (stringcasecompare(str, "--") == 0)
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
    
    if(commandSocket)
    {
        commandSocket->connect(ip, port);
    }
    else
    {
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
    }

    return 0;
}
