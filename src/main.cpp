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

#include "FffProcessor.h"
#include "settingRegistry.h"

#include "SettingsToGV.h"

namespace cura
{
    
void print_usage()
{
    cura::logError("\n");
    cura::logError("usage:\n");
    cura::logError("CuraEngine help\n");
    cura::logError("\tShow this help message\n");
    cura::logError("\n");
    cura::logError("CuraEngine connect <host>[:<port>] [-j <settings.json>]\n");
    cura::logError("  --connect <host>[:<port>]\n\tConnect to <host> via a command socket, \n\tinstead of passing information via the command line\n");
    cura::logError("  -j\n\tLoad settings.json file to register all settings and their defaults\n");
    cura::logError("\n");
    cura::logError("CuraEngine slice [-v] [-p] [-j <settings.json>] [-s <settingkey>=<value>] [-g] [-e] [-o <output.gcode>] [-l <model.stl>] [--next]\n");
    cura::logError("  -v\n\tIncrease the verbose level (show log messages).\n");
    cura::logError("  -p\n\tLog progress information.\n");
    cura::logError("  -j\n\tLoad settings.json file to register all settings and their defaults.\n");
    cura::logError("  -s <setting>=<value>\n\tSet a setting to a value for the last supplied object, \n\textruder train, or general settings.\n");
    cura::logError("  -l <model_file>\n\tLoad an STL model. \n");
    cura::logError("  -g\n\tSwitch setting focus to the current mesh group only.\n\tUsed for one-at-a-time printing.\n");
    cura::logError("  -e\n\tAdd a new extruder train.\n");
    cura::logError("  --next\n\tGenerate gcode for the previously supplied mesh group and append that to \n\tthe gcode of further models for one-at-a-time printing.\n");
    cura::logError("  -o <output_file>\n\tSpecify a file to which to write the generated gcode.\n");
    cura::logError("\n");
    cura::logError("The settings are appended to the last supplied object:\n");
    cura::logError("CuraEngine slice [general settings] \n\t-g [current group settings] \n\t-e [extruder train settings] \n\t-l obj_inheriting_from_last_extruder_train.stl [object settings] \n\t--next [next group settings]\n\t... etc.\n");
    cura::logError("\n");
}

//Signal handler for a "floating point exception", which can also be integer division by zero errors.
void signal_FPE(int n)
{
    (void)n;
    cura::logError("Arithmetic exception.\n");
    exit(1);
}

void print_call(int argc, char **argv)
{
    cura::logError("Command called:\n");
    for (int idx= 0; idx < argc; idx++)
        cura::logError("%s ", argv[idx]);
    cura::logError("\n");
}

void connect(int argc, char **argv)
{
    CommandSocket::instantiate();
    std::string ip;
    int port = 49674;
    
    std::string ip_port(argv[2]);
    if (ip_port.find(':') != std::string::npos)
    {
        ip = ip_port.substr(0, ip_port.find(':'));
        port = std::stoi(ip_port.substr(ip_port.find(':') + 1).data());
    }

    
    for(int argn = 3; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            for(str++; *str; str++)
            {
                switch(*str)
                {
                case 'v':
                    cura::increaseVerboseLevel();
                    break;
                case 'j':
                    argn++;
                    if (SettingRegistry::getInstance()->loadJSONsettings(argv[argn]))
                    {
                        cura::logError("ERROR: Failed to load json file: %s\n", argv[argn]);
                    }
                    break;
                default:
                    cura::logError("Unknown option: %c\n", *str);
                    print_call(argc, argv);
                    print_usage();
                    break;
                }
            }
        }
    }
    
    CommandSocket::getInstance()->connect(ip, port);
}

void slice(int argc, char **argv)
{   
    FffProcessor::getInstance()->time_keeper.restart();
    
    FMatrix3x3 transformation; // the transformation applied to a model when loaded
                        
    MeshGroup* meshgroup = new MeshGroup(FffProcessor::getInstance());
    
    int extruder_train_nr = 0;

    SettingsBase* last_extruder_train = meshgroup->createExtruderTrain(0); 
    SettingsBase* last_settings_object = FffProcessor::getInstance();
    for(int argn = 2; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            if (str[1] == '-')
            {
                if (stringcasecompare(str, "--next") == 0)
                {
                    try {
                        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
                        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
                        meshgroup->finalize();
                        log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());
                        
                        for (int extruder_nr = 0; extruder_nr < FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count"); extruder_nr++)
                        { // initialize remaining extruder trains and load the defaults
                            meshgroup->createExtruderTrain(extruder_nr)->setExtruderTrainDefaults(extruder_nr); // create new extruder train objects or use already existing ones
                        }
                        //start slicing
                        FffProcessor::getInstance()->processMeshGroup(meshgroup);
                        
                        // initialize loading of new meshes
                        FffProcessor::getInstance()->time_keeper.restart();
                        delete meshgroup;
                        meshgroup = new MeshGroup(FffProcessor::getInstance());
                        last_extruder_train = meshgroup->createExtruderTrain(0); 
                        last_settings_object = meshgroup;
                        
                    }catch(...){
                        cura::logError("Unknown exception\n");
                        exit(1);
                    }
                }else{
                    cura::logError("Unknown option: %s\n", str);
                }
            }else{
                for(str++; *str; str++)
                {
                    switch(*str)
                    {
                    case 'v':
                        cura::increaseVerboseLevel();
                        break;
                    case 'p':
                        cura::enableProgressLogging();
                        break;
                    case 'j':
                        argn++;
                        if (SettingRegistry::getInstance()->loadJSONsettings(argv[argn]))
                        {
                            cura::logError("ERROR: Failed to load json file: %s\n", argv[argn]);
                        }
                        break;
                    case 'e':
                        str++;
                        extruder_train_nr = int(*str - '0'); // TODO: parse int instead (now "-e10"="-e:" , "-e11"="-e;" , "-e12"="-e<" .. etc) 
                        last_settings_object = meshgroup->createExtruderTrain(extruder_train_nr);
                        last_extruder_train = last_settings_object;
                        break;
                    case 'l':
                        argn++;
                        
                        log("Loading %s from disk...\n", argv[argn]);
                        // transformation = // TODO: get a transformation from somewhere
                        
                        if (!loadMeshIntoMeshGroup(meshgroup, argv[argn], transformation, last_extruder_train))
                        {
                            logError("Failed to load model: %s\n", argv[argn]);
                        }
                        else 
                        {
                            last_settings_object = &(meshgroup->meshes.back()); // pointer is valid until a new object is added, so this is OK
                        }
                        break;
                    case 'o':
                        argn++;
                        if (!FffProcessor::getInstance()->setTargetFile(argv[argn]))
                        {
                            cura::logError("Failed to open %s for output.\n", argv[argn]);
                            exit(1);
                        }
                        break;
                    case 'g':
                        last_settings_object = meshgroup;
                    case 's':
                        {
                            //Parse the given setting and store it.
                            argn++;
                            char* valuePtr = strchr(argv[argn], '=');
                            if (valuePtr)
                            {
                                *valuePtr++ = '\0';

                                last_settings_object->setSetting(argv[argn], valuePtr);
                            }
                        }
                        break;
                    default:
                        cura::logError("Unknown option: %c\n", *str);
                        print_call(argc, argv);
                        print_usage();
                        exit(1);
                        break;
                    }
                }
            }
        }
        else
        {
            
            cura::logError("Unknown option: %s\n", argv[argn]);
            print_call(argc, argv);
            print_usage();
            exit(1);
        }
    }

    int extruder_count = FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count");
    for (extruder_train_nr = 0; extruder_train_nr < extruder_count; extruder_train_nr++)
    { // initialize remaining extruder trains and load the defaults
        meshgroup->createExtruderTrain(extruder_train_nr)->setExtruderTrainDefaults(extruder_train_nr); // create new extruder train objects or use already existing ones
    }
    
    
#ifndef DEBUG
    try {
#endif
        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
        meshgroup->finalize();
        log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());
        
        //start slicing
        FffProcessor::getInstance()->processMeshGroup(meshgroup);

#ifndef DEBUG
    }catch(...){
        cura::logError("Unknown exception\n");
        exit(1);
    }
#endif
    //Finalize the processor, this adds the end.gcode. And reports statistics.
    FffProcessor::getInstance()->finalize();

    delete meshgroup;
}

}//namespace cura

using namespace cura;

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac. On windows this is done on process creation from the GUI.
    setpriority(PRIO_PROCESS, 0, 10);
#endif

#ifndef DEBUG
    //Register the exception handling for arithmic exceptions, this prevents the "something went wrong" dialog on windows to pop up on a division by zero.
    signal(SIGFPE, signal_FPE);
#endif

    Progress::init();
    
    
    logCopyright("\n");
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


    if (argc < 2)
    {
        print_usage();
        exit(1);
    }
    
    if (stringcasecompare(argv[1], "connect") == 0)
    {
        connect(argc, argv);
    } 
    else if (stringcasecompare(argv[1], "slice") == 0)
    {
        slice(argc, argv);
    }
    else if (stringcasecompare(argv[1], "help") == 0)
    {
        print_usage();
        exit(0);
    }
    else if (stringcasecompare(argv[1], "analyse") == 0)
    { // CuraEngine analyse [json] [output.gv] [engine_settings] -[p|i|e|w]
        // p = show parent-child relations
        // i = show inheritance function
        // e = show error functions
        // w = show warning functions
        // dot refl_ff.gv -Tpng > rafl_ff_dotted.png
        // see meta/HOWTO.txt
        
        bool parent_child_viz = false;
        bool inherit_viz = false;
        bool warning_viz = false;
        bool error_viz = false;
        if (argc >= 5)
        {
            char* str = argv[5];
            if (str[0] == '-')
            {
                for(str++; *str; str++)
                {
                    switch(*str)
                    {
                    case 'p':
                        parent_child_viz = true;
                        break;
                    case 'i':
                        inherit_viz = true;
                        break;
                    case 'e':
                        error_viz = true;
                        break;
                    case 'w':
                        warning_viz = true;
                        break;
                    default:
                        cura::logError("Unknown option: %c\n", *str);
                        print_call(argc, argv);
                        print_usage();
                        break;
                    }
                }
            }
        }
        
        SettingsToGv gv_out(argv[3], argv[4], parent_child_viz, inherit_viz, error_viz, warning_viz);
        if (gv_out.generate(std::string(argv[2])))
        {
            cura::logError("ERROR: Failed to analyse json file: %s\n", argv[2]);
        }
        exit(0);
    }
    else
    {
        cura::logError("Unknown command: %s\n", argv[1]);
        print_call(argc, argv);
        print_usage();
        exit(1);
    }
    
    return 0;
}