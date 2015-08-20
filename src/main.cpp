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

#include "fffProcessor.h"
#include "settingRegistry.h"

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


void connect(fffProcessor& processor, int argc, char **argv)
{
    CommandSocket* commandSocket = new CommandSocket(&processor);
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
                    if (SettingRegistry::getInstance()->loadJSON(argv[argn]))
                    {
                        cura::logError("ERROR: Failed to load json file: %s\n", argv[argn]);
                    }
                    break;
                default:
                    cura::logError("Unknown option: %c\n", *str);
                    break;
                }
            }
        }
    }
    
    commandSocket->connect(ip, port);
}

void slice(fffProcessor& processor, int argc, char **argv)
{   
    processor.time_keeper.restart();
    
    FMatrix3x3 transformation; // the transformation applied to a model when loaded
                        
    MeshGroup meshgroup(&processor);
    
    int extruder_train_nr = 0;

    SettingsBase* last_extruder_train = meshgroup.getExtruderTrain(0);
    SettingsBase* last_settings_object = &processor;
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
                        meshgroup.finalize();
                        log("Loaded from disk in %5.3fs\n", processor.time_keeper.restart());
                        
                        for (int extruder_nr = 0; extruder_nr < processor.getSettingAsCount("machine_extruder_count"); extruder_nr++)
                        { // initialize remaining extruder trains and load the defaults
                            meshgroup.getExtruderTrain(extruder_nr)->setExtruderTrainDefaults(extruder_nr); // also initializes yet uninitialized extruder trains
                        }
                        //start slicing
                        processor.processMeshGroup(&meshgroup);
                        
                        // initialize loading of new meshes
                        processor.time_keeper.restart();
                        meshgroup = MeshGroup(&processor);
                        last_settings_object = &meshgroup;
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
                    case 'v':
                        cura::increaseVerboseLevel();
                        break;
                    case 'p':
                        cura::enableProgressLogging();
                        break;
                    case 'j':
                        argn++;
                        if (SettingRegistry::getInstance()->loadJSON(argv[argn]))
                        {
                            cura::logError("ERROR: Failed to load json file: %s\n", argv[argn]);
                        }
                        break;
                    case 'e':
                        str++;
                        extruder_train_nr = int(*str - '0'); // TODO: parse int instead (now "-e10"="-e:" , "-e11"="-e;" , "-e12"="-e<" .. etc) 
                        last_settings_object = meshgroup.getExtruderTrain(extruder_train_nr);
                        last_extruder_train = meshgroup.getExtruderTrain(extruder_train_nr);
                        break;
                    case 'l':
                        argn++;
                        
                        log("Loading %s from disk...\n", argv[argn]);
                        // transformation = // TODO: get a transformation from somewhere
                        
                        if (!loadMeshIntoMeshGroup(&meshgroup, argv[argn], transformation, last_extruder_train))
                        {
                            logError("Failed to load model: %s\n", argv[argn]);
                        }
                        else 
                        {
                            last_settings_object = &(meshgroup.meshes.back()); // pointer is valid until a new object is added, so this is OK
                        }
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

                                last_settings_object->setSetting(argv[argn], valuePtr);
                            }
                        }
                        break;
                    default:
                        cura::logError("Unknown option: %c\n", *str);
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
            print_usage();
            exit(1);
        }
    }

    for (extruder_train_nr = 0; extruder_train_nr < processor.getSettingAsCount("machine_extruder_count"); extruder_train_nr++)
    { // initialize remaining extruder trains and load the defaults
        meshgroup.getExtruderTrain(extruder_train_nr)->setExtruderTrainDefaults(extruder_train_nr); // also initializes yet uninitialized extruder trains
    }
    
    
#ifndef DEBUG
    try {
#endif
        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
        meshgroup.finalize();
        log("Loaded from disk in %5.3fs\n", processor.time_keeper.restart());
        
        //start slicing
        processor.processMeshGroup(&meshgroup);
        
#ifndef DEBUG
    }catch(...){
        cura::logError("Unknown exception\n");
        exit(1);
    }
#endif
    //Finalize the processor, this adds the end.gcode. And reports statistics.
    processor.finalize();
    
}

}//namespace cura

using namespace cura;

#include "utils/AABB.h"
#include "utils/SVG.h"
int main() 
{
    
    Polygons outline;
    {
        {
            PolygonRef poly = outline.newPoly();
//             poly.emplace_back(139588, 109695);
//             poly.emplace_back(139897, 109759);
//             poly.emplace_back(140194, 109862);
//             poly.emplace_back(140476, 110003);
//             poly.emplace_back(140739, 110180);
//             poly.emplace_back(140976, 110389);
//             poly.emplace_back(141185, 110628);
//             poly.emplace_back(141360, 110892);
//             poly.emplace_back(141500, 111177);
//             poly.emplace_back(141602, 111478);
//             poly.emplace_back(141663, 111789);
            poly.emplace_back(141684, 112106);
            poly.emplace_back(141683, 112310);
            poly.emplace_back(141684, 112424);
            poly.emplace_back(141683, 112627);
            poly.emplace_back(141684, 112741);
            poly.emplace_back(141683, 112944);
            poly.emplace_back(141684, 113059);
            poly.emplace_back(141683, 113262);
            poly.emplace_back(141684, 113377);
            poly.emplace_back(141683, 113580);
            poly.emplace_back(141684, 113694);
            poly.emplace_back(141683, 113898);
            poly.emplace_back(141684, 114012);
            poly.emplace_back(141683, 114216);
            poly.emplace_back(141684, 114330);
            poly.emplace_back(141683, 114533);
            poly.emplace_back(141684, 114648);
            poly.emplace_back(141683, 114851);
            poly.emplace_back(141684, 114966);
            poly.emplace_back(141683, 115169);
            poly.emplace_back(141684, 115284);
            poly.emplace_back(141683, 115487);
            poly.emplace_back(141684, 115601);
            poly.emplace_back(141683, 115805);
            poly.emplace_back(141684, 115919);
            poly.emplace_back(141683, 116123);
            poly.emplace_back(141684, 116237);
            poly.emplace_back(141683, 116441);
            poly.emplace_back(141684, 116555);
            poly.emplace_back(141683, 116759);
            poly.emplace_back(141684, 116873);
            poly.emplace_back(141683, 117077);
            poly.emplace_back(141684, 117191);
            poly.emplace_back(141683, 117395);
            poly.emplace_back(141684, 117509);
            poly.emplace_back(141683, 117712);
            poly.emplace_back(141684, 117826);
            poly.emplace_back(141683, 118030);
            poly.emplace_back(141684, 118144);
            poly.emplace_back(141683, 118348);
            poly.emplace_back(141684, 118462);
            poly.emplace_back(141683, 118666);
            poly.emplace_back(141684, 118780);
            poly.emplace_back(141683, 118984);
            poly.emplace_back(141684, 119098);
            poly.emplace_back(141683, 119302);
            poly.emplace_back(141684, 119416);
            poly.emplace_back(141683, 119619);
            poly.emplace_back(141684, 119733);
            poly.emplace_back(141683, 119937);
            poly.emplace_back(141684, 120051);
            poly.emplace_back(141683, 120255);
            poly.emplace_back(141684, 120369);
            poly.emplace_back(141683, 120573);
            poly.emplace_back(141684, 120686);
            poly.emplace_back(141683, 120891);
            poly.emplace_back(141684, 121004);
            poly.emplace_back(141683, 121209);
            poly.emplace_back(141684, 121322);
            poly.emplace_back(141683, 121526);
            poly.emplace_back(141684, 121640);
            poly.emplace_back(141683, 121844);
            poly.emplace_back(141684, 121958);
            poly.emplace_back(141683, 122162);
            poly.emplace_back(141684, 122276);
            poly.emplace_back(141683, 122480);
            poly.emplace_back(141684, 122593);
            poly.emplace_back(141683, 122798);
            poly.emplace_back(141684, 122911);
            poly.emplace_back(141683, 123116);
            poly.emplace_back(141684, 123229);
            poly.emplace_back(141683, 123433);
            poly.emplace_back(141684, 123547);
//             poly.emplace_back(141667, 123864);
//             poly.emplace_back(141613, 124176);
//             poly.emplace_back(141525, 124480);
//             poly.emplace_back(141402, 124770);
//             poly.emplace_back(141248, 125044);
//             poly.emplace_back(141064, 125295);
//             poly.emplace_back(140854, 125521);
//             poly.emplace_back(140620, 125720);
//             poly.emplace_back(140367, 125888);
//             poly.emplace_back(140098, 126024);
//             poly.emplace_back(139815, 126124);
//             poly.emplace_back(139521, 126187);
//             poly.emplace_back(139219, 126213);
//             poly.emplace_back(138916, 126200);
//             poly.emplace_back(138419, 126116);
//             poly.emplace_back(137800, 126022);
//             poly.emplace_back(137386, 125918);
//             poly.emplace_back(137096, 125792);
//             poly.emplace_back(136824, 125631);
//             poly.emplace_back(136574, 125436);
//             poly.emplace_back(136352, 125210);
//             poly.emplace_back(136162, 124955);
//             poly.emplace_back(136007, 124678);
//             poly.emplace_back(135889, 124384);
//             poly.emplace_back(135809, 124076);
//             poly.emplace_back(135770, 123761);
//             poly.emplace_back(135769, 123557);
//             poly.emplace_back(135770, 123443);
//             poly.emplace_back(135807, 123128);
//             poly.emplace_back(135932, 122624);
//             poly.emplace_back(136087, 122010);
//             poly.emplace_back(136219, 121389);
//             poly.emplace_back(136329, 120764);
//             poly.emplace_back(136416, 120135);
//             poly.emplace_back(136478, 119503);
//             poly.emplace_back(136518, 118869);
//             poly.emplace_back(136534, 118233);
//             poly.emplace_back(136536, 118119);
//             poly.emplace_back(136527, 117597);
//             poly.emplace_back(136496, 116963);
//             poly.emplace_back(136442, 116330);
//             poly.emplace_back(136365, 115699);
            poly.emplace_back(136264, 115073);
//             poly.emplace_back(136139, 114450);
//             poly.emplace_back(135993, 113834);
//             poly.emplace_back(135834, 113220);
//             poly.emplace_back(135772, 112795);
//             poly.emplace_back(135771, 112592);
//             poly.emplace_back(135772, 112478);
//             poly.emplace_back(135850, 111965);
//             poly.emplace_back(135876, 111854);
//             poly.emplace_back(135981, 111556);
//             poly.emplace_back(136119, 111274);
//             poly.emplace_back(136288, 111011);
//             poly.emplace_back(136486, 110773);
//             poly.emplace_back(136709, 110560);
//             poly.emplace_back(137213, 110222);
//             poly.emplace_back(137488, 110104);
//             poly.emplace_back(138066, 109935);
//             poly.emplace_back(138550, 109782);
//             poly.emplace_back(138656, 109747);
//             poly.emplace_back(138963, 109690);
//             poly.emplace_back(139275, 109673);
            
            
//             poly.simplify(100);
        }
    }
    
    {
        PolygonRef poly = outline.newPoly();
        poly.emplace_back(Point(140000,110000));
        poly.emplace_back(Point(141000,110000));
        poly.emplace_back(Point(142000,110000));
        poly.emplace_back(Point(143000,110000));
    }
    
    {
        PolygonRef poly = outline.newPoly();
        poly.emplace_back(Point(142000,110000));
        poly.emplace_back(Point(143000,110000));
        poly.emplace_back(Point(143000,115000));
        poly.emplace_back(Point(140000,110000));
        poly.emplace_back(Point(141000,110000));
    }
    
    {
        PolygonRef poly = outline.newPoly();
        poly.emplace_back(Point(140000,100000));
        poly.emplace_back(Point(141000,100000));
    }
    {
        PolygonRef poly = outline.newPoly();
        poly.emplace_back(Point(136000,110000));
        poly.emplace_back(Point(136000,120000));
        poly.emplace_back(Point(136005,120000));
        poly.emplace_back(Point(136010,120000));
        poly.emplace_back(Point(136015,120000));
        poly.emplace_back(Point(136020,120000));
        poly.emplace_back(Point(136025,120000));
        poly.emplace_back(Point(136030,120000));
        poly.emplace_back(Point(136035,120000));
        poly.emplace_back(Point(136040,120000));
        poly.emplace_back(Point(136045,120000));
        poly.emplace_back(Point(136050,120000));
        poly.emplace_back(Point(136055,120000));
        poly.emplace_back(Point(136060,120000));
        poly.emplace_back(Point(136065,120000));
        poly.emplace_back(Point(136070,120000));
        poly.emplace_back(Point(136075,120000));
        poly.emplace_back(Point(136080,120000));
        poly.emplace_back(Point(136085,120000));
        poly.emplace_back(Point(136090,120000));
        poly.emplace_back(Point(136095,120000));
        poly.emplace_back(Point(136100,120000));
        poly.emplace_back(Point(137000,120000));
    }
    
//     optimizePolygons(outline);
    
    outline.simplify(10);
    
//     outline = outline.smooth(10, 1000);
    
    SVG svg("output/layer158.html", AABB(outline));
    svg.writeAreas(outline);
    svg.writePoints(outline);
}

int main_(int argc, char **argv)
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
    
    fffProcessor processor;

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
        connect(processor, argc, argv);
    } 
    else if (stringcasecompare(argv[1], "slice") == 0)
    {
        slice(processor, argc, argv);
    }
    else
    {
        cura::logError("Unknown command: %s\n", argv[1]);
        print_usage();
        exit(1);
    }
    
    return 0;
}