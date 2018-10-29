//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifdef _OPENMP
    #include <omp.h> // omp_get_num_threads
#endif // _OPENMP
#include <string>
#include "Application.h"
#include "FffProcessor.h"
#include "communication/ArcusCommunication.h" //To connect via Arcus to the front-end.
#include "communication/CommandLine.h" //To use the command line to slice stuff.
#include "progress/Progress.h"
#include "utils/logoutput.h"
#include "utils/string.h" //For stringcasecompare.

namespace cura
{

Application::Application()
: communication(nullptr)
, current_slice(0)
{
}

Application::~Application()
{
    delete communication;
}

Application& Application::getInstance()
{
    static Application instance; //Constructs using the default constructor.
    return instance;
}

#ifdef ARCUS
void Application::connect()
{
    std::string ip = "127.0.0.1";
    int port = 49674;

    //Parse port number from IP address.
    std::string ip_port(argv[2]);
    std::size_t found_pos = ip_port.find(':');
    if (found_pos != std::string::npos)
    {
        ip = ip_port.substr(0, found_pos);
        port = std::stoi(ip_port.substr(found_pos + 1).data());
    }

#ifdef _OPENMP
    int n_threads;
#endif // _OPENMP

    for(size_t argn = 3; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            for(str++; *str; str++)
            {
                switch(*str)
                {
                case 'v':
                    increaseVerboseLevel();
                    break;
#ifdef _OPENMP
                case 'm':
                    str++;
                    n_threads = std::strtol(str, &str, 10);
                    str--;
                    n_threads = std::max(1, n_threads);
                    omp_set_num_threads(n_threads);
                    break;
#endif // _OPENMP
                default:
                    logError("Unknown option: %c\n", *str);
                    printCall();
                    printHelp();
                    break;
                }
            }
        }
    }

    ArcusCommunication* arcus_communication = new ArcusCommunication();
    arcus_communication->connect(ip, port);
    communication = arcus_communication;
}
#endif //ARCUS

void Application::printCall() const
{
    cura::logError("Command called:\n");
    for (size_t argument_index = 0; argument_index < argc; argument_index++)
    {
        cura::logError("%s ", argv[argument_index]);
    }
    cura::logError("\n");
}

void Application::printHelp() const
{
    logAlways("\n");
    logAlways("usage:\n");
    logAlways("CuraEngine help\n");
    logAlways("\tShow this help message\n");
    logAlways("\n");
#ifdef ARCUS
    logAlways("CuraEngine connect <host>[:<port>] [-j <settings.def.json>]\n");
    logAlways("  --connect <host>[:<port>]\n\tConnect to <host> via a command socket, \n\tinstead of passing information via the command line\n");
    logAlways("  -v\n\tIncrease the verbose level (show log messages).\n");
#ifdef _OPENMP
    logAlways("  -m<thread_count>\n\tSet the desired number of threads. Supports only a single digit.\n");
#endif // _OPENMP
    logAlways("\n");
#endif //ARCUS
    logAlways("CuraEngine slice [-v] [-p] [-j <settings.json>] [-s <settingkey>=<value>] [-g] [-e<extruder_nr>] [-o <output.gcode>] [-l <model.stl>] [--next]\n");
    logAlways("  -v\n\tIncrease the verbose level (show log messages).\n");
#ifdef _OPENMP
    logAlways("  -m<thread_count>\n\tSet the desired number of threads.\n");
#endif // _OPENMP
    logAlways("  -p\n\tLog progress information.\n");
    logAlways("  -j\n\tLoad settings.def.json file to register all settings and their defaults.\n");
    logAlways("  -s <setting>=<value>\n\tSet a setting to a value for the last supplied object, \n\textruder train, or general settings.\n");
    logAlways("  -l <model_file>\n\tLoad an STL model. \n");
    logAlways("  -g\n\tSwitch setting focus to the current mesh group only.\n\tUsed for one-at-a-time printing.\n");
    logAlways("  -e<extruder_nr>\n\tSwitch setting focus to the extruder train with the given number.\n");
    logAlways("  --next\n\tGenerate gcode for the previously supplied mesh group and append that to \n\tthe gcode of further models for one-at-a-time printing.\n");
    logAlways("  -o <output_file>\n\tSpecify a file to which to write the generated gcode.\n");
    logAlways("\n");
    logAlways("The settings are appended to the last supplied object:\n");
    logAlways("CuraEngine slice [general settings] \n\t-g [current group settings] \n\t-e0 [extruder train 0 settings] \n\t-l obj_inheriting_from_last_extruder_train.stl [object settings] \n\t--next [next group settings]\n\t... etc.\n");
    logAlways("\n");
    logAlways("In order to load machine definitions from custom locations, you need to create the environment variable CURA_ENGINE_SEARCH_PATH, which should contain all search paths delimited by a (semi-)colon.\n");
    logAlways("\n");
}

void Application::printLicense() const
{
    logAlways("\n");
    logAlways("Cura_SteamEngine version %s\n", VERSION);
    logAlways("Copyright (C) 2018 Ultimaker\n");
    logAlways("\n");
    logAlways("This program is free software: you can redistribute it and/or modify\n");
    logAlways("it under the terms of the GNU Affero General Public License as published by\n");
    logAlways("the Free Software Foundation, either version 3 of the License, or\n");
    logAlways("(at your option) any later version.\n");
    logAlways("\n");
    logAlways("This program is distributed in the hope that it will be useful,\n");
    logAlways("but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
    logAlways("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
    logAlways("GNU Affero General Public License for more details.\n");
    logAlways("\n");
    logAlways("You should have received a copy of the GNU Affero General Public License\n");
    logAlways("along with this program.  If not, see <http://www.gnu.org/licenses/>.\n");
}

void Application::slice()
{
    std::vector<std::string> arguments;
    for (size_t argument_index = 0; argument_index < argc; argument_index++)
    {
        arguments.emplace_back(argv[argument_index]);
    }

    communication = new CommandLine(arguments);
}

void Application::run(const size_t argc, char** argv)
{
    this->argc = argc;
    this->argv = argv;

    printLicense();
    Progress::init();

    if (argc < 2)
    {
        printHelp();
        exit(1);
    }

#pragma omp parallel
    {
#pragma omp master
        {
#ifdef _OPENMP
            log("OpenMP multithreading enabled, likely number of threads to be used: %u\n", omp_get_num_threads());
#else
            log("OpenMP multithreading disabled\n");
#endif
        }
    }

#ifdef ARCUS
    if (stringcasecompare(argv[1], "connect") == 0)
    {
        connect();
    }
    else
#endif //ARCUS
    if (stringcasecompare(argv[1], "slice") == 0)
    {
        slice();
    }
    else if (stringcasecompare(argv[1], "help") == 0)
    {
        printHelp();
    }
    else
    {
        logError("Unknown command: %s\n", argv[1]);
        printCall();
        printHelp();
        exit(1);
    }

    if (!communication)
    {
        //No communication channel is open any more, so either:
        //- communication failed to connect, or
        //- the engine was called with an unknown command or a command that doesn't connect (like "help").
        //In either case, we don't want to slice.
        exit(0);
    }
    while (communication->hasSlice())
    {
        communication->sliceNext();
    }
}

} //Cura namespace.