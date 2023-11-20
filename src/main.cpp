// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <iostream> //To change the formatting of std::cerr.
#include <signal.h> //For floating point exceptions.
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <sys/resource.h> //For setpriority.
#endif

#include <sentry.h>

#include <spdlog/spdlog.h>

#include "Application.h"

namespace cura
{

// Signal handler for a "floating point exception", which can also be integer division by zero errors.
void signal_FPE(int n)
{
    (void)n;
    spdlog::error("Arithmetic exception.");
    exit(1);
}

} // namespace cura

int main(int argc, char** argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    // Lower the process priority on linux and mac. On windows this is done on process creation from the GUI.
    setpriority(PRIO_PROCESS, 0, 10);
#endif

#ifndef DEBUG
    // Register the exception handling for arithmetic exceptions, this prevents the "something went wrong" dialog on windows to pop up on a division by zero.
    signal(SIGFPE, cura::signal_FPE);
#endif
    std::cerr << std::boolalpha;

    // Setup sentry error handling.
    sentry_options_t* options = sentry_options_new();
    // TODO: Right now we just hardcode the key. We should probably get that from some kind of secret for release builds
    sentry_options_set_dsn(options, "https://734f9ec9024f73e53701d59c3ffddfe3@o323038.ingest.sentry.io/4506257745510401");
    // This is also the default-path. For further information and recommendations:
    // https://docs.sentry.io/platforms/native/configuration/options/#database-path
    sentry_options_set_database_path(options, ".sentry-native");
    // TODO: Hardcoded the version number, we should also get that from somewhere. Can't be bothered to figure that out now
    sentry_options_set_release(options, "curaengine@1.0.0");
    sentry_options_set_debug(options, 1);
    sentry_init(options);

    cura::Application::getInstance().run(argc, argv);

    sentry_close();

    return 0;
}
