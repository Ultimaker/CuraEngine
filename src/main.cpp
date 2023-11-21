// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <iostream> //To change the formatting of std::cerr.
#include <signal.h> //For floating point exceptions.
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <sys/resource.h> //For setpriority.
#endif

#ifdef SENTRY_URL
#include <filesystem>
#include <sentry.h>

#include <fmt/format.h>
#endif

#include <string>

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
// Want to set the sentry URL? Use '-c user.curaengine:sentry_url=<url> -o curaengine:enable_sentry=True' with conan install
#ifdef SENTRY_URL
    // Setup sentry error handling.
    sentry_options_t* options = sentry_options_new();
    sentry_options_set_dsn(options, std::string(SENTRY_URL).c_str());
    // This is also the default-path. For further information and recommendations:
    // https://docs.sentry.io/platforms/native/configuration/options/#database-path
#if defined(__linux__)
    const auto config_path = std::filesystem::path(std::getenv("HOME")).append("/.local/share/cura/.sentry-native");
#elif defined(__APPLE__) && defined(__MACH__)
    const auto config_path = std::filesystem::path(std::getenv("HOME")).append("/Library/Application Support/cura/.sentry-native");
#elif defined(_WIN64)
    const auto config_path = std::filesystem::path(std::getenv("APPDATA")).append("/cura/.sentry-native");
#endif
    sentry_options_set_database_path(options, config_path.c_str());
    sentry_options_set_release(options, fmt::format("curaengine@{}", CURA_ENGINE_VERSION).c_str());
    sentry_init(options);
#endif

    cura::Application::getInstance().run(argc, argv);

#ifdef SENTRY_URL
    sentry_close();
#endif

    return 0;
}
