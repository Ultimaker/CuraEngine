// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/visual_debug/logger.h"

#include <fmt/chrono.h>
#include <fmt/format.h>

#include <spdlog/details/os.h>

#include "utils/string.h"

namespace cura::debug::details
{
bool LoggersImpl::Enabled()
{
    isString auto visual_debug = toLower( spdlog::details::os::getenv( "CURAENGINE_VISUALDEBUG" ));
    return visual_debug == "1" || visual_debug == "on" || visual_debug == "true";
}

std::filesystem::path LoggersImpl::VisualDebugPath()
{
    if ( now_.empty())
    {
        now_ = fmt::format( "{:%Y%m%d_%H_%M}", std::chrono::system_clock::now());
    }
    auto vtu_dir = spdlog::details::os::getenv( "CURAENGINE_VTU_DIR" );
    namespace fs = std::filesystem;
    auto vtu_path = vtu_dir.empty() ? fs::current_path().append( fmt::format( "visual_debug/{}", now_ )) : fs::path( vtu_dir ).append( now_ );
    if ( !fs::is_directory( vtu_path ))
    {
        spdlog::error( "CURAENGINE_VTU_DIR should be a directory" );
    }
    if ( !fs::exists( vtu_path ))
    {
        fs::create_directories( vtu_path );
    }
    return vtu_path;
}
} // namespace cura::debug::details