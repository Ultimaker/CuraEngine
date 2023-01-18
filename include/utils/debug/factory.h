// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_FACTORY_H
#define CURAENGINE_FACTORY_H

#include <memory>

#include <fmt/format.h>
#include <spdlog/cfg/helpers.h>
#include <spdlog/details/os.h>

#include "utils/concepts/geometry.h"
#include "utils/debug/visual_logger.h"
#include "utils/string.h"

namespace cura::debug::details
{
template<typename... Args>
[[nodiscard]] shared_visual_logger make_visual_logger(std::string_view id, Args&&... args)
{
    isString auto visual_debug = toLower(spdlog::details::os::getenv("CURAENGINE_VISUALDEBUG"));
    if (visual_debug == "1" || visual_debug == "on" || visual_debug == "true")
    {
        namespace fs = std::filesystem;
        auto now = std::chrono::system_clock::now();
        auto current_dir_name = fmt::format("{:%Y%m%d_%H_%M}", now);

        auto vtu_dir = spdlog::details::os::getenv("CURAENGINE_VTU_DIR");
        auto vtu_path = vtu_dir.empty() ? fs::current_path().append(fmt::format("visual_debug/{}", current_dir_name)) : fs::path(vtu_dir).append(current_dir_name);
        if (! fs::is_directory(vtu_path))
        {
            spdlog::error("CURAENGINE_VTU_DIR should be a directory");
        }
        if (! fs::exists(vtu_path))
        {
            fs::create_directories(vtu_path);
        }
        return std::make_shared<VisualLogger>(id, vtu_path, std::forward<Args>(args)...);
    }
    return std::make_shared<VisualLogger>(id);
}
} // namespace cura::debug::details

#endif // CURAENGINE_FACTORY_H
