// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ReadTestSettings.h"
#include <cstdio>
#include <spdlog/spdlog.h>

namespace cura
{

bool readTestSettings(const std::string& filename, Settings& settings)
{
    spdlog::info("filename: {}", filename);

    FILE* handle = std::fopen(filename.c_str(), "r");
    if (! handle)
    {
        spdlog::error("Failed to open file: {}", filename);
        return false;
    }

    while (true)
    {
        char key[100];
        char value[100];

        int read = std::fscanf(handle, "%[a-zA-Z0-9_]=%[][a-zA-Z0-9_-.,:()/; ]\n", key, value);

        if (read == EOF)
        {
            break;
        }
        else if (read <= 0)
        {
            return false;
        }

        settings.add(std::string(key), std::string(value));
    }

    return true;
}
} // namespace cura