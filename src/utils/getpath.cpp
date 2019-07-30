#include "getpath.h"

#include <iostream>

#ifdef _WIN32
#include <windows.h> // GetFullPathNameA
#else
#include <libgen.h> // dirname
#include <cstring> // std::strcpy
#endif

namespace cura
{
std::string getPathName(const std::string& filePath) {
#ifdef _WIN32
    char buffer[MAX_PATH];
    char* name_start;
    DWORD path_size = GetFullPathNameA(filePath.c_str(), static_cast<DWORD>(MAX_PATH), buffer, &name_start);
    if (path_size == 0)
    {
        std::cerr << "Failed to get full path for [" << filePath.c_str() << "]" << std::endl;
        exit(1);
    }
    std::string folder_name{name_start, path_size};
#else
    char buffer[filePath.size()];
    std::strcpy(buffer, filePath.c_str()); // copy the string because dirname(.) changes the input string!!!
    std::string folder_name{dirname(buffer)};
#endif
    return folder_name;
}
}