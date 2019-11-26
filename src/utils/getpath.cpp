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
    char buffer[MAX_PATH];  // This buffer will hold the complete path.
    char* file_name_start;  // This is the pointer to the start of the file name.
    DWORD path_size = GetFullPathNameA(filePath.c_str(), static_cast<DWORD>(MAX_PATH), buffer, &file_name_start);
    if (path_size == 0)
    {
        std::cerr << "Failed to get full path for [" << filePath.c_str() << "]" << std::endl;
        exit(1);
    }
    // Only take the directory part of
    DWORD dir_path_size = path_size - (path_size - (file_name_start - buffer));
    std::string folder_name{buffer, dir_path_size};
#else
    char buffer[filePath.size()];
    std::strcpy(buffer, filePath.c_str()); // copy the string because dirname(.) changes the input string!!!
    std::string folder_name{dirname(buffer)};
#endif
    return folder_name;
}
}