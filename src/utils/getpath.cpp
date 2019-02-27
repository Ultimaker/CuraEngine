#include "getpath.h"

#include <vector>

#ifdef _WIN32
#include <windows.h> // GetFullPathNameA
#else
#include <libgen.h> // dirname
#include <cstring> // std::strcpy
#endif

namespace cura
{
std::string getPathName(const std::string& filePath) {
    std::vector<char> buffer(filePath.size() + 1);
#ifdef _WIN32
    char* name_start;
    GetFullPathNameA(filePath.c_str(), static_cast<DWORD>(buffer.size()), buffer.data(), &name_start);
    std::string folder_name{buffer.data(), name_start};
#else
    std::strcpy(buffer.data(), filePath.c_str()); // copy the string because dirname(.) changes the input string!!!
    std::string folder_name{dirname(buffer.data())};
#endif
    return folder_name;
}
}