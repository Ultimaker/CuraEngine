//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_STL_H
#define UTILS_STL_H

#include <stdio.h> // for file output

#include "floatpoint.h"
#include "logoutput.h"
#include "NoCopy.h"

namespace cura {

/*
 * Writing STL to file
 * 
 * mainly used for visualization purposes of 3D algorithms
 */
class STL : NoCopy
{
public:
private:

    FILE* out; // the output file
    char filename[1024];

public:
    STL(const std::string filename);

    ~STL();
    
    void writeFace(FPoint3 a, FPoint3 b, FPoint3 c);

    template<typename... Args>
    void printf(const char* txt, Args&&... args);

};

template<typename... Args>
void STL::printf(const char* txt, Args&&... args)
{
    fprintf(out, txt, args...);
}

} // namespace cura
#endif // UTILS_STL_H
