/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSOR_H
#define TEXTURE_PROCESSOR_H

#include <vector>

#include "slicer/Slicer.h"

namespace cura
{

class TextureProcessor
{
public:
    static void process(std::vector<Slicer*>& slicerList);
};

} // namespace cura

#endif // TEXTURE_PROCESSOR_H