/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSOR_H
#define TEXTURE_PROCESSOR_H

#include <vector>

#include "slicer/Slicer.h"
#include "mesh.h"

namespace cura
{

class TextureProcessor
{
public:
    /*!
     * Apply offsets in the xy plane corresponding to pixel intensities
     */
    static void processBumpMap(Mesh* mesh, SlicerLayer& layer);

    /*!
     * Apply a zigzag pattern with offsets corresponding to pixel intensities
     */
    static void processDualColorTexture(Mesh* mesh, SlicerLayer& layer);
protected:
    static void process(Mesh* mesh, SlicerLayer& layer, bool dual_color_texture);
    
};

} // namespace cura

#endif // TEXTURE_PROCESSOR_H