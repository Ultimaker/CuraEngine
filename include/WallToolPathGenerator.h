// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_WALLTOOLPATHGENERATOR_H
#define CURAENGINE_WALLTOOLPATHGENERATOR_H

namespace cura
{

enum class WallToolPathGenerator
{
    Arachne, // Use arachne to generate smart extrusion tool paths with variable line width
    NaiveInset // Use a naive implementation to generate tool paths with constant line width
};

}

#endif
