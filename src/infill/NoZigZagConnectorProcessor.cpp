// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/NoZigZagConnectorProcessor.h"

#include "utils/macros.h"


namespace cura
{

void NoZigZagConnectorProcessor::registerVertex(const Point&)
{
    // No need to add anything.
}

void NoZigZagConnectorProcessor::registerScanlineSegmentIntersection(const Point&, int)
{
    // No need to add anything.
}

void NoZigZagConnectorProcessor::registerPolyFinished()
{
    // No need to add anything.
}

} // namespace cura
