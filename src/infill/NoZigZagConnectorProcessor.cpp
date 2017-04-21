//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/macros.h"

#include "NoZigZagConnectorProcessor.h"


namespace cura 
{

void NoZigZagConnectorProcessor::registerVertex(const Point&)
{
    //No need to add anything.
}

void NoZigZagConnectorProcessor::registerScanlineSegmentIntersection(const Point&, bool)
{
    //No need to add anything.
}

void NoZigZagConnectorProcessor::registerPolyFinished()
{

}



} // namespace cura 
