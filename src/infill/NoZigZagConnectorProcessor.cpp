/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "../utils/macros.h"

#include "NoZigZagConnectorProcessor.h"


namespace cura 
{

void NoZigZagConnectorProcessor::registerVertex(const Point& vertex)
{
    UNUSED_PARAM(vertex);
}

void NoZigZagConnectorProcessor::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{
    UNUSED_PARAM(intersection);
    UNUSED_PARAM(scanline_is_even);
}

void NoZigZagConnectorProcessor::registerPolyFinished()
{

}



} // namespace cura 
