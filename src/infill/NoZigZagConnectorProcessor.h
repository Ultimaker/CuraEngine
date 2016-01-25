/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H
#define INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H

#include "../utils/polygon.h"
#include "ZigzagConnectorProcessor.h"

namespace cura
{

class NoZigZagConnectorProcessor : public ZigzagConnectorProcessor
{
public:
    NoZigZagConnectorProcessor(const PointMatrix& rotation_matrix, Polygons& result)
    : ZigzagConnectorProcessor(rotation_matrix, result)
    {
    }

    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};


} // namespace cura


#endif // INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H